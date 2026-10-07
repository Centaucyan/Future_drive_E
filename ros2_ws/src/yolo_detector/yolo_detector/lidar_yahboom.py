#!/usr/bin/env python3
from networkx.generators import spectral_graph_forge
from matplotlib import backend_managers
from aiohttp import client_exceptions
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,CompressedImage,LaserScan
from vision_msgs.msg import Detection2DArray,Detection2D,ObjectHypothesisWithPose
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2,json,time,math
import struct

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameter('video_source','0')
        self.declare_parameter('model_name','models/futuredrive_yolo26n_best.pt')
        self.declare_parameter('confidence_threshold',0.5)
        self.declare_parameter('device','cpu')
        self.declare_parameter('input_topic','/image_raw/compressed')
        self.declare_parameter('max_det',50)

        self.declare_parameter('image_width',640)
        self.declare_parameter('image_height',480)
        self.declare_parameter('fx',438.783367)
        self.declare_parameter('fy',437.302876)
        self.declare_parameter('cx',305.593336)
        self.declare_parameter('cy',243.738352)
        self.declare_parameter('fov_x',72.0)
        self.declare_parameter('fov_y',57.5)
        self.declare_parameter('height_m',0.085)
        self.declare_parameter('pitch',0.0)

        self.declare_parameter('lidar_angle_offset_deg',0.0)
        self.declare_parameter('lidar_window_deg',3.0)
        self.declare_parameter('real_height_four_wheeler',1.5)
        self.declare_parameter('real_height_two_wheeler',1.2)
        self.declare_parameter('real_height_person',1.7)
        self.declare_parameter('min_distance',0.1)
        self.declare_parameter('max_distance',50.0)

        model_name=self.get_parameter('model_name').value
        device=self.get_parameter('device').value
        input_topic=self.get_parameter('input_topic').value
        self.conf_threshold=self.get_parameter('confidence_threshold').value
        self.max_det=self.get_parameter('max_det').value

        self.image_width=self.get_parameter('image_width').value
        self.image_height=self.get_parameter('image_height').value
        self.fx=self.get_parameter('fx').value
        self.fy=self.get_parameter('fy').value
        self.cx=self.get_parameter('cx').value
        self.cy=self.get_parameter('cy').value
        self.fov_x=self.get_parameter('fov_x').value
        self.fov_y=self.get_parameter('fov_y').value
        self.camera_height=self.get_parameter('height_m').value
        self.camera_pitch_deg=self.get_parameter('pitch').value

        self.lidar_angle_offset_deg=self.get_parameter('lidar_angle_offset_deg').value
        self.lidar_window_deg=self.get_parameter('lidar_window_deg').value

        self.real_heights={
            0:self.get_parameter('real_height_four_wheeler').value,
            1:self.get_parameter('real_height_two_wheeler').value,
            2:self.get_parameter('real_height_person').value
        }
        self.min_dist=self.get_parameter('min_distance').value
        self.max_dist=self.get_parameter('max_distance').value

        self.get_logger().info(f'🔄 YOLO 모델 로딩 중: {model_name}')
        self.model=YOLO(model_name)
        self.model.to(device)
        self.get_logger().info(f'✅ YOLO 모델 로드 완료 (디바이스: {device})')

        self.bridge=CvBridge()
        self.latest_scan=None
        self.previous_distances={}
        self.previous_time=time.time()

        self.scan_sub=self.create_subscription(LaserScan,'/scan',self.scan_callback,10)
        self.subscription=self.create_subscription(CompressedImage,input_topic,self.image_callback,10)

        self.detection_pub=self.create_publisher(Detection2DArray,'/yolo/detections',10)
        self.json_pub=self.create_publisher(String,'/yolo/detections_json',10)
        self.result_image_pub=self.create_publisher(Image,'/yolo/result_image',10)
        self.collision_warning_pub=self.create_publisher(String,'/collision_warning',10)

        self.inference_count=0
        self.total_inference_time=0.0

        self.get_logger().info(f'📥 구독 토픽: {input_topic}')
        self.get_logger().info('📤 발행 토픽: /yolo/detections, /yolo/detections_json, /yolo/result_image, /collision_warning')
        self.get_logger().info('🚀 YoloDetector 노드 시작!')

    def scan_callback(self,msg):
        self.latest_scan=msg

    def get_lidar_distance(self,bbox_center_x,image_width):
        if self.latest_scan is None:
            return None

        scan=self.latest_scan

        camera_angle_deg=((bbox_center_x/image_width)-0.5)*self.fov_x
        lidar_angle_deg=camera_angle_deg+self.lidar_angle_offset_deg

        angle_min=math.radians(lidar_angle_deg-self.lidar_window_deg)
        angle_max=math.radians(lidar_angle_deg+self.lidar_window_deg)

        start_idx=int((angle_min-scan.angle_min)/scan.angle_increment)
        end_idx=int((angle_max-scan.angle_min)/scan.angle_increment)

        distances=[]

        for i in range(start_idx,end_idx+1):
            if 0<=i<len(scan.ranges):
                r=scan.ranges[i]
                if math.isfinite(r) and scan.range_min<=r<=scan.range_max:
                    distances.append(r)

        return min(distances) if distances else None

    def estimate_monocular_distance(self,class_id,bbox):
        x1,y1,x2,y2=bbox
        h_px=max(1.0,y2-y1)
        real_h=self.real_heights.get(class_id,1.0)

        pitch_rad=math.radians(self.camera_pitch_deg)
        angle_alpha=math.atan((y2-self.cy)/self.fy)
        total_angle=pitch_rad+angle_alpha

        if total_angle>math.radians(0.5):
            distance=self.camera_height/math.tan(total_angle)
        else:
            distance=(self.fy*real_h)/h_px

        return float(max(self.min_dist,min(self.max_dist,distance)))

    def image_callback(self,msg):
        try:
            np_arr=np.frombuffer(msg.data,np.uint8)
            cv_image=cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
            if cv_image is None:
                self.get_logger().error('❌ CompressedImage 디코딩 실패')
                return
        except Exception as e:
            self.get_logger().error(f'❌ 이미지 변환 실패: {e}')
            return

        start_time=time.time()
        results=self.model(
            cv_image,
            conf=self.conf_threshold,
            max_det=self.max_det,
            verbose=False
        )
        inference_time=time.time()-start_time

        self.inference_count+=1
        self.total_inference_time+=inference_time

        result=results[0]
        boxes=result.boxes

        detection_array_msg=Detection2DArray()
        detection_array_msg.header=msg.header
        json_detections=[]
        annotated_image=cv_image.copy()

        collision_candidates = []

        if boxes is not None and len(boxes)>0:                                         
            for box in boxes:
                x1,y1,x2,y2=box.xyxy[0].cpu().numpy()
                cx=(x1+x2)/2.0
                cy=(y1+y2)/2.0
                w=x2-x1
                h=y2-y1

                class_id=int(box.cls[0].cpu().numpy())
                confidence=float(box.conf[0].cpu().numpy())
                class_name=self.model.names[class_id]

                distance=self.get_lidar_distance(cx,cv_image.shape[1])

                monocular_distance=self.estimate_monocular_distance(
                    class_id,
                    [float(x1),float(y1),float(x2),float(y2)]
                )

                # 충돌 판단용 거리
                warning_distance = distance
                image_center = cv_image.shape[1] / 2.0

                # 화면 중앙 ± 20% 정도를 전방 영역으로 설정
                front_width = cv_image.shape[1] * 0.20
                is_front = abs(cx - image_center) <= front_width
                collision_distance_threshold = 0.5
                danger = (
                    is_front
                    and warning_distance is not None
                    and warning_distance < collision_distance_threshold
                )
                
                # 상대속도 계산
                current_time = time.time()

                track_key = f"{class_id}_{int(cx / 50)}"

                relative_speed = 0.0

                if (warning_distance is not None
                    and track_key in self.previous_distances):
                    previous_distance, previous_time = self.previous_distances[track_key]

                    dt = current_time - previous_time

                    if dt > 0.001:
                        relative_speed = (  
                            warning_distance - previous_distance
                        ) / dt

                if warning_distance is not None:
                    self.previous_distances[track_key] = (
                        warning_distance,
                        current_time
                    )

                # 위험 객체만 후보에 저장
                if danger:
                    collision_candidates.append({
                        'distance': warning_distance,
                        'relative_speed': relative_speed,
                        'object_class': class_name
                    })

                # Detection 처리                
                detection=Detection2D()
                detection.bbox.center.position.x=float(cx)
                detection.bbox.center.position.y=float(cy)
                detection.bbox.size_x=float(w)
                detection.bbox.size_y=float(h)

                hypothesis=ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id=str(class_id)
                hypothesis.hypothesis.score=confidence
                detection.results.append(hypothesis)
                detection_array_msg.detections.append(detection)

                json_detections.append({
                    'class_id':class_id,
                    'class_name':class_name,
                    'confidence':round(confidence,3),
                    'bbox':{
                        'x1':round(float(x1),1),
                        'y1':round(float(y1),1),
                        'x2':round(float(x2),1),
                        'y2':round(float(y2),1)
                    },
                    'center':{
                        'x':round(float(cx),1),
                        'y':round(float(cy),1)
                    },
                    'lidar_distance_m':round(distance,2) if distance is not None else None,
                    'monocular_distance_m':round(monocular_distance,2)
                })

                color=self._get_color(class_id)
                cv2.rectangle(
                    annotated_image,
                    (int(x1),int(y1)),
                    (int(x2),int(y2)),
                    color,2
                )

                label=f'{class_name} {confidence:.2f}'
                if distance is not None:
                    label+=f' | L:{distance:.2f}m'
                label+=f' | M:{monocular_distance:.2f}m'

                label_size,_=cv2.getTextSize(
                    label,cv2.FONT_HERSHEY_SIMPLEX,0.6,2)

                cv2.rectangle(
                    annotated_image,
                    (int(x1),int(y1)-label_size[1]-10),
                    (int(x1)+label_size[0],int(y1)),
                    color,-1
                )

                cv2.putText(
                    annotated_image,
                    label,
                    (int(x1),int(y1)-5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,(255,255,255),2
                )
        
        if collision_candidates:
                most_dangerous = min(
                    collision_candidates,
                    key=lambda x: x['distance']
                )
                warning_msg = String()
                warning_msg.data = json.dumps({
                    'danger': True,
                    'distance': round(
                        float(most_dangerous['distance']), 2
                    ),
                    'relative_speed': round(
                        float(most_dangerous['relative_speed']), 2
                    ),
                    'object_class': most_dangerous['object_class']
                }, ensure_ascii=False)

                self.collision_warning_pub.publish(warning_msg)
                self.get_logger().warn(f'⚠️ 위험 감지! 물체: {most_dangerous["object_class"]}, 거리: {most_dangerous["distance"]:.2f}m, 속도: {most_dangerous["relative_speed"]:.2f}m/s')

        else:
            warning_msg = String()
            warning_msg.data = json.dumps({
                'danger': False,
                'distance': -1.0,
                'relative_speed': 0.0,
                'object_class': ''
            }, ensure_ascii=False)

            self.collision_warning_pub.publish(warning_msg)

        fps=1.0/inference_time if inference_time>0 else 0
        fps_text=f'FPS: {fps:.1f} | Objects: {len(json_detections)}'

        cv2.putText(
            annotated_image,
            fps_text,
            (10,30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,(0,255,0),2
        )

        self.detection_pub.publish(detection_array_msg)

        json_msg=String()
        json_msg.data=json.dumps({
            'timestamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
            'inference_time_ms':round(inference_time*1000,1),
            'num_detections':len(json_detections),
            'detections':json_detections
        },ensure_ascii=False)
        self.json_pub.publish(json_msg)

        result_msg=self.bridge.cv2_to_imgmsg(
            annotated_image,encoding='bgr8')
        result_msg.header=msg.header
        self.result_image_pub.publish(result_msg)

        if self.inference_count%30==0:
            avg_time=self.total_inference_time/self.inference_count
            self.get_logger().info(
                f'📊 추론 #{self.inference_count} | '
                f'검출: {len(json_detections)}개 | '
                f'추론시간: {inference_time*1000:.1f}ms | '
                f'평균: {avg_time*1000:.1f}ms | '
                f'FPS: {1.0/avg_time:.1f}'
            )

    def _get_color(self,class_id):
        colors=[
            (255,0,0),(0,255,0),(0,0,255),
            (255,255,0),(255,0,255),(0,255,255),
            (128,0,0),(0,128,0),(0,0,128),
            (128,128,0),(128,0,128),(0,128,128),
            (255,128,0),(255,0,128),(128,255,0),
            (0,255,128),(128,0,255),(0,128,255),
            (255,128,128),(128,255,128)
        ]
        return colors[class_id%len(colors)]

    
def main(args=None):
    rclpy.init(args=args)
    node=YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 사용자에 의해 종료됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
