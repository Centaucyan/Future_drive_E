#!/usr/bin/env python3

import json
import math
import struct
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import (
    CompressedImage,
    Image,
    LaserScan,
    PointCloud2,
    PointField,
)
from std_msgs.msg import String
from ultralytics import YOLO
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameter('video_source','0')
        self.declare_parameter('model_name','models/futuredrive_yolo26n_best.pt')
        self.declare_parameter('confidence_threshold', 0.30)
        self.declare_parameter('four_wheeler_confidence', 0.75)
        self.declare_parameter('two_wheeler_confidence', 0.35)
        self.declare_parameter('person_confidence', 0.40)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('input_topic', '/image_raw/compressed')
        self.declare_parameter('max_det', 50)
        self.declare_parameter('imgsz', 416)

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
        self.conf_threshold = self.get_parameter(
            'confidence_threshold'
        ).value

        self.class_thresholds = {
            0: self.get_parameter(
                'four_wheeler_confidence'
            ).value,
            1: self.get_parameter(
                'two_wheeler_confidence'
            ).value,
            2: self.get_parameter(
                'person_confidence'
            ).value,
        }

        self.max_det = self.get_parameter('max_det').value
        self.imgsz = self.get_parameter('imgsz').value

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

        # person과 2wheel의 순간 오검출 제거용
        self.temporal_history = {
            1: [],
            2: [],
        }
        self.temporal_required_frames = 3
        self.temporal_iou_threshold = 0.20

        self.scan_sub=self.create_subscription(LaserScan,'/scan',self.scan_callback,10)
        self.subscription=self.create_subscription(CompressedImage,input_topic,self.image_callback,10)

        self.detection_pub=self.create_publisher(Detection2DArray,'/yolo/detections',10)
        self.json_pub=self.create_publisher(String,'/yolo/detections_json',10)
        self.result_image_pub=self.create_publisher(Image,'/yolo/result_image',10)
        self.result_compressed_image_pub = self.create_publisher(CompressedImage, "/yolo/result_image/compressed", 10)
        self.collision_warning_pub=self.create_publisher(String,'/collision_warning',10)
        self.yolo_obstacle_pub = self.create_publisher(
            PointCloud2,
            '/yolo/obstacles',
            10,
        )

        self.inference_count=0
        self.total_inference_time=0.0

        self.collision_distance_threshold = 0.2
        self.lidar_monocular_threshold = 2.0

        self.get_logger().info(f'📥 구독 토픽: {input_topic}')
        self.get_logger().info('📤 발행 토픽: /yolo/detections, /yolo/detections_json, /yolo/result_image, /yolo/result_image/compressed, /collision_warning')
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
        """ 단안 거리 추정 함수 """
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

    @staticmethod
    def calculate_iou(box_a, box_b):
        ax1, ay1, ax2, ay2 = box_a
        bx1, by1, bx2, by2 = box_b

        inter_x1 = max(ax1, bx1)
        inter_y1 = max(ay1, by1)
        inter_x2 = min(ax2, bx2)
        inter_y2 = min(ay2, by2)

        inter_w = max(0.0, inter_x2 - inter_x1)
        inter_h = max(0.0, inter_y2 - inter_y1)
        inter_area = inter_w * inter_h

        area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
        area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
        union_area = area_a + area_b - inter_area

        if union_area <= 0.0:
            return 0.0

        return inter_area / union_area

    def is_temporally_confirmed(self, class_id, bbox):
        history = self.temporal_history.get(class_id)

        if history is None:
            return True

        if len(history) < self.temporal_required_frames - 1:
            return False

        for previous_frame_boxes in history:
            matched = any(
                self.calculate_iou(bbox, previous_bbox)
                >= self.temporal_iou_threshold
                for previous_bbox in previous_frame_boxes
            )

            if not matched:
                return False

        return True

    @staticmethod
    def is_invalid_four_wheeler_box(
        class_id,
        bbox,
        image_width,
        image_height,
    ):
        if class_id != 0:
            return False

        x1, y1, x2, y2 = bbox
        box_width = max(1.0, x2 - x1)
        box_height = max(1.0, y2 - y1)
        box_area_ratio = (
            box_width * box_height
        ) / float(image_width * image_height)

        touches_left = x1 <= image_width * 0.01
        touches_top = y1 <= image_height * 0.01
        abnormally_vertical = box_height > box_width * 1.40
        sufficiently_large = box_area_ratio >= 0.08

        return (
            touches_left
            and touches_top
            and abnormally_vertical
            and sufficiently_large
        )

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

        # 클래스별 기준 중 가장 낮은 값으로 YOLO 후보를 먼저 받는다.
        # 이후 아래 박스 반복문에서 클래스별 기준을 다시 적용한다.
        min_confidence = min(self.class_thresholds.values())

        start_time = time.time()
        results = self.model(
            cv_image,
            conf=min_confidence,
            imgsz=self.imgsz,
            max_det=self.max_det,
            verbose=False,
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

        # 모델 내부 클래스명은 토픽과 학습 데이터 호환성을 위해 유지하고,
        # 검출 결과 영상에 표시할 때만 짧은 이름으로 변환한다.
        display_class_names = {
            'four_wheeler': '4wheel',
            'two_wheeler': '2wheel',
            'person': 'person',
        }

        collision_candidates = []
        yolo_obstacles = []

        current_temporal_boxes = {
            1: [],
            2: [],
        }

        if boxes is not None:
            for candidate_box in boxes:
                candidate_class_id = int(
                    candidate_box.cls[0].cpu().numpy()
                )
                candidate_confidence = float(
                    candidate_box.conf[0].cpu().numpy()
                )

                candidate_threshold = self.class_thresholds.get(
                    candidate_class_id,
                    self.conf_threshold,
                )

                if candidate_confidence < candidate_threshold:
                    continue

                if candidate_class_id not in current_temporal_boxes:
                    continue

                candidate_bbox = tuple(
                    float(value)
                    for value in candidate_box.xyxy[0].cpu().numpy()
                )
                current_temporal_boxes[
                    candidate_class_id
                ].append(candidate_bbox)

        confirmed_temporal_boxes = {
            1: [],
            2: [],
        }

        for temporal_class_id, current_boxes in (
            current_temporal_boxes.items()
        ):
            for current_bbox in current_boxes:
                if self.is_temporally_confirmed(
                    temporal_class_id,
                    current_bbox,
                ):
                    confirmed_temporal_boxes[
                        temporal_class_id
                    ].append(current_bbox)

            history = self.temporal_history[temporal_class_id]
            history.append(current_boxes)

            maximum_history = self.temporal_required_frames - 1

            if len(history) > maximum_history:
                history.pop(0)

        if boxes is not None and len(boxes)>0:
            for box in boxes:
                x1,y1,x2,y2=box.xyxy[0].cpu().numpy()
                cx=(x1+x2)/2.0
                cy=(y1+y2)/2.0
                w=x2-x1
                h=y2-y1

                class_id = int(box.cls[0].cpu().numpy())
                confidence = float(box.conf[0].cpu().numpy())
                class_name = self.model.names[class_id]

                class_threshold = self.class_thresholds.get(
                    class_id,
                    self.conf_threshold,
                )

                if confidence < class_threshold:
                    continue

                bbox = (
                    float(x1),
                    float(y1),
                    float(x2),
                    float(y2),
                )

                if self.is_invalid_four_wheeler_box(
                    class_id,
                    bbox,
                    cv_image.shape[1],
                    cv_image.shape[0],
                ):
                    continue

                if class_id in confirmed_temporal_boxes:
                    temporally_confirmed = any(
                        self.calculate_iou(
                            bbox,
                            confirmed_bbox,
                        ) >= 0.99
                        for confirmed_bbox in (
                            confirmed_temporal_boxes[class_id]
                        )
                    )

                    if not temporally_confirmed:
                        continue

                distance = self.get_lidar_distance(
                    cx,
                    cv_image.shape[1],
                )
                if distance is not None:
                    camera_angle_deg = (
                        (cx / cv_image.shape[1]) - 0.5
                    ) * self.fov_x

                    angle_rad = math.radians(
                        camera_angle_deg
                        + self.lidar_angle_offset_deg
                    )

                    obstacle_x = distance * math.cos(angle_rad)
                    obstacle_y = distance * math.sin(angle_rad)
                    yolo_obstacles.append(
                        (obstacle_x, obstacle_y, 0.0)
                    )

                monocular_distance=self.estimate_monocular_distance(
                    class_id,
                    [float(x1),float(y1),float(x2),float(y2)]
                )

                # 충돌 판단은 기존 코드와 동일하게 LiDAR 거리만 사용한다.
                # 단안 추정 거리는 화면 표시와 JSON 정보 제공에만 사용한다.
                warning_distance = distance

                image_center = cv_image.shape[1] / 2.0

                # 화면 중앙 ± 20% 정도를 전방 영역으로 설정
                front_width = cv_image.shape[1] * 0.20
                is_front = abs(cx - image_center) <= front_width
                danger = (
                    is_front
                    and warning_distance is not None
                    and warning_distance < 0.5
                )
                
                # 상대속도 계산
                current_time = time.time()

                track_key = f"{class_id}_{int(cx / 50)}"

                relative_speed = 0.0

                if (warning_distance is not None
                    and track_key in self.previous_distances):
                    previous_distance, previous_time = (
                        self.previous_distances[track_key]
                    )

                    dt = current_time - previous_time

                    if dt > 0.001:
                        relative_speed = (warning_distance - previous_distance)/dt

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

                display_name = display_class_names.get(
                    class_name,
                    class_name,
                )
                label = f'{display_name} {confidence:.2f}'

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

        obstacle_msg = self.create_yolo_obstacle_cloud(
            yolo_obstacles,
            msg.header.stamp,
        )
        self.yolo_obstacle_pub.publish(obstacle_msg)
        
        if collision_candidates:
                most_dangerous = min(
                    collision_candidates,
                    key=lambda x: x['distance']
                )
                warning_msg = String()
                warning_msg.data = json.dumps({
                    'danger': True,
                    'distance': round(float(most_dangerous['distance']), 2),
                    'relative_speed': round(float(most_dangerous['relative_speed']), 2),
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

        result_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
        result_msg.header = msg.header
        self.result_image_pub.publish(result_msg)

        # 압축 이미지 발행
        encoded_ok, encoded = cv2.imencode(".jpg", annotated_image)
        if encoded_ok:
            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = "jpeg"
            compressed_msg.data = encoded.tobytes()
            self.result_compressed_image_pub.publish(compressed_msg)
        else:
            self.get_logger().error("❌ YOLO 결과 이미지 JPEG 압축 실패")


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

    def create_yolo_obstacle_cloud(self, obstacles, stamp):
        """LiDAR가 측정한 객체 위치를 PointCloud2로 변환한다."""
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_footprint'
        msg.height = 1
        msg.width = len(obstacles)
        msg.fields = [
            PointField(
                name='x',
                offset=0,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name='y',
                offset=4,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name='z',
                offset=8,
                datatype=PointField.FLOAT32,
                count=1,
            ),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.data = b''.join(
            struct.pack('<fff', x, y, z)
            for x, y, z in obstacles
        )
        msg.is_dense = True
        return msg

    
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
