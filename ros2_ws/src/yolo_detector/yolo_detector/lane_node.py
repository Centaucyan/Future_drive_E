#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2,numpy as np,rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge


class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')

        # ROS2 Topic
        self.declare_parameter('image_topic','/camera/image_raw')
        self.declare_parameter('result_topic','/lane/result')
        self.image_topic=self.get_parameter('image_topic').value
        self.result_topic=self.get_parameter('result_topic').value

        # Threshold: HLS/LAB 색상 + Sobel X
        self.yellow_h_min=15
        self.yellow_h_max=35
        self.yellow_s_min=70
        self.yellow_l_min=70
        self.sobel_low=50
        self.sobel_high=255

        # Sliding Window
        self.nwindows=9
        self.window_margin=80
        self.minpix=50

        # Polynomial fitting
        self.min_fit_points=50
        self.max_fit_rmse=70.0

        # 차선 폭/위치 검증 및 프레임 간 변화 제한
        self.min_lane_width_ratio=0.15
        self.max_lane_width_ratio=0.85
        self.max_width_change_ratio=0.50
        self.max_offset_jump_ratio=0.20
        self.smooth_factor=0.7
        self.num_lane_points=20

        # 이전 프레임의 차선 정보
        self.prev_left_fit=None
        self.prev_right_fit=None
        self.prev_lane_width=None
        self.prev_offset=None
        self.failed_count=0

        # Perspective Transform 행렬
        self.M=None
        self.Minv=None
        self.transform_size=None
        self.bridge=CvBridge()

        # ROS2 Sub/Pub
        self.image_sub=self.create_subscription(Image,self.image_topic,self.image_callback,10)
        self.result_pub=self.create_publisher(Float32MultiArray,self.result_topic,10)

        self.get_logger().info('Lane detection started')

    def image_callback(self,msg):
        try:
            image=self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        result=self.detect(image)

        # /lane/result 기본 구성
        data=[
            float(result['valid']),
            result['lane_center_x'],
            result['lateral_offset'],
            result['left_bottom_x'],
            result['right_bottom_x']
        ]

        # 검출 성공 시 원본 영상 좌표의 좌/우 차선 점 추가
        if result['valid']:
            for lx,ly,rx,ry in zip(
                result['left_points_x'],result['left_points_y'],
                result['right_points_x'],result['right_points_y']
            ):
                data.extend([lx,ly,rx,ry])

        out=Float32MultiArray()
        out.data=[float(x) for x in data]
        self.result_pub.publish(out)

    def create_perspective_transform(self,w,h):
        """ 원본 영상의 사다리꼴 영역 -> 직사각형(Bird's-Eye View) """
        if self.transform_size==(w,h) and self.M is not None:return

        # 카메라 기준으로 캘리브레이션 필요
        src=np.float32([
            [w*.28,h*.95],[w*.45,h*.75],
            [w*.55,h*.75],[w*.75,h*.95]
        ])
        dst=np.float32([
            [w*.25,h],[w*.25,0],
            [w*.75,0],[w*.75,h]
        ])
        self.M=cv2.getPerspectiveTransform(src,dst)
        self.Minv=cv2.getPerspectiveTransform(dst,src)
        self.transform_size=(w,h)

    def threshold_lane(self,image):
        """ 차선 후보를 Binary 영상으로 생성 """
        h,w=image.shape[:2]

        # Gaussian Blur: 영상 노이즈 제거
        blurred=cv2.GaussianBlur(image,(5,5),0)

        # HLS 색상 정보
        hls=cv2.cvtColor(blurred,cv2.COLOR_BGR2HLS)
        h_chan,l_chan,s_chan=hls[:,:,0],hls[:,:,1],hls[:,:,2]

        # LAB B 채널: 노란색 검출 보조 [L:밝기/A:G-R/B:B-Y]
        lab=cv2.cvtColor(blurred,cv2.COLOR_BGR2LAB)
        b_lab=lab[:,:,2]

        # CLAHE: 밝기 대비 향상
        clahe=cv2.createCLAHE(clipLimit=2.5,tileGridSize=(8,8))
        l_clahe=clahe.apply(l_chan)

        # 영상 하단 도로 밝기를 기준으로 White threshold 자동 조절
        road_l=l_chan[int(h*.60):,:]
        mean_l,std_l=float(np.mean(road_l)),float(np.std(road_l))
        adaptive_white_min=max(120,min(165,int(mean_l+1.2*std_l)))

        # White / Yellow 차선 검출
        white_mask=(l_chan>=adaptive_white_min)|((l_clahe>=185)&(s_chan<=110))
        yellow_mask=(b_lab>=150)|((h_chan>=self.yellow_h_min)&(h_chan<=self.yellow_h_max)&(s_chan>=self.yellow_s_min)&(l_chan>=self.yellow_l_min))
        color_mask=white_mask|yellow_mask

        # Sobel X: 차선과 도로 사이의 밝기 변화 검출
        sobelx=cv2.Sobel(l_clahe,cv2.CV_64F,1,0,ksize=3)
        abs_sobel=np.absolute(sobelx)
        max_value=abs_sobel.max()

        if max_value>1e-6:
            scaled_sobel=np.uint8(255*abs_sobel/max_value)
        else:
            scaled_sobel=np.zeros_like(l_chan)

        gradient_mask=(scaled_sobel>=self.sobel_low)&(scaled_sobel<=self.sobel_high)&(l_clahe>=140)

        # 색상 검출 + 경계 검출
        combined=color_mask|gradient_mask
        binary=np.zeros_like(l_chan,dtype=np.uint8)
        binary[combined]=255

        # 작은 점 형태의 노이즈 제거
        kernel=cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
        return cv2.morphologyEx(binary,cv2.MORPH_OPEN,kernel)

    # -------------------------------------------------
    # Perspective Warp
    # -------------------------------------------------
    def perspective_warp(self,binary):
        h,w=binary.shape[:2]
        self.create_perspective_transform(w,h)
        return cv2.warpPerspective(binary,self.M,(w,h),flags=cv2.INTER_LINEAR)

    def histogram(self,binary_warped):
        """ 하단 영역의 차선 픽셀 분포로 좌/우 차선 시작점 탐색 """
        h,w=binary_warped.shape[:2]
        hist=np.sum(binary_warped[int(h*.50):,:]>0,axis=0)

        left_min_x,left_max_x=int(w*.08),int(w*.45)
        right_min_x,right_max_x=int(w*.55),int(w*.92)
        left_sub,right_sub=hist[left_min_x:left_max_x],hist[right_min_x:right_max_x]

        left_base=int(np.argmax(left_sub)+left_min_x) if len(left_sub)>0 and np.max(left_sub)>0 else int(w*.25)
        right_base=int(np.argmax(right_sub)+right_min_x) if len(right_sub)>0 and np.max(right_sub)>0 else int(w*.75)

        return hist,left_base,right_base

    def find_lane_pixels(self,binary_warped):
        """ 최초 검출 또는 이전 차선 정보가 없을 때 사용 """
        h,w=binary_warped.shape[:2]
        hist,left_base,right_base=self.histogram(binary_warped)
        nonzeroy,nonzerox=binary_warped.nonzero()
        window_height=max(1,h//self.nwindows)
        left_current,right_current=left_base,right_base
        left_lane_inds,right_lane_inds=[],[]

        for window in range(self.nwindows):
            win_y_low=h-(window+1)*window_height
            win_y_high=h-window*window_height
            lx1,lx2=left_current-self.window_margin,left_current+self.window_margin
            rx1,rx2=right_current-self.window_margin,right_current+self.window_margin

            good_left=((nonzeroy>=win_y_low)&(nonzeroy<win_y_high)&(nonzerox>=lx1)&(nonzerox<lx2)).nonzero()[0]
            good_right=((nonzeroy>=win_y_low)&(nonzeroy<win_y_high)&(nonzerox>=rx1)&(nonzerox<rx2)).nonzero()[0]
            left_lane_inds.append(good_left)
            right_lane_inds.append(good_right)

            if len(good_left)>self.minpix:left_current=int(np.mean(nonzerox[good_left]))
            if len(good_right)>self.minpix:right_current=int(np.mean(nonzerox[good_right]))

        left_lane_inds=np.concatenate(left_lane_inds) if left_lane_inds else np.array([],dtype=np.int32)
        right_lane_inds=np.concatenate(right_lane_inds) if right_lane_inds else np.array([],dtype=np.int32)

        return (
            nonzerox[left_lane_inds],nonzeroy[left_lane_inds],
            nonzerox[right_lane_inds],nonzeroy[right_lane_inds],
            hist,left_base,right_base
        )

    def search_around_previous_fit(self,binary_warped):
        """ 이전 프레임의 Polynomial 주변만 탐색 """
        nonzeroy,nonzerox=binary_warped.nonzero()
        left_fit,right_fit=self.prev_left_fit,self.prev_right_fit

        left_center=left_fit[0]*nonzeroy**2+left_fit[1]*nonzeroy+left_fit[2]
        right_center=right_fit[0]*nonzeroy**2+right_fit[1]*nonzeroy+right_fit[2]

        left_lane_inds=np.abs(nonzerox-left_center)<=self.window_margin
        right_lane_inds=np.abs(nonzerox-right_center)<=self.window_margin

        return (
            nonzerox[left_lane_inds],nonzeroy[left_lane_inds],
            nonzerox[right_lane_inds],nonzeroy[right_lane_inds]
        )

    def fit_lane(self,x,y,side,h):
        """ x = Ay² + By + C 형태로 fitting """
        if len(x)<self.min_fit_points:
            self.get_logger().debug(f'{side} FIT FAIL | points={len(x)}')
            return None

        y_span=float(np.max(y)-np.min(y)) if len(y)>0 else 0

        try:
            if y_span<h*.30:
                line_coeff=np.polyfit(y,x,1)
                coeff=np.array([0.,line_coeff[0],line_coeff[1]])
            else:
                coeff=np.polyfit(y,x,2)

                # 지나치게 큰 곡률은 직선으로 대체
                if abs(coeff[0])>1.5e-3:
                    line_coeff=np.polyfit(y,x,1)
                    coeff=np.array([0.,line_coeff[0],line_coeff[1]])
        except Exception as e:
            self.get_logger().debug(f'{side} FIT FAIL | {e}')
            return None

        fitted=np.polyval(coeff,y)
        rmse=float(np.sqrt(np.mean((x-fitted)**2)))

        # fitting 오차가 크면 잘못된 차선으로 판단
        if rmse>self.max_fit_rmse:
            self.get_logger().debug(f'{side} FIT FAIL | RMSE={rmse:.2f}')
            return None

        return coeff

    def lane_x(self,fit,y):
        return fit[0]*y*y+fit[1]*y+fit[2]

    def validate_lane(self,left_fit,right_fit,h,w):
        """ 좌/우 차선 순서, 차선 폭, 프레임 간 폭 변화 검증 """
        y_bottom=h-1
        left_bottom,right_bottom=self.lane_x(left_fit,y_bottom),self.lane_x(right_fit,y_bottom)
        lane_width=right_bottom-left_bottom
        width_ratio=lane_width/w

        # 좌/우 차선 반전 및 width 제한
        if left_bottom>=right_bottom or width_ratio<self.min_lane_width_ratio or width_ratio>self.max_lane_width_ratio:
            return False

        # width 변화 제한
        if self.prev_lane_width is not None:
            width_change=abs(lane_width-self.prev_lane_width)/max(self.prev_lane_width,1e-6)
            if width_change>self.max_width_change_ratio:return False

        return True

    def transform_point(self,x,y):
        """ Bird's-Eye View 좌표 → 원본 영상 좌표 """
        point=np.array([[[float(x),float(y)]]],dtype=np.float32)
        transformed=cv2.perspectiveTransform(point,self.Minv)
        return float(transformed[0,0,0]),float(transformed[0,0,1])

    def make_lane_points(self,left_fit,right_fit,h,w):
        """ Polynomial 차선을 원본 영상 좌표의 점들로 변환 """
        plot_y=np.linspace(0,h-1,self.num_lane_points)
        left_x,right_x=self.lane_x(left_fit,plot_y),self.lane_x(right_fit,plot_y)

        left_points=np.stack([left_x,plot_y],axis=1).astype(np.float32).reshape(-1,1,2)
        right_points=np.stack([right_x,plot_y],axis=1).astype(np.float32).reshape(-1,1,2)

        left_original=cv2.perspectiveTransform(left_points,self.Minv).reshape(-1,2)
        right_original=cv2.perspectiveTransform(right_points,self.Minv).reshape(-1,2)

        left_original[:,0]=np.clip(left_original[:,0],0,w-1)
        left_original[:,1]=np.clip(left_original[:,1],0,h-1)
        right_original[:,0]=np.clip(right_original[:,0],0,w-1)
        right_original[:,1]=np.clip(right_original[:,1],0,h-1)

        return (
            left_original[:,0].tolist(),left_original[:,1].tolist(),
            right_original[:,0].tolist(),right_original[:,1].tolist()
        )

    def invalid_result(self):
        """ 검출 실패가 5회 이상이면 이전 차선 정보 초기화 """
        self.failed_count+=1

        if self.failed_count>=5:
            self.prev_left_fit=None
            self.prev_right_fit=None
            self.prev_lane_width=None
            self.prev_offset=None

        return {
            'valid':False,
            'lane_center_x':0.0,
            'lateral_offset':0.0,
            'left_bottom_x':0.0,
            'right_bottom_x':0.0
        }

    def detect(self,image):
        """ 전체 차선 검출 """
        h,w=image.shape[:2]

        # 1. 차선 후보 생성
        binary=self.threshold_lane(image)

        # 2. Bird's-Eye View 변환
        warped=self.perspective_warp(binary)

        # 3. 차선 픽셀 탐색
        if self.prev_left_fit is not None and self.prev_right_fit is not None:
            leftx,lefty,rightx,righty=self.search_around_previous_fit(warped)

            # 이전 위치 주변에서 충분한 픽셀이 없으면 Sliding Window 재탐색
            if len(leftx)<self.min_fit_points or len(rightx)<self.min_fit_points:
                leftx,lefty,rightx,righty,_,_,_=self.find_lane_pixels(warped)
        else:
            leftx,lefty,rightx,righty,_,_,_=self.find_lane_pixels(warped)

        # 4. Polynomial fitting
        left_fit=self.fit_lane(leftx,lefty,'LEFT',h)
        right_fit=self.fit_lane(rightx,righty,'RIGHT',h)

        if left_fit is None or right_fit is None:
            return self.invalid_result()

        # 5. 좌/우 차선 검증
        if not self.validate_lane(left_fit,right_fit,h,w):
            return self.invalid_result()

        # 6. Bird's-Eye View 하단의 차선 위치
        y_bottom=h-1
        left_bottom_warp=float(self.lane_x(left_fit,y_bottom))
        right_bottom_warp=float(self.lane_x(right_fit,y_bottom))
        center_bottom_warp=(left_bottom_warp+right_bottom_warp)/2.0

        # 7. 원본 영상 좌표로 변환
        left_bottom_x,_=self.transform_point(left_bottom_warp,y_bottom)
        right_bottom_x,_=self.transform_point(right_bottom_warp,y_bottom)
        center_bottom_x,_=self.transform_point(center_bottom_warp,y_bottom)

        # 차선 중심과 영상 중심의 좌우 차이
        image_center=w/2.0
        lane_center_x=center_bottom_x
        lateral_offset=lane_center_x-image_center

        # 8. 프레임 간 Offset 급변 방지
        if self.prev_offset is not None:
            offset_jump=abs(lateral_offset-self.prev_offset)
            if offset_jump>w*self.max_offset_jump_ratio:
                return self.invalid_result()

        # 9. EMA smoothing
        if self.prev_left_fit is not None and self.prev_right_fit is not None:
            smooth_left_fit=self.smooth_factor*left_fit+(1.0-self.smooth_factor)*self.prev_left_fit
            smooth_right_fit=self.smooth_factor*right_fit+(1.0-self.smooth_factor)*self.prev_right_fit
        else:
            smooth_left_fit,smooth_right_fit=left_fit.copy(),right_fit.copy()

        lane_width_warp=right_bottom_warp-left_bottom_warp

        self.failed_count=0
        self.prev_left_fit=smooth_left_fit.copy()
        self.prev_right_fit=smooth_right_fit.copy()
        self.prev_lane_width=lane_width_warp
        self.prev_offset=lateral_offset

        # 10. 원본 영상 좌표의 차선 점 생성
        left_points_x,left_points_y,right_points_x,right_points_y=self.make_lane_points(
            smooth_left_fit,smooth_right_fit,h,w
        )

        return {
            'valid':True,
            'lane_center_x':float(lane_center_x),
            'lateral_offset':float(lateral_offset),
            'left_bottom_x':float(left_bottom_x),
            'right_bottom_x':float(right_bottom_x),
            'left_points_x':left_points_x,
            'left_points_y':left_points_y,
            'right_points_x':right_points_x,
            'right_points_y':right_points_y
        }

    def destroy_node(self):
        self.destroy_subscription(self.image_sub)
        self.destroy_publisher(self.result_pub)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node=LaneDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()