#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Pose


class LaneMapPublisher(Node):
    def __init__(self):
        super().__init__('lane_map_publisher')

        self.bridge=CvBridge()
        self.occupancy_grid_size=200
        self.vertical_offset=5
        self.height_offset=58
        self.captured_width=70
        self.captured_height=50

        self.frame_rate=10.0

        self.preview_hub=self.create_publisher(Image, "/preview", 10)
        self.image_pub=self.create_publisher(Image, "/lane_map", 10)

        self.header=Header()
        self.header.frame_id="base_link"

        self.map_info=MapMetaData()
        self.map_info.width=self.occupancy_grid_size
        self.map_info.height=100
        self.map_info.resolution=0.1
        self.map_info.origin.position.x = -10.0
        self.map_info.origin.position.y = -10.0

        self.cam=None
        self.imshowout=None

        self.create_timer(1.0 / self.frame_rate, self.publish_maps)


class PerspectiveTransform:
    def __init__(self, camera_angle):

        self.top_trim_ratio=0.0
        self.camera_angle=camera_angle
        
        self.horizontal_corner_cut_ratio = 0.26

        self.output_img_shape_x = 640
        self.output_img_shape_y = 480

    def cal_trim_ratio(self):
        return 0.25*(self.camera_angle)/100
    
    def trim_top_border(self, img):
        y_min = (int)(img.shape[0] * self.calc_trim_ratio())
        y_max = (int)(img.shape[0])
        x_min = 0
        x_max = (int)(img.shape[1])

        return img[y_min:y_max, x_min:x_max]
    
    def convert_to_flat(self, img):
        
        # Define the trapezoid to transform into rectangle
        top_left = (int)(img.shape[1] * self.horizontal_corner_cut_ratio), (int)(img.shape[0])
        top_right = (int)(img.shape[1] - img.shape[1] * self.horizontal_corner_cut_ratio), (int)(img.shape[0])
        bottom_left = 0, 0
        bottom_right = (int)(img.shape[1]), 0

        src_pts = np.float32([[top_left], [top_right], [bottom_left], [bottom_right]])
        dest_pts = np.float32([ [0, self.output_img_shape_y], [self.output_img_shape_x, self.output_img_shape_y] ,[0, 0], [self.output_img_shape_x, 0]])

        matrix = cv2.getPerspectiveTransform(dest_pts, src_pts)
        output = cv2.warpPerspective(img, matrix, (self.output_img_shape_x, self.output_img_shape_y))

        return output

transform = PerspectiveTransform(0)

def grass_filter(original_image):
    img=cv2.cvtColor(original_image, cv2.COLOR_BGR2HSV)
    lower=np.array([0,0,40])
    upper=np.array([255,140,210])

    mask=cv2.inRange(img,lower,upper)
    mask=255-mask
    return mask

def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, vertices, 255)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

class CameraNode(Node):
    def __init__(self,start_time,):
        super().__init__('camera_node')
        self.img_num=0
        self.start_time=None
        self.imshowout=None
        self.bridge=CvBridge()

        self.cam=cv2.VideoCapture(1,cv2.CAP_V4L2)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH,640)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.create_timer(1/10, self.process_camera_image)

    def process_camera_image(self):
        read_success, image=self.cam.read()

        if  not read_success:
            self.get_logger().warn("Camera Read Failed")
            return 
        blurred_image=cv2.GaussianBlur(image, (5,5),0)
        blurred_image=cv2.GaussianBlur(blurred_image,(5,5),0)

        filtered_image=grass_filter(blurred_image)

        height,width=image.shape[:2]
        region_of_interest_vertices=[(0,height),(width/2, height/2+120),
                                     (width,height)]
        
        gray_image=filtered_image
        cropped_image=region_of_interest(gray_image,np.array([region_of_interest_vertices]),np.int32)
        blurred=cv2.GaussianBlur(cropped_image,(7,7),0)
        blurred[blurred<245]=0

        perpsective_crop = transform.trim_top_border(blurred)
        perspective_warp = transform.convert_to_flat(perpsective_crop)

        self.publish_preview(image,perpsective_crop)

        self.numpy_to_occupancy_grid(perspective_warp)

    def publish_preview(self, original_image, processed_image):
        try:
            # Prepare the image message for preview
            preview_msg = self.bridge.cv2_to_imgmsg(original_image, encoding="bgr8")
            self.preview_hub.publish(preview_msg)

            # Publish the processed image
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding="mono8")
            self.image_pub.publish(processed_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish images: {str(e)}")   

    def numpy_to_occupancy_grid(self, data_map):
        data_map = cv2.resize(data_map, dsize=(80, 80), interpolation=cv2.INTER_LINEAR) / 2
        flattened = list(data_map.flatten().astype(int))

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_link"

        msg = OccupancyGrid(header=header, info=self.map_info, data=flattened)
        self.image_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    lane_map_publisher = LaneMapPublisher()
    camera_node = CameraNode()

    rclpy.spin(lane_map_publisher)
    rclpy.spin(camera_node)

    # Cleanup
    camera_node.cam.release()
    rclpy.shutdown()


if __name__ == '__main__':
    main()