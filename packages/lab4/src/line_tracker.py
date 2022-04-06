#!/usr/bin/env python3

import time
import rospy
import cv2
from std_msgs.msg import Float32
from sensor_msgs.msg import Image	
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32


class LineTracker:
    '''This Class Runs A ROS Node For Tracking The Centroid Of The Path From A Given Camera Feed'''
    def __init__(self, debug=False):
        self.debug = debug
        self.last_reading = 0
        self.last_readings = [0 for x in range(10)]
        self.path_pub = rospy.Publisher("/csc22912/output/line_tracker", Float32)
        self.image_pub = rospy.Publisher("/csc22912/output/image_raw/compressed", CompressedImage)
        self.subscriber = rospy.Subscriber("/csc22912/camera_node/image/compressed", CompressedImage, self.imgCallback, queue_size=1)
    
    @staticmethod
    def filter_yellow(img):
        """Set Yellow To White And Everything Else To Black"""
        low = np.array([18, 75, 75])
        high = np.array([45, 255, 255])
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        return cv2.inRange(hsv, low, high)


    @staticmethod
    def compute_centroid(image, height, width):
        """Given A Binary Image, Compute The Centroid Of The Largest Contour."""
        x, y, count = 0, 0, 0
        contours, _ = cv2.findContours(image, 1, cv2.CHAIN_APPROX_SIMPLE)
        moments = [cv2.moments(x) for x in contours]
        rects = [cv2.boundingRect(x) for x in contours]
        for m, r in zip(moments, rects):
            if m['m00'] != 0:
                # if (not (r[0] < (0.25 * width) and r[1] < (0.25 * height))) and (not (r[0] > (width - 0.25 * width) and r[1] < (0.25 * height))):
                count += r[2] * r[3]
                x += r[2] * r[3] * int(m['m10'] / m['m00'])
                y += r[2] * r[3] * int(m['m01']/m['m00'])
        return (x // count, y // count) if count > 0 else (-1, -1)

    def imgCallback(self, ros_data):
        '''
        This Callback Runs Whenever A New Image Is Published From The Camera.
        We Use This To Detect The Centroid Of The Path As Well As Its Colour.
        '''
        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        
        # Slice Camera View
        vertical_cut = 255
        horizontal_cut = 75
        width = image_np.shape[1]
        height = image_np.shape[0] - vertical_cut
        image_np = image_np[vertical_cut:,horizontal_cut:width-horizontal_cut, :]
        # rospy.loginfo(image_np.shape)
        
        filtered_img = self.filter_yellow(image_np)
        x, y = self.compute_centroid(filtered_img, height, width)
        offset = (image_np.shape[1] // 2) - x
        scale = image_np.shape[1] // 2
        scaled_offset = offset / scale
        self.last_readings.pop()
        self.last_readings.append(scaled_offset)
        if x < 0:
            scaled_offset = 0.0
        elif x < 0 and (sum(self.last_readings) / 10.0) > 0:
            scaled_offset = 0.0
        if abs(self.last_reading - scaled_offset) > 0.5 and scaled_offset != 0:
            scaled_offset = self.last_reading
        self.last_reading = scaled_offset
        

        # Id Debug Is True, Show The Center Of The Path On The Camera Feed
        if self.debug:
            #rospy.loginfo(f"Track Offset: {scaled_offset}")
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            rospy.loginfo(f"Centroid: {scaled_offset}")
            msg.data = np.array(cv2.imencode('.jpg', filtered_img)[1]).tostring()
            self.image_pub.publish(msg)

        # Publish The Centroid Of The Path
        track_msg = Float32(scaled_offset)
        self.path_pub.publish(track_msg)

    def run(self):
        '''Run The Node'''
        rospy.spin()


def main():
    rospy.init_node('path_tracker')
    node = LineTracker(debug=False)
    node.run()


if __name__ == "__main__":
    main()