#!/usr/bin/env python3

import time
import rospkg
import rospy
import cv2
from std_msgs.msg import Int32
from sensor_msgs.msg import Image	
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32


class StopMatcher:
    '''This Class Runs A ROS Node For Tracking The Centroid Of The Path From A Given Camera Feed'''
    def __init__(self, debug=False):
        self.debug = debug
        self.matcher_pub = rospy.Publisher("/csc22912/output/stop_matcher", Int32)
        self.image_pub = rospy.Publisher("/csc22912/output/image_raw/compressed", CompressedImage)
        self.subscriber = rospy.Subscriber("/csc22912/camera_node/image/compressed", CompressedImage, self.imgCallback, queue_size=1)

        self.path = rospkg.RosPack().get_path("lab4")
        self.detector = cv2.ORB_create()
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.img_to_match = cv2.imread(self.path + "/resources/stopSignToMatch.png", cv2.IMREAD_COLOR)
        self.img_to_match = cv2.cvtColor(self.img_to_match, cv2.COLOR_BGR2GRAY)
        self.img_to_match = cv2.resize(self.img_to_match, (100, 100), interpolation=cv2.INTER_AREA)
        #print(image_np.type())
        self.stop_sign_descriptors = self.detector.detectAndCompute(self.img_to_match, None)[1]
    
    @staticmethod
    def detect_red(img):
        # Bounds For Red
        lower_bound_1 = np.array([0, 75, 75])
        upper_bound_1 = np.array([10, 255, 255])
        lower_bound_2 = np.array([160, 75, 75])
        upper_bound_2 = np.array([180, 255, 255])

        # Filter Colour Red
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_red = np.bitwise_or(cv2.inRange(hsv, lower_bound_1, upper_bound_1), cv2.inRange(hsv, lower_bound_2, upper_bound_2))

        # Get Contours
        contours, _ = cv2.findContours(img_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        rects = [cv2.boundingRect(c) for c in contours]
        square_rects = list(filter(lambda x: 1.0 < x[3] / x[2] < 2.5, rects))
        big_rects = list(filter(lambda x: x[2] > 15 and x[3] > 15 and x[2] < 65 and x[3] < 65, square_rects))
        return big_rects
        return [max(square_rects, key=lambda x: x[2] * x[3])] if len(contours) > 0 else []

    def match(self, img):
        try:
            # Extract Features From Test Image
            img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            img_resized = cv2.resize(img_gray, (100, 100), interpolation=cv2.INTER_AREA)
            descriptors = self.detector.detectAndCompute(img_resized, None)[1]

            # Match Images
            if descriptors is not None:
                matches = self.matcher.knnMatch(self.stop_sign_descriptors, descriptors, k=2)
                matches = list(filter(lambda x: len(x) == 2, matches))
                good_matches = [m[0] for m in matches if m[0].distance < (0.75 * m[1].distance)]
                return good_matches
            return []
        except Exception:
            return []

    def imgCallback(self, ros_data):
        '''
        This Callback Runs Whenever A New Image Is Published From The Camera.
        We Use This To Detect The Centroid Of The Path As Well As Its Colour.
        '''
        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        
        rects = self.detect_red(image_np)
        padding = 2
        #rospy.loginfo([(r[2], r[3], r[2] * r[3]) for r in rects])
        matches_list = []
        for r in rects:
            sliced_image = image_np[r[1]-padding-1:r[1]+r[3]+padding, r[0]-padding:r[0]+r[2]+padding, :]
            matches_list.append(len(self.match(sliced_image)))
            cv2.rectangle(image_np, (r[0] - padding-1, r[1] - padding), (r[0] + r[2] + padding, r[1] + r[3] + padding), color=(255, 0, 0))
        matches = max(matches_list) if len(matches_list) > 0 else 0

        # Publish The Number Of Matches
        match_msg = Int32(matches)
        self.matcher_pub.publish(match_msg)

        # If Debug Is True, Show The Bounding Boxes And Log The Matches
        if self.debug:
            #rospy.loginfo(matches)
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
            self.image_pub.publish(msg)


    def run(self):
        '''Run The Node'''
        rospy.spin()


def main():
    rospy.init_node('stop_matcher')
    node = StopMatcher(debug=True)
    node.run()


if __name__ == "__main__":
    main()
