#!/usr/bin/env python3

import imp
import time
import os
import rospy
from std_msgs.msg import Float32, Int32
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped
from std_msgs.msg import String
import smach
from threading import Lock, Thread

MOTOR_TOPIC = "/csc22912/car_cmd_switch_node/cmd"
LEFT_MOTOR_TOPIC = "/csc22912/wheels_driver_node/wheels_cmd"
RIGHT_MOTOR_TOPIC = ""
LINE_TRACKER_TOPIC = "/csc22912/output/line_tracker"
STOP_MATCHER_TOPIC = "/csc22912/output/stop_matcher"
VELOCITY = 0.30


class MotorController:
    def __init__(self):
        # self.pub = rospy.Publisher(MOTOR_TOPIC, WheelsCmdStamped, queue_size=10)
        self.stopped = False
        self.mutex = Lock()
        self.targetVelocity = 0.0
        self.linearVelocity = 0.0
        self.angularVelocity = 0.0
        self.acceleration = 0.05
        self.rate = rospy.Rate(10)
        self.pub = rospy.Publisher(LEFT_MOTOR_TOPIC, WheelsCmdStamped, queue_size=10)
    
    
    def drive(self, linearVelocity, angularVelocity):
        '''
        msg = Twist2DStamped()
        msg.header.stamp = rospy.Time.now()
        msg.v = linearVelocity
        msg.omega = 0 if angularVelocity == 0 else max(angularVelocity * 5.5, 2.5, key=abs)
        rospy.loginfo(f"OMEGA: {msg.omega}")
        self.pub.publish(msg)
        '''
        rospy.loginfo(f"Angular Velocity: {angularVelocity}")
        max_angular_velocity = 0.55
        if abs(angularVelocity) < 0.20:
            angularVelocity *= 1.5
        if abs(angularVelocity) > max_angular_velocity:
            angularVelocity = max_angular_velocity * -1 if angularVelocity < 0 else max_angular_velocity
        vel_left, vel_right = self.turn_to_velocities(linearVelocity, angularVelocity)
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.Time.now()
        msg.vel_left = vel_left
        msg.vel_right = vel_right
        self.pub.publish(msg)
        self.rate.sleep()
    
    def turn_to_velocities(self, linearVelocity, angularVelocity):
        left_velocity = linearVelocity
        right_velocity = linearVelocity
        difference = abs(angularVelocity)
        left_accel = -0.01 if angularVelocity > 0 else 0.01
        right_accel = 0.01 if angularVelocity > 0 else -0.01
        while difference > 0:
            left_velocity += left_accel
            right_velocity += right_accel
            difference -= 0.02
        return left_velocity, right_velocity
    
    def stop(self):
        self.stopped = True
        self.thread.join()


class LineTracker:
    def __init__(self):
        self.line = 0
        self.mutex = Lock()
        self.subscriber = rospy.Subscriber(LINE_TRACKER_TOPIC, Float32, self.callback)
    
    def callback(self, data):
        with self.mutex:
            self.line = data.data
    
    def get_line(self):
        with self.mutex:
            return self.line


class StopDetector:
    def __init__(self):
        self.matches = 0
        self.mutex = Lock()
        self.subscriber = rospy.Subscriber(STOP_MATCHER_TOPIC, Int32, self.callback)
    
    def callback(self, data):
        with self.mutex:
            self.matches = data.data
    
    def get_matches(self):
        with self.mutex:
            return self.matches
    
    def stop_sign_detected(self):
        # rospy.loginfo(f"MATCHES: {self.get_matches()}")
        return self.get_matches() > 4


class State(smach.State):
    def __init__(self, motor_publisher, line_tracker, stop_matcher, outcomes, input_keys=[], output_keys=[]):
        self.rate = rospy.Rate(30)
        self.line_tracker: LineTracker = line_tracker
        self.motor_publisher: MotorController = motor_publisher
        self.stop_matcher: StopDetector = stop_matcher
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
    
    def drive(self, linearVelocity, angularVelocity):
        self.motor_publisher.drive(linearVelocity, angularVelocity)
    
    def stop(self):
        self.motor_publisher.drive(0, 0)
    
    def track_line(self):
        return self.line_tracker.get_line()
    
    def is_stop_sign(self):
        return self.stop_matcher.stop_sign_detected()
    
    def execute(self, ud):
        raise NotImplementedError


class FollowPath(State):
    def __init__(self, motor_publisher, line_tracker, stop_matcher):
        State.__init__(self, motor_publisher, line_tracker, stop_matcher, outcomes=["stop"])

    def execute(self, ud):
        while not self.is_stop_sign():
            self.drive(VELOCITY, self.track_line())
            self.rate.sleep()
        return "stop"


class Stop(State):
    def __init__(self, motor_publisher, line_tracker, stop_matcher):
        State.__init__(self, motor_publisher, line_tracker, stop_matcher, outcomes=["finish"])

    def execute(self, ud):
        t = time.time()
        while (time.time() - t) < 2:
            self.drive(VELOCITY, self.track_line())
            self.rate.sleep()
        self.stop()
        rospy.sleep(2)
        return "finish"


class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

    def run(self):
        line_tracker = LineTracker()
        motor_controller = MotorController()
        stop_matcher = StopDetector()
        sm = smach.StateMachine(outcomes=['FINISH'])
    
        # Open the container
        with sm:
            # Add states to the container
            smach.StateMachine.add('FOLLOW_PATH', FollowPath(motor_controller, line_tracker, stop_matcher), transitions={'stop':'STOP'})
            smach.StateMachine.add('STOP', Stop(motor_controller, line_tracker, stop_matcher), transitions={'finish':'FINISH'})
    
        # Execute SMACH plan
        rospy.sleep(2)
        outcome = sm.execute()


if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='my_publisher_node')
    # run node
    node.run()
