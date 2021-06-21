#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import Twist, Pose, Point, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud
from std_msgs.msg import UInt8
from depth_app.msg import ObsInformation

# import math tool
from enum import Enum
import numpy as np
import math
import time
import tf
import csv

'''
First: publish /control_mode=2 to follow GPS
'''

class MotionController:
    def __init__(self):
        self.sub_mode_control = rospy.Subscriber('/control_mode', UInt8, self.cbReceiveMode)
        self.sub_obs_info = rospy.Subscriber('/obs_info', ObsInformation, self.cbObsInfo)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.cbOdom)
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.ModeControlSeq = Enum('ModeControlSeq','idle followGPS obsAvoid')
        self.AvoidSeq = Enum('AvoidSeq','checkObsLocation avoidByDiamond')
        self.current_mode = self.ModeControlSeq.followGPS.value
        self.current_avoid = self.AvoidSeq.checkObsLocation.value
        
        # self adjust
        self.acceptable_error = 0.2
        self.angVel_bound = 0.5 # 0.5*57=23 degree
        self.linVel_bound = 0.5
        self.angKp = 0.6
        self.linKp = 0.3
        self.detectingLength = 2
        self.detectingWidth = 2
        self.accumDetectingLength = 0
        self.accumDetectingWidth = 0
        # self.fnMoving2BonusAlphaGain = 0.03
        # self.fnMoving2BonusBetaGain = 1.0
        # self.yAxisAdjustRange = 1.2

        # pose of robot
        self.robot_x = .0
        self.robot_y = .0
        self.robot_yaw = .0
        self.waypoints = np.array([[7,0],[7,-2],[0,-2],[0,-3],[7,-3]])#np.load('/home/vincent/catkin_ws/src/mower/temp/GPS_goal.npy')
        self.index_waypoint = 0
        self.goal_x = self.waypoints[self.index_waypoint][0]
        self.goal_y = self.waypoints[self.index_waypoint][1]
        self.is_triggered = False

        # depth information
        # self.pintcloudWidth = None
        # self.minDepth = 10
        self.detectedTimes = 0
        self.obsInfo = np.zeros(4)
        print("successfully initialized!!")
        loop_rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            if self.is_triggered == True:
                self.fnControlNode()
            loop_rate.sleep()

        rospy.on_shutdown(self.fnShutDown)
    
    def cbReceiveMode(self, msgMode):
        rospy.loginfo("Starts the process with %d", msgMode.data)
        self.current_mode = msgMode.data
        self.is_triggered = True

    def cbOdom(self, msgOdom):
        pose_x, pose_y, pose_yaw = self.fnGetWheelPose(msgOdom)
        self.robot_x = pose_x
        self.robot_y = pose_y
        self.robot_yaw = pose_yaw
    
    def fnMoving(self):
        x = self.robot_x
        y = self.robot_y

        x_diff, y_diff = self.goal_x - x, self.goal_y - y
        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff) - self.robot_yaw + np.pi) % (2 * np.pi) - np.pi # [-pi, pi]
        
        print("x_diff: %.3f, y_diff: %.3f, rho: %.3f, alpha: %.3f" % (x_diff, y_diff, rho, alpha) )
        print("now goal: ", self.goal_x, self.goal_y)
        if rho < self.acceptable_error:
            self.index_waypoint += 1
            if self.index_waypoint == len(self.waypoints):
                print("finish all waypoints!")
                return True

            self.goal_x = self.waypoints[self.index_waypoint][0]
            self.goal_y = self.waypoints[self.index_waypoint][1]
            rospy.loginfo("progress %s/%s, %s%%" % (self.index_waypoint+1, len(self.waypoints),
                                                        100*(self.index_waypoint+1)/len(self.waypoints)) )
            return False
        else:
            self.fnTrackPcontrol(rho, alpha)
            return False

    def cbObsInfo(self,msgObsInfo):
        self.detectedTimes += 1
        self.accumDetectingLength += msgObsInfo.obs_min_dist
        self.accumDetectingWidth += msgObsInfo.obs_width

    def fnDetectedTimes(self):
        print("detect obs times: ", self.detectedTimes)
        if self.detectedTimes > 4:
            self.detectedTimes = 0
            self.detectingLength = self.accumDetectingLength / 5
            self.detectingWidth = self.accumDetectingWidth / 5
            
            return True
        return False

    def fnAvoiding(self):
        if self.current_avoid == self.AvoidSeq.checkObsLocation.value:
            print ("...self.AvoidSeq.checkObsLocation: enter...")
            self.fnStop()
            self.is_sequence_finished = self.fnDetectedTimes()
            
            if self.is_sequence_finished == True:
                print ("Finished: checkObsLocation")
                self.current_avoid = self.AvoidSeq.avoidByDiamond.value
                self.is_sequence_finished = False
            return False

        elif self.current_avoid == self.AvoidSeq.avoidByDiamond.value:
            # self.pintcloudWidth = np.max(self.pintcloudX) - np.min(self.pintcloudX)
            # self.fnTurnSec(3, np.arctan2(self.pintcloudWidth, self.minDepth))
            print ("...self.AvoidSeq.avoidByDiamond: enter...")
            self.fnTurnSec(3, -np.pi/8)
            self.fnStop()
            self.fnLIinearSec(6, self.detectingLength*1.2)
            self.fnStop()
            # self.fnTurnSec(6, np.pi/4)
            # self.fnStop()
            # self.fnLIinearSec(6, self.detectingLength*1.1)
            # self.fnStop()
            # self.fnTurnSec(3, -np.pi/8)
            # self.fnStop()
            print ("Finished: stop")
            self.is_sequence_finished = False
            self.AvoidSeq = self.AvoidSeq.checkObsLocation.value
            return True

    def fnTrackPcontrol(self, dist, angle):
        msgVel = Twist()
        if angle > 0.16 or angle < -0.16:
            print("adjust angle! angle: ", angle)
            angularVel = angle * self.angKp
            if np.abs(angularVel) > self.angVel_bound : 
                if angularVel > 0:
                    angularVel = self.angVel_bound
                else:
                    angularVel = -self.angVel_bound
            msgVel.angular.z = angularVel

        elif dist > 0.1:
            
            linearVel = dist * self.linKp
            if np.abs(linearVel) > self.linVel_bound :
                if linearVel > 0 :
                    linearVel = self.linVel_bound
                else: 
                    linearVel = -self.linVel_bound
            msgVel.linear.x = linearVel
        
        self.pub_cmd.publish(msgVel)

    def fnControlNode(self):
        if self.current_mode == self.ModeControlSeq.idle.value:
            print("----current mode: idle ----")
            self.fnConstVel(0.2)

        if self.current_mode == self.ModeControlSeq.followGPS.value:
            print("----current mode: follow GPS----")
            if self.fnMoving() == True:
                print("Finished Mode: all waypoints")
                # self.msg_pub.data = 1
                # self.pub_finish.publish(self.msg_pub)
                self.is_triggered = False
                self.fnStop()
                # self.current_motion_sequence = self.MotionSequence.searching_marker.value ##############3
                # self.current_changing_dir_sequence = self.ChangDirSequence.initial_turn.value #############
                # self.is_marker_pose_received = False

        if self.current_mode == self.ModeControlSeq.obsAvoid.value:
            print("----current mode: avoidance----")
            if self.fnAvoiding() == True:
                self.current_mode = self.ModeControlSeq.followGPS.value
                # self.current_mode = self.ModeControlSeq.idle.value
                self.detectedTimes = 0
                self.accumDetectingLength = 2
                self.accumDetectingWidth = 2
                print("Finished Mode: avoidance")


    def fnGetWheelPose(self, OdomMsg):
        pose_x = OdomMsg.pose.pose.position.x
        pose_y = OdomMsg.pose.pose.position.y
        quaternion = (
            OdomMsg.pose.pose.orientation.x,
            OdomMsg.pose.pose.orientation.y,
            OdomMsg.pose.pose.orientation.z,
            OdomMsg.pose.pose.orientation.w )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2] 
        return pose_x, pose_y, yaw

    def fnLIinearSec(self, sec, dist):
        print("fnLIinearSec(dist, sec): %.2f %.2f" % (dist, sec))
        linVel = dist/sec
        print("linear velocity %.2f" % (linVel))
        for i in range(int(sec/0.2)):
            self.fnConstVel(linVel)
            time.sleep(0.2)
  
    def fnConstVel(self, vel):
        msgVel = Twist()
        if np.abs(vel) > self.linVel_bound:
            if vel > 0:
                vel = self.linVel_bound
            else: 
                vel = -self.linVel_bound
            print("Input velocity over ", self.linVel_bound)
        # print("linear vel: ", vel)
        msgVel.linear.x = vel
        self.pub_cmd.publish(msgVel)

    def fnTurnSec(self, sec, angle):
        print("fnTurnSec(angle, sec): %.2f %.2f" % (angle, sec))
        angVel = angle/sec
        print("angular velocity %.2f" % (angVel))
        for i in range(int(sec/0.2)):
            self.fnConstAngVel(angVel)
            print(i)
            time.sleep(0.2)

    def fnConstAngVel(self, angVel):
        msgVel = Twist()
        if np.abs(angVel) > self.angVel_bound:
            if angVel > 0:
                angVel = self.angVel_bound
            else: 
                angVel = -1 * self.angVel_bound
            print("Input angVelocity over ", self.angVel_bound)
        # print("turn angVel: ", angVel)
        msgVel.angular.z = angVel
        self.pub_cmd.publish(msgVel)

    def fnStop(self):
        msgVel = Twist()
        msgVel.linear.x = 0
        msgVel.linear.y = 0
        msgVel.linear.z = 0
        msgVel.angular.x = 0
        msgVel.angular.y = 0
        msgVel.angular.z = 0
        self.pub_cmd.publish(msgVel)
    
    def main(self):
        print("successfully initialized!")
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('Mower_motion_node')
    node = MotionController() 
    node.main()