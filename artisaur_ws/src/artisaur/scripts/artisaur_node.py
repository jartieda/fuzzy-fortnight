#!/usr/bin/env python

import rospy
import math
from urdf_parser_py.urdf import URDF
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from motor import Motor
from display import Display
from eyes import Eyes


def euclidean_of_vectors(xyz1, xyz2):
    return math.sqrt(
        (xyz1[0] - xyz2[0]) ** 2 +
        (xyz1[1] - xyz2[1]) ** 2 +
        (xyz1[2] - xyz2[2]) ** 2)


def get_joint(robot, joint_name):
    for joint in robot.joints:
        if joint.name == joint_name:
            return joint
    raise Exception(f"Joint {joint_name} not found")


def get_link(robot, link_name):
    for link in robot.links:
        if link.name == link_name:
            return link
    raise Exception(f"Link {link_name} not found")


class NanoSaur:
    def __init__(self):
        rospy.init_node('nanosaur')
        self.eyes = Eyes()
        self.rate = rospy.get_param("~rate", 5)
        self.timer_period = 1.0 / self.rate
        self.rpm = rospy.get_param("~rpm", 150)
        left_id = rospy.get_param("~motor.left.channel", 1)
        self.left_wheel_name = rospy.get_param("~motor.left.wheel", "sprocket_left_joint")
        right_id = rospy.get_param("~motor.right.channel", 4)
        self.right_wheel_name = rospy.get_param("~motor.right.wheel", "sprocket_right_joint")
        
        rospy.loginfo(f"RPM motors {self.rpm} - timer {self.timer_period}")
        rospy.loginfo(f"Motor left: Channel {left_id} - Wheel {self.left_wheel_name}")
        rospy.loginfo(f"Motor Right: Channel {right_id} - Wheel {self.right_wheel_name}")
        
        self.mright = Motor(right_id, self.rpm)
        self.mleft = Motor(left_id, self.rpm)

        rospy.Subscriber('robot_description', String, self.configure_robot)
        self.joint_state = JointState()
        self.joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.drive_sub = rospy.Subscriber('cmd_vel', Twist, self.drive_callback)

        self.p = [0.0, 0.0]  # [right, left]
        self.r = [0.0, 0.0]  # [right, left]
        self.rate = rospy.Rate(self.rate)

        rospy.loginfo("Hello NanoSaur!")

    def configure_robot(self, description):
        rospy.loginfo('Got description, configuring robot')
        robot = URDF.from_xml_string(description)
        joint_left = get_joint(robot, self.left_wheel_name)
        joint_right = get_joint(robot, self.right_wheel_name)
        self.wheel_separation = euclidean_of_vectors(joint_left.origin.xyz, joint_right.origin.xyz)
        link_left = get_link(robot, joint_left.child)
        self.radius = link_left.collision.geometry.radius
        rospy.loginfo(f"Wheel separation {self.wheel_separation} - Radius {self.radius}")

    def convert_speed(self, v, w):
        half_wheel_separation = self.wheel_separation / 2.0
        vr = v + half_wheel_separation * w
        vl = v - half_wheel_separation * w
        rr = vr / self.radius
        rl = vl / self.radius
        return [rr, rl]

    def drive_callback(self, msg):
        self.eyes.ping()
        v = msg.linear.x
        w = msg.angular.z
        rospy.logdebug(f"v={v} w={w}")
        r = self.convert_speed(v, w)
        rospy.logdebug(f"rad {r}")
        max_speed = self.rpm / 60.0
        self.r = [max(-max_speed, min(max_speed, r[0])), max(-max_speed, min(max_speed, r[1]))]

        if r[0] != self.r[0]:
            rospy.logwarn(f"ref speed over {r[0] - self.r[0]}")
        if r[1] != self.r[1]:
            rospy.logwarn(f"ref speed over {r[1] - self.r[1]}")

        rpmr = self.r[0] * 60.0
        rpml = self.r[1] * 60.0
        rospy.loginfo(f"RPM R={rpmr} L={rpml}")
        self.mright.set_speed(rpmr)
        self.mleft.set_speed(rpml)

    def run(self):
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            self.joint_state.header.stamp = now
            self.joint_state.name = [self.left_wheel_name, self.right_wheel_name]
            self.p[0] = (self.p[0] + self.r[0] * self.timer_period) % (2 * math.pi)
            self.p[1] = (self.p[1] + self.r[1] * self.timer_period) % (2 * math.pi)
            self.joint_state.position = self.p
            self.joint_state.velocity = self.r
            self.joint_pub.publish(self.joint_state)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        nanosaur = NanoSaur()
        nanosaur.run()
    except rospy.ROSInterruptException:
        pass