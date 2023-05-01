#! /usr/bin/python


import math
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class Police:
    @staticmethod
    def run():
        rospy.init_node('police')

        speed = rospy.get_param('~speed')
        police = Police(speed=speed)

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            police.track_thief()
            r.sleep()

    def __init__(self, speed: float) -> None:
        self.speed = speed
        self.distance_min = 1.0

        self.pose = None
        self.thief_pose = None

        self.sub_pose = rospy.Subscriber('/police/pose', Pose, self.pose_update)
        self.sub_thief_pose = rospy.Subscriber('/thief/pose', Pose, self.thief_pose_update)
        self.pub_vel = rospy.Publisher('/police/cmd_vel', Twist, queue_size=10)

    def track_thief(self) -> None:
        if self.pose is None or self.thief_pose is None:
            return

        if self.euclidean_distance() >= self.distance_min:
            msg = Twist()
            msg.linear.x = self.vel_linear()
            msg.angular.z = self.vel_angular()
            self.pub_vel.publish(msg)
        else:
            msg = Twist()
            self.pub_vel.publish(msg)

    def vel_linear(self) -> float:
        return self.speed

    def vel_angular(self) -> float:
        angle = self.angle_to_thief() - self.pose.theta

        # Without it turtle can't turn through the shortest angle at the border of the 2nd and 3rd quadrants
        if angle < -math.pi:
            angle += 2 * math.pi
        elif angle > math.pi:
            angle -= 2 * math.pi

        return angle

    def euclidean_distance(self) -> float:
        return math.sqrt((self.thief_pose.x - self.pose.x)**2 + (self.thief_pose.y - self.pose.y)**2)

    def angle_to_thief(self) -> float:
        return math.atan2((self.thief_pose.y - self.pose.y), (self.thief_pose.x - self.pose.x))
    
    def pose_update(self, pose: Pose) -> None:
        self.pose = pose

    def thief_pose_update(self, pose: Pose) -> None:
        self.thief_pose = pose


if __name__ == '__main__':
    Police.run()
