#!/usr/bin/env python
import time
import sys
import rospy
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32

def list_minus(lst1, lst2):
    lst = []
    for i in range(0, len(lst1)):
        lst.append(lst1[i] - lst2[i])
    return lst

class ControlPointsServer:
    def init(self):
        rospy.init_node('control_points_manually', anonymous=True)
        self.control_points_ = []
        self.control_points_pub_topic_name_ = rospy.get_param("~control_points_pub_topic_name", "control_points")
        self.segment_period_time_ = rospy.get_param("~segment_period_time", 1.0)

        self.control_points_pub_ = rospy.Publisher(self.control_points_pub_topic_name_, PolygonStamped, queue_size = 1)

    def vector3dConvertToPoint32(self, vec):
        pt = Point32()
        pt.x = vec[0]
        pt.y = vec[1]
        pt.z = vec[2]
        return pt

    def manuallyPublish(self):
        height = 2.0
        self.control_points_ = [[0.44, 0.88, height], 
                                [0.44, 0.44, height], 
                                [0.44, 0.44, height], 
                                [0.44, 0.0, height], 
                                [0.44, 0.0, height],
                                [0.0, 0.0, height], 
                                [-0.44, 0.0, height],
                                [-0.64, 0.3, height],
                                [-0.94, 0.6, height],
                                [-1.34, 0.6, height]
                                # [-0.88, 0.0, height], 
                                # [-1.18, 0.3, height],
                                # [-1.48, 0.6, height],
                                # [-1.88, 0.6, height]
        ]
        control_polygon_points = PolygonStamped()
        for i in range(0, len(self.control_points_)):
            control_point = self.vector3dConvertToPoint32(self.control_points_[i])
            control_polygon_points.polygon.points.append(control_point)
        self.control_points_pub_.publish(control_polygon_points)

if __name__ == '__main__':
    try:
        control_pts_server = ControlPointsServer()
        control_pts_server.init()
        time.sleep(1.0)
        control_pts_server.manuallyPublish()
        print "publish finished"
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
