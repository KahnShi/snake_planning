#!/usr/bin/env python

import time
import sys
import rospy
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32

class SamplePointsServer:
    def init(self):
        rospy.init_node('sample_points_manually', anonymous=True)
        self.sample_points_ = []
        self.sample_points_pub_topic_name_ = rospy.get_param("~sample_points_pub_topic_name", "sampling_points")
        self.sample_points_pub_ = rospy.Publisher(self.sample_points_pub_topic_name_, PolygonStamped, queue_size = 1)

    def vector3dConvertToPoint32(self, vec):
        pt = Point32()
        pt.x = vec[0]
        pt.y = vec[1]
        pt.z = vec[2]
        return pt

    def manuallyPublish(self):
        height = 0.0
        ## [0.0, 1.0, 2.0], [1.0, 2.0, 2.0], [2.0, 2.0, 2.0], [5.0, 0.0, 2.0],[6.0, 0.0, 2.0]
        # self.sample_points_ = [[0.0, 1.0, 2.0],
        #                        [-10000.0, -10000.0, -10000.0],
        #                        [1.0, 2.0, 2.0],
        #                        [-10000.0, -10000.0, -10000.0],
        #                        [2.0, 2.0, 2.0],
        #                        [-10000.0, -10000.0, -10000.0],
        #                        [5.0, 0.0, 2.0],
        #                        [-10000.0, -10000.0, -10000.0],
        #                        [6.0, 0.0, 2.0],
        #                        [0.0, 0.0, 0.0]
        #                        ]
        self.sample_points_ = [[0.5, 0.5, 2.0],
                               [-10000.0, -10000.0, -10000.0],
                               [1.0, 1.0, 2.0],
                               [-10000.0, -10000.0, -10000.0],
                               [1.5, 1.0, 2.0],
                               [-10000.0, -10000.0, -10000.0],
                               [2.0, 1.0, 2.0],
                               [-10000.0, -10000.0, -10000.0],
                               [2.5, 0.5, 2.0],
                               [-10000.0, -10000.0, -10000.0],
                               [3.0, 0.0, 2.0],
                               [-10000.0, -10000.0, -10000.0],
                               [3.5, 0.0, 2.0],
                               [0.0, 0.0, 0.0]
        ]
        sample_polygon_points = PolygonStamped()
        sample_polygon_points.header.frame_id = "/world"
        sample_polygon_points.header.stamp = rospy.Time.now()
        for i in range(0, len(self.sample_points_)):
            sample_point = self.vector3dConvertToPoint32(self.sample_points_[i])
            sample_polygon_points.polygon.points.append(sample_point)
        self.sample_points_pub_.publish(sample_polygon_points)

if __name__ == '__main__':
    try:
        sample_pts_server = SamplePointsServer()
        sample_pts_server.init()
        time.sleep(1.0)
        sample_pts_server.manuallyPublish()
        print "publish finished"
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
