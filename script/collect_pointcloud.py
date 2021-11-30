#!/usr/bin/env python

from __future__ import print_function
import os
import sys
import rospy
from phoxi_camera.srv import *
import rospy
from sensor_msgs.msg import Image, PointCloud2
import geometry_msgs.msg._Transform as transform
from std_srvs.srv import Empty

import ros_numpy

def callback_pcl(pc2_msg):
    xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_msg)
    print(xyz_array)

from dynamic_reconfigure.msg import ConfigDescription
import time, threading, pickle


class get_pointcloud(object):
    def __init__(self, frame_number):
        rospy.init_node('scan_values', anonymous=True)
        rospy.wait_for_service('/phoxi_camera/get_frame')

        self.texture = []
        self._get_subscribers()
        self.lock = threading.Lock()
        self.logs = {}

        self.set_frame = rospy.ServiceProxy('/phoxi_camera/V2/set_coordinate_space', SetCoordinatesSpace)
        self.set_frame(frame_number)  # CameraSpace = 1, MarkerSpace = 3, RobotSpace = 4, CustomSpace = 5

        self.scan = rospy.ServiceProxy('/phoxi_camera/get_frame', GetFrame)
        self.connect_camera = rospy.ServiceProxy('/phoxi_camera/connect_camera', Empty)
        self.disconnect_camera = rospy.ServiceProxy('/phoxi_camera/disconnect_camera', Empty)
        self.save_frame = rospy.ServiceProxy('/phoxi_camera/save_frame', SaveFrame)


        self.set_transform = rospy.ServiceProxy('/phoxi_camera/V2/set_transformation', SetTransformationMatrix)
        trans = transform.Transform()
        self.set_transform(trans, frame_number, True, False)




    ## Update data with the recent scan
    def _update_(self, msg, name):

        if name == 'pointcloud':
            array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
        else:
            array = ros_numpy.numpify(msg)
        self.logs[name] = array


    def _get_subscribers(self):
        topics = {'pointcloud': PointCloud2,
                  'texture' : Image,
                  'rgb_texture' : Image,
                  'normal_map' : Image,
                  'depth_map': Image}

        for topic_i, type_i in topics.items():
            locals()['sub_'+ topic_i] = rospy.Subscriber(f'/phoxi_camera/{topic_i}', type_i, self._update_, topic_i)

    def save_data(self, file_name, i):
        self.save_frame(-1, os.path.join(os.getcwd(),f'log/test_{i}.ply') )

        with open(f'log/{file_name}.pickle', 'wb') as handle:
            pickle.dump(self.logs, handle, protocol=pickle.HIGHEST_PROTOCOL)

        # with open(f'log/{file_name}.pickle', 'rb') as handle:
        #     b = pickle.load(handle)


    def take_measurement(self):
        result = self.scan(-1)



if __name__ == "__main__":


    for i in range(3):
        print("start")
        pcl = get_pointcloud(frame_number=3)
        # pcl.connect_camera()
        print("take_measurement")
        pcl.take_measurement()

        rospy.sleep(3)
        pcl.save_data(f'test_marker_{i}',i)
    # pcl.disconnect_camera()
    # print("disconnected")
    rospy.spin()


