#! /usr/bin/env python
import rospy
from camera_info_manager import CameraInfoManager
from rospy import Publisher
from sensor_msgs.msg import CameraInfo

if __name__ == '__main__':
    rospy.init_node("pub_camera_info")

    ci_manager = CameraInfoManager(cname="JHE-129", url="package://phoxi_camera/config/camera_info/phoxi_camera.yaml")
    ci_manager.loadCameraInfo()

    pub = Publisher("phoxi_camera/camera_info", CameraInfo, queue_size=1)

    ci = ci_manager.camera_info
    while not rospy.is_shutdown():
        ci.header.stamp = rospy.Time.now()
        pub.publish(ci)
        rospy.sleep(0.1)
