#!/usr/bin/env python3

import argparse
import collections
import logging
import sys
import threading
import time
import numpy as np
import matplotlib.pyplot as plt

import bosdyn
import bosdyn.client
import bosdyn.client.util
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.async_tasks import AsyncPeriodicQuery, AsyncTasks
from bosdyn.client.robot_state import RobotStateClient

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2

LOGGER = logging.getLogger(__name__)

class AsyncPointCloud(AsyncPeriodicQuery):
    """Grab robot state."""

    def __init__(self, robot_state_client):
        super(AsyncPointCloud, self).__init__("point_clouds", robot_state_client, LOGGER,
                                              period_sec=0.2)

    def _start_query(self):
        return self._client.get_point_cloud_from_sources_async(["velodyne-point-cloud"])

def main(argv):

    hostname = rospy.get_param("hostname")
    username = rospy.get_param("username")
    password = rospy.get_param("password")

    # parser = argparse.ArgumentParser()
    # bosdyn.client.util.add_base_arguments(parser)
    # options = parser.parse_args(argv)

    sdk = bosdyn.client.create_standard_sdk('VelodyneClient')
    robot = sdk.create_robot(hostname)
    robot.authenticate(username, password)
    robot.sync_with_directory()

    _point_cloud_client = robot.ensure_client('velodyne-point-cloud')
    _point_cloud_task = AsyncPointCloud(_point_cloud_client)
    _task_list = [_point_cloud_task]
    _async_tasks = AsyncTasks(_task_list)
    print('Connected.')

    rospy.init_node('velodyne_point_cloud_publisher')
    point_cloud_publisher = rospy.Publisher('velodyne_points', PointCloud2, queue_size=10)

    update_thread = threading.Thread(target=_update_thread, args=[_async_tasks])
    update_thread.daemon = True
    update_thread.start()

    while any(task.proto is None for task in _task_list):
        time.sleep(0.1)
    fig = plt.figure()

    body_tform_butt = SE3Pose(-0.5, 0, 0, Quat())
    body_tform_head = SE3Pose(0.5, 0, 0, Quat())

    while True:
        if _point_cloud_task.proto is not None and _point_cloud_task.proto[0].point_cloud:
            data = np.frombuffer(_point_cloud_task.proto[0].point_cloud.data, dtype=np.float32) 
 
            # Convert the numpy array to a PointCloud2 message
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'gpe'
            fields = [PointField('x', 0, PointField.FLOAT32, 1),
                      PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1)]
            point_cloud_msg = pc2.create_cloud(header, fields, data.reshape(-1, 3))

            # Publish the PointCloud2 message
            point_cloud_publisher.publish(point_cloud_msg)

        time.sleep(0.1)

def _update_thread(async_task):
    while True:
        async_task.update()
        time.sleep(0.01)

if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)
