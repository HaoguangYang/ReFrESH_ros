#!/usr/bin/env python
import os
import sys
import rclpy
from rclpy.node import Node
from launch import LaunchService, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import multiprocessing
import signal
import nest_asyncio
nest_asyncio.apply()

def metaLauncher(launchDesc:LaunchDescription=None, launchDescList:list=[], ros_domain_id=0)\
    ->'tuple[multiprocessing.Process]':
    if ros_domain_id:
        if ros_domain_id<=101:
            os.environ["ROS_DOMAIN_ID"] = str(ros_domain_id)
        else:
            print("WARNING: ROS_DOMAIN_ID > 101. For compatibility reasons it is NOT set.")
    lsPool = [LaunchService(noninteractive=True) for i in launchDescList]
    i = 0
    for ls in lsPool:
        ls.include_launch_description(launchDescList[i])
        i += 1
    if launchDesc:
        lsPool.append(LaunchService(noninteractive=True))
        lsPool[-1].include_launch_description(launchDesc)
    nodeProcs = [multiprocessing.Process(target = ls.run) for ls in lsPool]
    for p in nodeProcs:
        p.start()
    return tuple(nodeProcs)

def metaLaunchShutdown(procs:'tuple[multiprocessing.Process]'):
    for item in procs:
        # use SIGINT instead of SIGTERM to stop child processes ahead of launch service
        os.kill(item.pid, signal.SIGINT)
        #item.terminate()
    for item in procs:
        item.join(1.0)

class ROS2LaunchGuard(Node):
    def __init__(self):
        super().__init__('ros2_metalaunch_guard_node')
        self.metaLaunchFile = LaunchDescription()
        self.launchedNodeConfigs = {}

    def setMetaLaunchFile(self, launchFile:str):
        self.metaLaunchFile = PythonLaunchDescriptionSource(launchFile)

    # TODO: hook up to service or action calls
    def launch(self, config_id=0, ros_domain_id=0):
        launchDesc = LaunchDescription([IncludeLaunchDescription(
            self.metaLaunchFile,
            #launch_arguments = {}.items()
        )])
        config = metaLauncher(launchDesc, ros_domain_id = ros_domain_id)
        self.launchedNodeConfigs.update({config_id : config})

    def shutdown(self, config_id=0):
        metaLaunchShutdown(self.launchedNodeConfigs.pop(config_id))

if __name__ == "__main__":
    rclpy.init()
    guard = ROS2LaunchGuard()
    if len(sys.argv) > 1:
        guard.setMetaLaunchFile(sys.argv[1])
    rclpy.spin(guard)
    rclpy.shutdown()
