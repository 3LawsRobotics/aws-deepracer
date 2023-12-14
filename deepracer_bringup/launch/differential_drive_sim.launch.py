#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    deepracer_bringup_dir = get_package_share_directory('deepracer_bringup')
    world_path = get_package_share_directory('aws_robomaker_small_warehouse_world')
    world = os.path.join(
        world_path, 'worlds', 'no_roof_small_warehouse', 'no_roof_small_warehouse.world'
    )

    # ros gazebo launcher
    gazebo_dir = get_package_share_directory('gazebo_ros')
    gazebo_server_launcher = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=gazebo_dir + '/launch/gzserver.launch.py'
        ),
        launch_arguments={
            'pause': 'false',
            'record': 'false',
            'verbose': 'false',
            'physics': 'ode',
        }.items(),
    )
    gazebo_client_launcher = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=gazebo_dir + '/launch/gzclient.launch.py'
        ),
        launch_arguments={'verbose': 'false'}.items(),
    )

    spawn_differential_drive = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(deepracer_bringup_dir, 'launch', 'differential_drive_spawn.launch.py')
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('world', description='SDF world file', default_value=world),
            DeclareLaunchArgument(name='gui', default_value='true'),
            DeclareLaunchArgument(name='use_sim_time', default_value='true'),
            gazebo_server_launcher,
            gazebo_client_launcher,
            spawn_differential_drive,
        ]
    )


if __name__ == '__main__':
    generate_launch_description()