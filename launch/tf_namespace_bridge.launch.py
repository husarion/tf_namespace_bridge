# Copyright 2026 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="robot1",
        description="Robot namespace — determines the source topics and frame prefix",
    )

    return LaunchDescription(
        [
            namespace_arg,
            Node(
                package="tf_namespace_bridge",
                executable="tf_namespace_bridge",
                name="tf_namespace_bridge",
                namespace=LaunchConfiguration("namespace"),
                output="screen",
            ),
        ]
    )
