import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('aruco_follower')
    mavic_description_path = os.path.join(package_dir, 'resource', 'aruco_follower.urdf')
    supervisor_description_path = os.path.join(
        package_dir, 'resource', 'playground_supervisor.urdf'
    )

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'mavic_playground.wbt')
    )

    mavic_driver = WebotsController(
        robot_name='Mavic_2_PRO',
        parameters=[
            { 'robot_description': mavic_description_path },
        ]
    )
    supervisor_driver = WebotsController(
        robot_name='Mavic_Supervisor',
        parameters=[
            { 'robot_description': supervisor_description_path },
        ]
    )

    return LaunchDescription([
        webots,
        mavic_driver,
        supervisor_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])