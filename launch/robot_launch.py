import os
from launch                            import LaunchDescription
from launch.actions                    import ExecuteProcess, TimerAction
from launch_ros.actions                import Node
import xacro


def generate_launch_description():
    # ── Resolve paths ────────────────────────────────────────────────────────
    base_dir    = os.path.join(os.path.expanduser('~'), 'Desktop', 'mp_ROS')
    robot_dir   = os.path.join(base_dir, 'robot')
    world_file  = os.path.join(base_dir, 'worlds', 'parking_lot.world')
    urdf_file   = os.path.join(robot_dir, 'robot.urdf.xacro')
    rviz_config = os.path.join(base_dir, 'rviz', 'navigation.rviz')

    robot_desc = xacro.process_file(urdf_file).toxml()

    # ── 1. Gazebo ─────────────────────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen'
    )

    # ── 2. Robot state publisher ──────────────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time':      True,
        }],
        output='screen'
    )

    # ── 3. Spawn robot in Gazebo ──────────────────────────────────────────────
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            '-x', '0.0',
            '-y', '0.5',
            '-z', '0.1',
        ],
        output='screen'
    )

    # ── 4. RViz2 — Phase 7 visualization ─────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # ── 5. A* planner — delayed so Gazebo/odom is ready ──────────────────────
    planner = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=['python3',
                     os.path.join(base_dir, 'navigation', 'astar_planner.py')],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
        rviz,
        planner,
    ])
