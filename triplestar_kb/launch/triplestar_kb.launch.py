import lifecycle_msgs
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState


def generate_launch_description():
    """Generate launch description with conditional parameter loading."""
    store_path = LaunchConfiguration("store_path")
    preload_path = LaunchConfiguration("preload_path")
    preload_files = LaunchConfiguration("preload_files")

    default_store_path = PathJoinSubstitution(
        [EnvironmentVariable("HOME"), ".local", "share", "triplestar_kb"]
    )

    store_path_arg = DeclareLaunchArgument(
        "store_path",
        default_value=default_store_path,
        description="Path to KB store directory",
    )

    preload_path_arg = DeclareLaunchArgument(
        "preload_path",
        default_value="/kas/ws/data",
        description="Path to KB preload directory",
    )

    preload_files_arg = DeclareLaunchArgument(
        "preload_files",
        default_value="['']",
        description="Comma-separated list of TTL files to preload (overrides config file if provided)",
    )

    triplestar_kb_node = LifecycleNode(
        package="triplestar_kb",
        executable="kb_node",
        name="triplestar_kb",
        namespace="",
        output="screen",
        parameters=[
            {
                "store_path": store_path,
                "preload_files": preload_files,
                "preload_path": preload_path,
            }
        ],
    )

    triplestar_kb_node_config_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(triplestar_kb_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,  # type: ignore
        )
    )

    triplestar_kb_node_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=triplestar_kb_node,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(triplestar_kb_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,  # type: ignore
                    )
                )
            ],
        )
    )

    return LaunchDescription(
        [
            store_path_arg,
            preload_files_arg,
            preload_path_arg,
            triplestar_kb_node,
            triplestar_kb_node_config_event,
            triplestar_kb_node_activate_event,
        ]
    )
