# Copyright 2025 Enactic, Inc.
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

"""
OpenArm Bimanual Direct Control Launch File (No MoveIt)

This launcher provides direct hardware control without MoveIt planning.

=============================================================================
CONTROLLER MODE SELECTION
=============================================================================

┌--┬--┬--┐
│ Mode                │ Trajectory (Default)           │ Forward (Streaming)            │
├--┼--┼--┤
│ Use Case            │ Smooth paths, MoveIt, picking  │ Joystick, haptic, real-time    │
│ Controller          │ JointTrajectoryController      │ ForwardCommandController       │
│ Interface           │ Action (feedback/status)       │ Topic (one-way, fast)          │
│ Smoothness          │ High (interpolation)           │ None (jumps to target)         │
│ Safety              │ High (controlled accel)        │ Low (hard on motors)           │
│ Latency             │ Medium                         │ Low                            │
│ Partial joints      │ V Supported                    │ X Must send all joints         │
└--┴--┴--┘

LAUNCH EXAMPLES:
  # Trajectory mode (DEFAULT - recommended for most use cases):
  ros2 launch openarm_cyberwave master_openarm_direct.launch.py robot_id:=robot_openarm_v1

  # Forward position mode (for real-time haptic teleop):
  ros2 launch openarm_cyberwave master_openarm_direct.launch.py robot_id:=robot_openarm_v1 use_trajectory_mode:=false

  # Equivalent using robot_controller argument:
  ros2 launch openarm_cyberwave master_openarm_direct.launch.py robot_id:=robot_openarm_v1 robot_controller:=forward_position_controller

HOT-SWAP CONTROLLERS (without restarting):
  # Switch from Forward to Trajectory:
  ros2 control switch_controllers \\
      --deactivate left_forward_position_controller right_forward_position_controller \\
      --activate left_joint_trajectory_controller right_joint_trajectory_controller

  # Switch from Trajectory to Forward:
  ros2 control switch_controllers \\
      --deactivate left_joint_trajectory_controller right_joint_trajectory_controller \\
      --activate left_forward_position_controller right_forward_position_controller

CHECK ACTIVE CONTROLLERS:
  ros2 control list_controllers

IMPORTANT: Ensure your mqtt_bridge mapping's 'teleop_controller_mode' matches the active controller!
  - teleop_controller_mode: "trajectory"  → use JointTrajectoryController
  - teleop_controller_mode: "streaming"   → use ForwardCommandController

AUTOMATIC PARAMETER VERIFICATION:
  This launch file automatically verifies and displays trajectory parameters from
  robot_openarm_v1.yaml (lines 230-236) including:
    - Trajectory timing (default, min, max, safety_factor)
    - Motion scaling (velocity_scaling, acceleration_scaling)
    - Controller mode consistency checks
  
  See: docs/TRAJECTORY_PARAMETER_VERIFICATION.md for details

For MoveIt-based planning, use master_openarm.launch.py in openarm_bimanual_moveit_config.
=============================================================================
"""

import os
import sys
import signal
import atexit
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Import CAN manager for setup/teardown
CAN_MANAGER_AVAILABLE = False
try:
    # Try multiple import strategies
    imported = False
    
    # Strategy 1: Direct import (works if package is properly installed with Python path)
    try:
        from openarm_bringup.can_manager import CANManager
        CAN_MANAGER_AVAILABLE = True
        imported = True
    except ImportError:
        pass
    
    if not imported:
        # Strategy 2: Find installed package and add to path
        try:
            openarm_bringup_share = get_package_share_directory('openarm_bringup')
            install_root = os.path.dirname(os.path.dirname(openarm_bringup_share))  # Go up from share/openarm_bringup

            # Build candidate Python lib paths using the actual running Python version
            # (avoids hardcoding python3.10 which breaks on other distros / Python upgrades)
            pyver = f"python{sys.version_info.major}.{sys.version_info.minor}"
            python_paths = [
                os.path.join(install_root, 'local', 'lib', pyver, 'dist-packages'),
                os.path.join(install_root, 'lib', pyver, 'site-packages'),
                os.path.join(install_root, 'local', 'lib', pyver, 'site-packages'),
            ]

            for python_path in python_paths:
                if os.path.exists(os.path.join(python_path, 'openarm_bringup', 'can_manager.py')):
                    sys.path.insert(0, python_path)
                    from openarm_bringup.can_manager import CANManager
                    CAN_MANAGER_AVAILABLE = True
                    imported = True
                    break
        except Exception:
            pass

    if not imported:
        # Strategy 3: Fallback to source directory.
        # Prefer ROS_WS env var (set by systemd/Docker/shell); fall back to
        # __file__-relative derivation which only works with --symlink-install.
        _file_ws = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), *['..'] * 4))
        workspace_root = os.environ.get('ROS_WS', _file_ws)
        source_path = os.path.join(workspace_root, 'src', 'openarm_ros2', 'openarm_bringup')
        if os.path.exists(os.path.join(source_path, 'openarm_bringup', 'can_manager.py')):
            sys.path.insert(0, source_path)
            from openarm_bringup.can_manager import CANManager
            CAN_MANAGER_AVAILABLE = True
            imported = True

except Exception as e:
    print(f"[warn] CAN manager not available: {e}")

# Global CAN manager instance for shutdown handling
_can_manager = None


def _shutdown_can_manager():
    """Shutdown handler for CAN manager - disables motors and brings down CAN."""
    global _can_manager
    if _can_manager is not None:
        print("\n[Launch] Shutting down CAN manager...")
        _can_manager.shutdown()
        _can_manager = None


def _namespace_from_context(context, arm_prefix):
    arm_prefix_str = context.perform_substitution(arm_prefix)
    if arm_prefix_str:
        return arm_prefix_str.strip("/")
    return None


def _verify_mqtt_bridge_config(robot_id_str):
    """Load and verify mqtt_bridge trajectory parameters from mapping file."""
    try:
        mqtt_bridge_share = get_package_share_directory("mqtt_bridge")
        mapping_file = os.path.join(
            mqtt_bridge_share, "config", "mappings", f"{robot_id_str}.yaml"
        )
        
        if not os.path.exists(mapping_file):
            print(f"[master_openarm_direct] mapping file not found: {mapping_file}")
            return None
        
        with open(mapping_file, 'r') as f:
            config = yaml.safe_load(f)
        
        # Extract trajectory parameters from controllers.trajectory section
        controllers = config.get('controllers', {})
        trajectory = controllers.get('trajectory', {})
        
        if not trajectory:
            print(f"[master_openarm_direct] no 'controllers.trajectory' section in {robot_id_str}.yaml")
            return None
        
        # Extract trajectory timing parameters
        params = {
            'default_trajectory_time_sec': trajectory.get('default_trajectory_time_sec'),
            'min_trajectory_time_sec': trajectory.get('min_trajectory_time_sec'),
            'max_trajectory_time_sec': trajectory.get('max_trajectory_time_sec'),
            'trajectory_safety_factor': trajectory.get('trajectory_safety_factor'),
            'velocity_scaling': trajectory.get('velocity_scaling'),
            'acceleration_scaling': trajectory.get('acceleration_scaling'),
            'trajectory_points': trajectory.get('trajectory_points'),
            'teleop_controller_mode': config.get('teleop_controller_mode'),
        }
        
        return params
        
    except Exception as e:
        print(f"[master_openarm_direct] error loading mqtt_bridge config: {e}")
        return None


def _print_trajectory_config(params, robot_controller_str):
    """Pretty-print trajectory configuration verification."""
    if not params:
        return
    
    print("\n" + "="*80)
    print("MQTT BRIDGE TRAJECTORY CONFIGURATION VERIFICATION")
    print("="*80)
    
    teleop_mode = params.get('teleop_controller_mode', 'UNKNOWN')
    
    # Check if controller mode matches mqtt_bridge config
    controller_type_map = {
        'joint_trajectory_controller': 'trajectory',
        'forward_position_controller': 'streaming',
        'forward_velocity_controller': 'streaming',
    }
    expected_mode = controller_type_map.get(robot_controller_str, 'UNKNOWN')
    
    mode_match = "OK" if teleop_mode == expected_mode else "MISMATCH"
    print(f"Controller Mode Check: {mode_match}")
    print(f"  Active Controller:     {robot_controller_str}")
    print(f"  Expected teleop_mode:  '{expected_mode}'")
    print(f"  Actual teleop_mode:    '{teleop_mode}'")
    
    if teleop_mode != expected_mode:
        print(f"\n  warning: controller mismatch detected!")
        print(f"  Update robot_openarm_v1.yaml line 183:")
        print(f"    teleop_controller_mode: \"{expected_mode}\"")
    
    print("\nTrajectory Timing Parameters:")
    print(f"  default_trajectory_time_sec:  {params.get('default_trajectory_time_sec')} s")
    print(f"  min_trajectory_time_sec:      {params.get('min_trajectory_time_sec')} s")
    print(f"  max_trajectory_time_sec:      {params.get('max_trajectory_time_sec')} s")
    print(f"  trajectory_safety_factor:     {params.get('trajectory_safety_factor')}x")
    
    print("\nMotion Scaling (Safety Limits):")
    velocity_pct = params.get('velocity_scaling', 0) * 100
    accel_pct = params.get('acceleration_scaling', 0) * 100
    print(f"  velocity_scaling:             {params.get('velocity_scaling')} ({velocity_pct:.0f}% of max)")
    print(f"  acceleration_scaling:         {params.get('acceleration_scaling')} ({accel_pct:.0f}% of max)")
    print(f"  trajectory_points:            {params.get('trajectory_points')} waypoints")
    
    print("\nThese parameters are loaded by mqtt_bridge from:")
    print(f"  config/mappings/robot_openarm_v1.yaml (lines 230-236)")
    print("="*80 + "\n")


def _generate_robot_description(context, arm_type, use_fake_hardware, right_can_interface, left_can_interface):
    """Generate robot description using xacro processing."""
    arm_type_str = context.perform_substitution(arm_type)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    right_can_interface_str = context.perform_substitution(right_can_interface)
    left_can_interface_str = context.perform_substitution(left_can_interface)

    xacro_path = os.path.join(
        get_package_share_directory("openarm_description"),
        "urdf",
        "robot",
        "v10.urdf.xacro",
    )

    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": arm_type_str,
            "bimanual": "true",
            "use_fake_hardware": use_fake_hardware_str,
            "ros2_control": "true",
            "hand": "true",
            "right_can_interface": right_can_interface_str,
            "left_can_interface": left_can_interface_str,
        },
    ).toprettyxml(indent="  ")

    return robot_description


def _bringup_nodes(context, arm_type, use_fake_hardware, right_can_interface, left_can_interface, arm_prefix, use_trajectory_mode, robot_controller, allow_partial_joints, robot_id, manage_can):
    """Spawn robot state publisher, control node, and controllers.
    
    Controller selection priority:
    1. If robot_controller is explicitly set (non-empty), use it
    2. Otherwise, use use_trajectory_mode to auto-select:
       - True  → joint_trajectory_controller
       - False → forward_position_controller
    """
    global _can_manager
    
    namespace = _namespace_from_context(context, arm_prefix)
    robot_controller_str = context.perform_substitution(robot_controller)
    use_trajectory_mode_str = context.perform_substitution(use_trajectory_mode)
    allow_partial_joints_str = context.perform_substitution(allow_partial_joints)
    robot_id_str = context.perform_substitution(robot_id)
    manage_can_str = context.perform_substitution(manage_can)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    left_can_str = context.perform_substitution(left_can_interface)
    right_can_str = context.perform_substitution(right_can_interface)
    
    # CAN Interface Management (only if not using fake hardware)
    if manage_can_str.lower() == "true" and use_fake_hardware_str.lower() != "true":
        if CAN_MANAGER_AVAILABLE:
            print("\n" + "="*80)
            print("CAN INTERFACE MANAGEMENT")
            print("="*80)
            print("[Launch] Setting up CAN interfaces...")
            
            _can_manager = CANManager(
                left_can=left_can_str,
                right_can=right_can_str,
                auto_setup=False,
                auto_shutdown=False,  # We handle shutdown via signal handlers
            )
            
            # Setup CAN interfaces
            if _can_manager.setup_interfaces():
                print("[Launch] CAN interfaces configured")
                
                # Register shutdown handler for graceful cleanup
                atexit.register(_shutdown_can_manager)
                
                # Also register signal handlers for immediate response
                def signal_handler(signum, frame):
                    print(f"\n[Launch] Received signal {signum}, shutting down...")
                    _shutdown_can_manager()
                    sys.exit(128 + signum)
                
                signal.signal(signal.SIGINT, signal_handler)
                signal.signal(signal.SIGTERM, signal_handler)
            else:
                print("[Launch] CAN interface setup failed - continuing anyway")
            print("="*80 + "\n")
        else:
            print("[Launch] CAN manager not available - skipping CAN setup")
            print("[Launch]    Run 'colcon build --packages-select openarm_bringup' first")
    elif use_fake_hardware_str.lower() == "true":
        print("[Launch] Using fake hardware - skipping CAN setup")
    else:
        print("[Launch] CAN management disabled (manage_can:=false)")

    # Auto-select controller based on use_trajectory_mode if not explicitly set
    if not robot_controller_str:
        if use_trajectory_mode_str.lower() == "true":
            robot_controller_str = "joint_trajectory_controller"
        else:
            robot_controller_str = "forward_position_controller"

    robot_description = _generate_robot_description(
        context, arm_type, use_fake_hardware, right_can_interface, left_can_interface
    )
    robot_description_param = {"robot_description": robot_description}

    controllers_file = os.path.join(
        get_package_share_directory("openarm_bringup"),
        "config",
        "v10_controllers",
        "openarm_v10_bimanual_controllers_namespaced.yaml"
        if namespace
        else "openarm_v10_bimanual_controllers.yaml",
    )
    
    # Build controller parameters list - base config + optional override for partial joints
    controller_params = [robot_description_param, controllers_file]
    
    print("\n" + "="*80)
    print("CONTROLLER CONFIGURATION FILES")
    print("="*80)
    print(f"Base controller config:  {controllers_file}")
    
    if allow_partial_joints_str.lower() == "true":
        # Try to load from openarm_cyberwave first, then fall back to mqtt_bridge
        teleop_overrides_file = None
        try:
            teleop_overrides_file = os.path.join(
                get_package_share_directory("openarm_cyberwave"),
                "config",
                "bringup",
                "teleop_overrides.yaml",
            )
        except Exception:
            pass
        
        # Fall back to mqtt_bridge location if not found
        if not teleop_overrides_file or not os.path.exists(teleop_overrides_file):
            teleop_overrides_file = os.path.join(
                get_package_share_directory("mqtt_bridge"),
                "scripts",
                "openarm",
                "openarm_bringup",
                "config",
                "v10_controllers",
                "teleop_overrides.yaml",
            )
        
        if os.path.exists(teleop_overrides_file):
            controller_params.append(teleop_overrides_file)
            print(f"Teleop overrides:        {teleop_overrides_file} LOADED")
            print("  - Partial joint trajectories enabled")
            print("  - Gripper stalling enabled")
        else:
            print(f"Teleop overrides:        NOT FOUND (partial joints disabled)")
            print(f"  Expected: {teleop_overrides_file}")
    else:
        print("Teleop overrides:        DISABLED (allow_partial_joints=false)")
    print("="*80 + "\n")

    controller_manager_ref = (
        f"/{namespace}/controller_manager" if namespace else "/controller_manager"
    )

    # Map controller type to actual controller names
    controller_map = {
        "forward_position_controller": ("left_forward_position_controller", "right_forward_position_controller"),
        "forward_velocity_controller": ("left_forward_velocity_controller", "right_forward_velocity_controller"),
        "joint_trajectory_controller": ("left_joint_trajectory_controller", "right_joint_trajectory_controller"),
    }
    
    if robot_controller_str not in controller_map:
        raise ValueError(
            f"Unknown robot_controller: '{robot_controller_str}'. "
            f"Valid options: {list(controller_map.keys())}"
        )
    
    robot_controller_left, robot_controller_right = controller_map[robot_controller_str]
    
    # Log the selected controller mode for clarity
    print("\n" + "="*80)
    print("ACTIVE CONTROLLER CONFIGURATION")
    print("="*80)
    print(f"Controller mode:         {robot_controller_str}")
    print(f"  Left arm:              {robot_controller_left} ACTIVE")
    print(f"  Right arm:             {robot_controller_right} ACTIVE")
    print(f"  Left gripper:          left_gripper_controller ACTIVE")
    print(f"  Right gripper:         right_gripper_controller ACTIVE")
    print(f"  Joint state:           joint_state_broadcaster ACTIVE")
    
    # Show which controllers are NOT active
    all_arm_controllers = [
        "left_joint_trajectory_controller",
        "right_joint_trajectory_controller",
        "left_forward_position_controller",
        "right_forward_position_controller",
        "left_forward_velocity_controller",
        "right_forward_velocity_controller",
    ]
    inactive_controllers = [c for c in all_arm_controllers if c not in [robot_controller_left, robot_controller_right]]
    
    if inactive_controllers:
        print("\nInactive arm controllers (can be hot-swapped):")
        for controller in inactive_controllers:
            print(f"  {controller}   INACTIVE")
    
    print("="*80 + "\n")
    
    # Verify and display mqtt_bridge trajectory configuration
    mqtt_params = _verify_mqtt_bridge_config(robot_id_str)
    _print_trajectory_config(mqtt_params, robot_controller_str)

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            namespace=namespace,
            parameters=[robot_description_param],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="both",
            namespace=namespace,
            parameters=controller_params,
        ),
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    namespace=namespace,
                    arguments=[
                        "joint_state_broadcaster",
                        "--controller-manager",
                        controller_manager_ref,
                    ],
                )
            ],
        ),
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    namespace=namespace,
                    arguments=[
                        robot_controller_left,
                        robot_controller_right,
                        "-c",
                        controller_manager_ref,
                    ],
                )
            ],
        ),
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    namespace=namespace,
                    arguments=[
                        "left_gripper_controller",
                        "right_gripper_controller",
                        "-c",
                        controller_manager_ref,
                    ],
                )
            ],
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        # ========== Component Selection ==========
        DeclareLaunchArgument(
            "start_bringup",
            default_value="true",
            description="Start hardware bringup (robot_state_publisher, ros2_control, controllers)",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="false",
            description="Start RViz visualization",
        ),
        DeclareLaunchArgument(
            "start_mqtt_bridge",
            default_value="true",
            description="Start MQTT bridge for remote control",
        ),
        
        # ========== Controller Mode Selection ==========
        DeclareLaunchArgument(
            "use_trajectory_mode",
            default_value="true",
            description="True=JointTrajectoryController (smooth paths), False=ForwardCommandController (raw streaming)",
        ),
        DeclareLaunchArgument(
            "robot_controller",
            default_value="",  # Empty = auto-select based on use_trajectory_mode
            choices=["", "forward_position_controller", "forward_velocity_controller", "joint_trajectory_controller"],
            description="Override controller type (leave empty to use use_trajectory_mode setting)",
        ),
        DeclareLaunchArgument(
            "allow_partial_joints",
            default_value="true",
            description="Enable partial joint trajectories (teleop mode). If true, JointTrajectoryController accepts messages with subset of joints.",
        ),
        
        # ========== Hardware Configuration ==========
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time",
        ),
        DeclareLaunchArgument(
            "arm_type",
            default_value="v10",
            description="Arm type (v10)",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake/mock hardware for testing",
        ),
        DeclareLaunchArgument(
            "right_can_interface",
            default_value="can2",
            description="CAN interface for right arm",
        ),
        DeclareLaunchArgument(
            "left_can_interface",
            default_value="can3",
            description="CAN interface for left arm",
        ),
        DeclareLaunchArgument(
            "arm_prefix",
            default_value="",
            description="Namespace prefix for arm topics",
        ),
        
        # ========== MQTT Bridge Configuration ==========
        DeclareLaunchArgument(
            "robot_id",
            description="Robot ID for MQTT bridge mapping (e.g., robot_openarm_v1)",
        ),
        DeclareLaunchArgument(
            "debug_logs",
            default_value="false",
            description="Enable debug logging",
        ),
        
        # ========== CAN Management ==========
        DeclareLaunchArgument(
            "manage_can",
            default_value="true",
            description="Manage CAN interfaces (setup on start, disable motors + teardown on shutdown)",
        ),
    ]

    start_bringup = LaunchConfiguration("start_bringup")
    start_rviz = LaunchConfiguration("start_rviz")
    start_mqtt_bridge = LaunchConfiguration("start_mqtt_bridge")
    use_trajectory_mode = LaunchConfiguration("use_trajectory_mode")
    robot_controller = LaunchConfiguration("robot_controller")
    allow_partial_joints = LaunchConfiguration("allow_partial_joints")
    arm_type = LaunchConfiguration("arm_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    right_can_interface = LaunchConfiguration("right_can_interface")
    left_can_interface = LaunchConfiguration("left_can_interface")
    arm_prefix = LaunchConfiguration("arm_prefix")
    robot_id = LaunchConfiguration("robot_id")
    debug_logs = LaunchConfiguration("debug_logs")
    manage_can = LaunchConfiguration("manage_can")

    bringup_nodes = OpaqueFunction(
        function=_bringup_nodes,
        args=[arm_type, use_fake_hardware, right_can_interface, left_can_interface, arm_prefix, use_trajectory_mode, robot_controller, allow_partial_joints, robot_id, manage_can],
        condition=IfCondition(start_bringup),
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("openarm_description"), "rviz", "bimanual.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(
            PythonExpression(
                ["'", start_bringup, "' == 'true' and '", start_rviz, "' == 'true'"]
            )
        ),
    )

    mqtt_bridge_share = get_package_share_directory("mqtt_bridge")
    mqtt_bridge_params = os.path.join(mqtt_bridge_share, "config", "params.yaml")
    mqtt_mapping_file = PythonExpression([
        "'", mqtt_bridge_share, "/config/mappings/' + '", robot_id, "' + '.yaml'"
    ])

    mqtt_bridge_node = Node(
        package="mqtt_bridge",
        executable="mqtt_bridge_node",
        name="mqtt_bridge_node",
        output="screen",
        parameters=[
            mqtt_bridge_params,
            {
                "robot_id": robot_id,
                "debug_logs": debug_logs,
                "mapping_file": mqtt_mapping_file,
            },
        ],
        condition=IfCondition(start_mqtt_bridge),
    )

    return LaunchDescription(
        declared_arguments
        + [
            bringup_nodes,
            rviz_node,
            mqtt_bridge_node,
        ]
    )
