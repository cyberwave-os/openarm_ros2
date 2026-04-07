#!/usr/bin/env python3
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
Unified OpenArm Bimanual Launch File with Mode Switching

This launch file provides a unified interface to launch the OpenArm bimanual
system in different control modes with automatic controller configuration.

CONTROL MODES:
==============

1. TRAJECTORY MODE (trajectory)
   - Direct JointTrajectoryController control
   - No motion planning
   - Best for: Pre-planned trajectories, simple motions
   - Command: ros2 launch openarm_bringup master_openarm_unified.launch.py mode:=trajectory

2. STREAMING MODE (streaming)
   - ForwardCommandController (position or velocity)
   - Real-time, low-latency control
   - Best for: High-frequency teleop (>20Hz), haptic devices
   - Command: ros2 launch openarm_bringup master_openarm_unified.launch.py mode:=streaming

3. MOVEIT MODE (moveit)
   - MoveIt2 move_group + collision avoidance
   - No RViz visualization
   - Best for: Autonomous planning, collision-free paths
   - Command: ros2 launch openarm_bringup master_openarm_unified.launch.py mode:=moveit

4. MOVEIT+RVIZ MODE (moveit_rviz)
   - MoveIt2 move_group + RViz Motion Planning plugin
   - Interactive visualization
   - Best for: Development, testing, manual planning
   - Command: ros2 launch openarm_bringup master_openarm_unified.launch.py mode:=moveit_rviz

5. MOVEIT+MQTT MODE (moveit_mqtt)
   - MoveIt2 collision-aware teleoperation via MQTT
   - MQTT → MoveIt2 Planning → JointTrajectoryController
   - Best for: Safe remote teleoperation with collision avoidance
   - Command: ros2 launch openarm_bringup master_openarm_unified.launch.py mode:=moveit_mqtt robot_id:=robot_openarm_v1

CONTROLLER MATRIX:
==================

Mode           | Active Controllers                                      | MoveIt | MQTT  | RViz
---------------|--------------------------------------------------------|--------|-------|------
trajectory     | JointTrajectoryController                              | No     | Yes   | No
streaming      | ForwardCommandController                               | No     | Yes   | No
moveit         | JointTrajectoryController + move_group                 | Yes    | No    | No
moveit_rviz    | JointTrajectoryController + move_group + RViz          | Yes    | No    | Yes
moveit_mqtt    | JointTrajectoryController + move_group + MQTT bridge   | Yes    | Yes   | No

QUICK START:
============

# 1. Safe MQTT teleoperation with collision avoidance (RECOMMENDED for remote operation):
ros2 launch openarm_bringup master_openarm_unified.launch.py \\
    mode:=moveit_mqtt \\
    robot_id:=robot_openarm_v1

# 2. Manual planning with RViz (development/testing):
ros2 launch openarm_bringup master_openarm_unified.launch.py \\
    mode:=moveit_rviz

# 3. Direct trajectory control (no planning):
ros2 launch openarm_bringup master_openarm_unified.launch.py \\
    mode:=trajectory \\
    robot_id:=robot_openarm_v1

# 4. Low-latency streaming (high-frequency teleop):
ros2 launch openarm_bringup master_openarm_unified.launch.py \\
    mode:=streaming \\
    robot_id:=robot_openarm_v1

PARAMETERS:
===========

mode               : Control mode (trajectory/streaming/moveit/moveit_rviz/moveit_mqtt)
robot_id           : Robot ID for MQTT bridge (required for trajectory/streaming/moveit_mqtt)
use_fake_hardware  : Use mock hardware (default: false)
arm_type           : Arm version (default: v10)
left_can_interface : CAN interface for left arm (default: can3)
right_can_interface: CAN interface for right arm (default: can2)
allow_partial_joints: Enable partial joint trajectories (default: true)
use_trac_ik        : Use TRAC-IK solver instead of KDL (default: true, requires installation)
manage_can         : Manage CAN interfaces - setup on start, disable motors on shutdown (default: true)

CAN MANAGEMENT:
===============

When manage_can:=true (default), this launch file will:

1. ON STARTUP:
   - Configure CAN interfaces (bitrate=1Mbps, dbitrate=5Mbps, CAN-FD on)
   - Bring up can2 (right arm) and can3 (left arm)

2. ON SHUTDOWN (SIGINT/SIGTERM/Ctrl+C):
   - Send motor disable commands to all 8 motors on both CAN buses
   - Bring down CAN interfaces

This ensures motors are safely disabled when the launch is terminated,
preventing the robot from maintaining active torque without control.

To disable CAN management (e.g., if using external CAN setup):
   ros2 launch openarm_bringup master_openarm_unified.launch.py manage_can:=false
"""

import json
import os
import sys
import signal
import atexit
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnShutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Import CAN manager for setup/teardown
CAN_MANAGER_AVAILABLE = False
try:
    # Try multiple import strategies
    imported = False
    
    # Strategy 1: Direct import (works if package is properly installed with Python path)
    try:
        from openarm_bringup.can_manager import CANManager, setup_can_for_launch
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
                    from openarm_bringup.can_manager import CANManager, setup_can_for_launch
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
            from openarm_bringup.can_manager import CANManager, setup_can_for_launch
            CAN_MANAGER_AVAILABLE = True
            imported = True

except Exception as e:
    print(f"[warn] CAN manager not available: {e}")

# Global CAN manager instance for shutdown handling
_can_manager = None
_launched_process_pids = []


def _register_process_pid(process_name, pid):
    """Track launched process PIDs for cleanup."""
    global _launched_process_pids
    _launched_process_pids.append((process_name, pid))


def _cleanup_all_processes():
    """Kill all launched ROS processes on shutdown."""
    global _launched_process_pids
    
    if _launched_process_pids:
        print("\n[Launch] ============================================================")
        print("[Launch] SHUTDOWN: Cleaning up launched processes...")
        print("[Launch] ============================================================")
        
        for process_name, pid in _launched_process_pids:
            try:
                print(f"[Launch]   Terminating {process_name} (PID: {pid})")
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass  # Process already dead
            except Exception as e:
                print(f"[Launch] failed to kill {process_name}: {e}")
        
        # Wait a bit for graceful shutdown
        import time
        time.sleep(1)
        
        # Force kill any remaining processes
        for process_name, pid in _launched_process_pids:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            except Exception:
                pass
        
        _launched_process_pids.clear()
        print("[Launch] process cleanup complete")
        print("[Launch] ============================================================")


def generate_robot_description(context: LaunchContext, arm_type, use_fake_hardware, left_can_interface, right_can_interface):
    """Generate URDF robot description."""
    arm_type_str = context.perform_substitution(arm_type)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    left_can_interface_str = context.perform_substitution(left_can_interface)
    right_can_interface_str = context.perform_substitution(right_can_interface)
    
    xacro_path = os.path.join(
        get_package_share_directory("openarm_description"),
        "urdf", "robot", "v10.urdf.xacro"
    )
    
    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": arm_type_str,
            "bimanual": "true",
            "use_fake_hardware": use_fake_hardware_str,
            "ros2_control": "true",
            "hand": "true",
            "left_can_interface": left_can_interface_str,
            "right_can_interface": right_can_interface_str,
        },
    ).toprettyxml(indent="  ")
    
    return robot_description


def _shutdown_can_manager():
    """Shutdown handler for CAN manager - disables motors and brings down CAN."""
    global _can_manager
    if _can_manager is not None:
        print("\n[Launch] Shutting down CAN manager...")
        _can_manager.shutdown()
        _can_manager = None
    
    # Also cleanup all spawned processes
    _cleanup_all_processes()


def launch_setup(context: LaunchContext, *args, **kwargs):
    """Generate launch description based on selected mode."""
    global _can_manager
    
    # Get parameters
    mode = context.perform_substitution(LaunchConfiguration('mode'))
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    arm_type = LaunchConfiguration('arm_type')
    left_can_interface = LaunchConfiguration('left_can_interface')
    right_can_interface = LaunchConfiguration('right_can_interface')
    left_can_str = context.perform_substitution(left_can_interface)
    right_can_str = context.perform_substitution(right_can_interface)
    enforce_follower_can = context.perform_substitution(LaunchConfiguration('enforce_follower_can'))
    robot_id = LaunchConfiguration('robot_id')
    allow_partial_joints = LaunchConfiguration('allow_partial_joints')
    use_trac_ik = context.perform_substitution(LaunchConfiguration('use_trac_ik'))
    manage_can = context.perform_substitution(LaunchConfiguration('manage_can'))
    
    print(f"\n{'='*80}")
    print(f"OPENARM UNIFIED LAUNCH - MODE: {mode.upper()}")
    print(f"{'='*80}\n")
    
    # Force follower CAN mapping to avoid accidental leader-bus binding.
    # Physical wiring: can0=right_follower, can1=left_follower, can2=right_leader, can3=left_leader
    if enforce_follower_can.lower() == "true":
        if left_can_str != "can1" or right_can_str != "can0":
            print(
                f"[Launch] overriding CAN mapping "
                f"(left={left_can_str}, right={right_can_str}) -> follower mapping (left=can1, right=can0)"
            )
        left_can_str = "can1"
        right_can_str = "can0"

    # CAN Interface Management (only if not using fake hardware)
    if manage_can.lower() == "true" and use_fake_hardware_str.lower() != "true":
        if CAN_MANAGER_AVAILABLE:
            print("[Launch] Setting up CAN interfaces...")
            _can_manager = CANManager(
                left_can=left_can_str,
                right_can=right_can_str,
                auto_setup=False,
                auto_shutdown=False,  # We handle shutdown via launch events
            )
            
            # Setup CAN interfaces
            if _can_manager.setup_interfaces():
                print("[Launch] CAN interfaces configured")
                
                # Register shutdown handler for graceful cleanup
                # This will be called when the launch system shuts down
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
        else:
            print("[Launch] CAN manager not available - skipping CAN setup")
            print("[Launch]    Run 'colcon build --packages-select openarm_bringup' first")
    elif use_fake_hardware_str.lower() == "true":
        print("[Launch] Using fake hardware - skipping CAN setup")
    else:
        print("[Launch] CAN management disabled (manage_can:=false)")
    
    # Generate robot description
    robot_description = generate_robot_description(
        context,
        arm_type,
        use_fake_hardware,
        TextSubstitution(text=left_can_str),
        TextSubstitution(text=right_can_str),
    )
    robot_description_param = {"robot_description": robot_description}
    
    # Determine which controllers to load
    if mode == "streaming":
        robot_controller = "forward_position_controller"
        use_trajectory_mode = "false"
    else:
        robot_controller = "joint_trajectory_controller"
        use_trajectory_mode = "true"
    
    # Base controller configuration
    controllers_file = PathJoinSubstitution([
        FindPackageShare("openarm_bringup"),
        "config", "v10_controllers", "openarm_v10_bimanual_controllers.yaml"
    ])
    
    # Optional override for partial joints (teleop)
    # Load from mqtt_bridge (single source of truth)
    controller_params = [robot_description_param, controllers_file]
    if context.perform_substitution(allow_partial_joints) == "true":
        teleop_overrides = PathJoinSubstitution([
            FindPackageShare("mqtt_bridge"),
            "scripts", "openarm", "openarm_bringup", 
            "config", "v10_controllers", "teleop_overrides.yaml"
        ])
        controller_params.append(teleop_overrides)
        print("Loading teleop overrides from mqtt_bridge/scripts/openarm/")
    
    # Common nodes: robot_state_publisher + ros2_control
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param],
    )
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=controller_params,
    )
    
    # Controller spawners (delayed to let ros2_control start)
    # Spawn each controller separately with proper sequencing
    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,  # Wait longer for controller_manager
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            )
        ],
    )
    
    # Arm controllers (depends on mode)
    if mode == "streaming":
        left_ctrl = "left_forward_position_controller"
        right_ctrl = "right_forward_position_controller"
    else:
        left_ctrl = "left_joint_trajectory_controller"
        right_ctrl = "right_joint_trajectory_controller"
    
    # Spawn left arm controller - load and configure separately
    left_arm_controller_spawner = TimerAction(
        period=4.5,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[left_ctrl, "-c", "/controller_manager", "--activate-as-group"],
            )
        ],
    )
    
    # Spawn right arm controller
    right_arm_controller_spawner = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[right_ctrl, "-c", "/controller_manager", "--activate-as-group"],
            )
        ],
    )
    
    # Spawn left gripper controller
    left_gripper_controller_spawner = TimerAction(
        period=7.5,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["left_gripper_controller", "-c", "/controller_manager"],
            )
        ],
    )
    
    # Spawn right gripper controller
    right_gripper_controller_spawner = TimerAction(
        period=9.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["right_gripper_controller", "-c", "/controller_manager"],
            )
        ],
    )
    
    # MoveIt2 configuration (for moveit/moveit_rviz/moveit_mqtt modes)
    moveit_nodes = []
    if mode in ["moveit", "moveit_rviz", "moveit_mqtt"]:
        from moveit_configs_utils import MoveItConfigsBuilder
        
        # Build MoveIt configs
        moveit_config_builder = MoveItConfigsBuilder(
            "openarm", package_name="openarm_bimanual_moveit_config"
        )
        
        # Override kinematics with TRAC-IK if requested
        # Load from mqtt_bridge (single source of truth)
        if use_trac_ik.lower() == "true":
            kinematics_yaml = PathJoinSubstitution([
                FindPackageShare("mqtt_bridge"),
                "scripts", "openarm", "openarm_bimanual_moveit_config",
                "config", "kinematics_trac_ik.yaml"
            ])
            print("Using TRAC-IK solver from mqtt_bridge/scripts/openarm/")
        else:
            kinematics_yaml = PathJoinSubstitution([
                FindPackageShare("openarm_bimanual_moveit_config"),
                "config", "kinematics.yaml"
            ])
            print("[warn] using default KDL solver (consider use_trac_ik:=true)")
        
        moveit_config = moveit_config_builder.to_moveit_configs()
        
        # Override with optimized configs
        moveit_params = moveit_config.to_dict()
        moveit_params.update({"robot_description": robot_description})
        
        # move_group node
        move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_params],
        )
        moveit_nodes.append(move_group_node)
        
        # RViz (only for moveit_rviz mode)
        if mode == "moveit_rviz":
            rviz_config = PathJoinSubstitution([
                FindPackageShare("openarm_bimanual_moveit_config"),
                "config", "moveit.rviz"
            ])
            rviz_node = Node(
                package="rviz2",
                executable="rviz2",
                output="log",
                arguments=["-d", rviz_config],
                parameters=[moveit_params],
            )
            moveit_nodes.append(rviz_node)
    
    # MQTT bridge (for trajectory/streaming/moveit_mqtt modes)
    mqtt_nodes = []
    if mode in ["trajectory", "streaming", "moveit_mqtt"]:
        mqtt_bridge_params_file = PathJoinSubstitution([
            FindPackageShare("mqtt_bridge"),
            "config", "params.yaml"
        ])
        mqtt_mapping_file = PythonExpression([
            "'", FindPackageShare("mqtt_bridge"), "/config/mappings/' + '", robot_id, "' + '.yaml'"
        ])
        
        # Load twin_uuid from robot mapping file
        # (needed by trajectory aggregator and cartesian pose publisher)
        mqtt_bridge_share = get_package_share_directory("mqtt_bridge")
        robot_id_str = context.perform_substitution(robot_id)
        mapping_file_path = os.path.join(mqtt_bridge_share, "config", "mappings", f"{robot_id_str}.yaml")
        
        # Load mapping config to get twin_uuid and openarm thresholds
        twin_uuid_param = ""
        velocity_warn_threshold = 2.0   # fallback
        torque_warn_threshold = 50.0    # fallback

        try:
            with open(mapping_file_path, 'r') as f:
                mapping_config = yaml.safe_load(f)
                twin_uuid_param = mapping_config.get('metadata', {}).get('twin_uuid', '')
                openarm_cfg = mapping_config.get('openarm', {})
                velocity_warn_threshold = float(openarm_cfg.get('velocity_warn_threshold', velocity_warn_threshold))
                torque_warn_threshold = float(openarm_cfg.get('torque_warn_threshold', torque_warn_threshold))

                print(f"[Launch] Loaded from {robot_id_str}.yaml:")
                print(f"[Launch]   twin_uuid: {twin_uuid_param}")
                print(f"[Launch]   vel_warn={velocity_warn_threshold} rad/s, torque_warn={torque_warn_threshold} Nm")
        except Exception as e:
            print(f"[Launch] could not load mapping config: {e}")
            print(f"[Launch]    Using defaults where possible")
        
        # Load publish_rate and CYBERWAVE token from params.yaml
        params_file_path = os.path.join(mqtt_bridge_share, "config", "params.yaml")
        publish_rate = 20.0  # fallback default
        try:
            with open(params_file_path, 'r') as f:
                params_config = yaml.safe_load(f)
                mqtt_node_params = params_config.get('/mqtt_bridge_node', {})
                ros_params = mqtt_node_params.get('ros__parameters', {})
                publish_rate = float(ros_params.get('publish_rate', publish_rate))
                print(f"[Launch] loaded publish_rate={publish_rate} Hz from params.yaml")
        except Exception as e:
            print(f"[Launch] could not load publish_rate from params.yaml: {e}")

        # Load .env file from mqtt_bridge package (if present) so credentials are
        # available even when the shell session hasn't sourced it manually.
        env_file_path = os.path.join(mqtt_bridge_share, '..', '..', '..', '..', 'src', 'mqtt_bridge', '.env')
        env_file_path = os.path.normpath(env_file_path)
        if not os.path.isfile(env_file_path):
            # Fallback: find .env relative to this launch file's source tree
            env_file_path = os.path.join(
                os.path.dirname(os.path.realpath(__file__)),
                '..', '..', '..', 'mqtt_bridge', '.env'
            )
            env_file_path = os.path.normpath(env_file_path)
        if os.path.isfile(env_file_path):
            with open(env_file_path, 'r') as _ef:
                for _line in _ef:
                    _line = _line.strip()
                    if _line and not _line.startswith('#') and '=' in _line:
                        _k, _, _v = _line.partition('=')
                        _k = _k.strip()
                        _v = _v.strip().strip('"').strip("'")
                        if _k and _k not in os.environ:
                            os.environ[_k] = _v

        # Load CYBERWAVE token with priority:
        # 1. CYBERWAVE_API_KEY environment variable (or .env file above)
        # 2. params.yaml cyberwave_token field
        cyberwave_token = os.environ.get('CYBERWAVE_API_KEY', '')
        
        if not cyberwave_token:
            try:
                with open(params_file_path, 'r') as f:
                    params_config = yaml.safe_load(f)
                    mqtt_node_params = params_config.get('/mqtt_bridge_node', {})
                    ros_params = mqtt_node_params.get('ros__parameters', {})
                    cyberwave_token = ros_params.get('cyberwave_token', '')
                    if cyberwave_token:
                        print(f"[Launch] loaded CYBERWAVE token from params.yaml")
            except Exception as e:
                print(f"[Launch] could not load params.yaml for token: {e}")
        else:
            print(f"[Launch] using CYBERWAVE_API_KEY from environment")
        
        if not cyberwave_token:
            print(f"[Launch] no CYBERWAVE token found - MQTT publishing will be disabled")
            print(f"[Launch]    set CYBERWAVE_API_KEY env var or configure cyberwave_token in params.yaml")
            # Final fallback: read from /etc/cyberwave/credentials.json
            try:
                with open("/etc/cyberwave/credentials.json") as _f:
                    _creds = json.load(_f)
                cyberwave_token = _creds.get("api_key") or _creds.get("token", "")
                if cyberwave_token:
                    print(f"[Launch] loaded CYBERWAVE token from /etc/cyberwave/credentials.json")
            except Exception:
                pass

        mqtt_bridge_node = Node(
            package="mqtt_bridge",
            executable="mqtt_bridge_node",
            output="screen",
            parameters=[
                mqtt_bridge_params_file,
                {
                    "robot_id": robot_id,
                    "mapping_file": mqtt_mapping_file,
                },
            ],
            additional_env={'CYBERWAVE_API_KEY': cyberwave_token},
        )
        mqtt_nodes.append(mqtt_bridge_node)

        # Add Cartesian pose publisher for end-effector and base poses
        cartesian_pose_publisher_node = Node(
            package="openarm_cyberwave",
            executable="openarm_cartesian_pose_publisher.py",
            name="openarm_cartesian_pose_publisher",
            output="screen",
            parameters=[
                {
                    "publish_rate": publish_rate,
                    "cartesian_pose_rate": publish_rate,
                    "robot_id": robot_id,
                    "twin_uuid": twin_uuid_param,
                    "base_frame": "world",
                    "left_ee_frame": "openarm_left_hand_tcp",
                    "right_ee_frame": "openarm_right_hand_tcp",
                    "torso_frame": "openarm_body_link0",
                    "velocity_warn_threshold": velocity_warn_threshold,
                    "torque_warn_threshold": torque_warn_threshold,
                },
            ],
            additional_env={'CYBERWAVE_API_KEY': cyberwave_token},
        )
        mqtt_nodes.append(cartesian_pose_publisher_node)
        
        # Note: MoveIt2 plugin runs INSIDE mqtt_bridge_node when
        # motion_controller.type="moveit" is set in the mapping config.
        # No separate node needed!
    
    # Print configuration summary
    print(f"Controllers: {left_ctrl}, {right_ctrl}")
    print(f"MoveIt2: {'Enabled' if moveit_nodes else 'Disabled'}")
    print(f"MQTT Bridge: {'Enabled' if mqtt_nodes else 'Disabled'}")
    print(f"RViz: {'Enabled' if mode == 'moveit_rviz' else 'Disabled'}")
    print(f"{'='*80}\n")
    
    # Assemble launch description
    return [
        robot_state_pub,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        left_arm_controller_spawner,
        right_arm_controller_spawner,
        left_gripper_controller_spawner,
        right_gripper_controller_spawner,
    ] + moveit_nodes + mqtt_nodes


def generate_launch_description():
    """Generate launch description with mode selection."""
    
    declared_arguments = [
        DeclareLaunchArgument(
            "mode",
            default_value="trajectory",
            choices=["trajectory", "streaming", "moveit", "moveit_rviz", "moveit_mqtt"],
            description="Control mode: trajectory (direct), streaming (low-latency), "
                       "moveit (planning only), moveit_rviz (planning+viz), "
                       "moveit_mqtt (collision-aware teleop)"
        ),
        DeclareLaunchArgument(
            "robot_id",
            default_value="robot_openarm_v1",
            description="Robot ID for MQTT bridge mapping"
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake/mock hardware for testing"
        ),
        DeclareLaunchArgument(
            "arm_type",
            default_value="v10",
            description="Arm type version"
        ),
        DeclareLaunchArgument(
            "left_can_interface",
            default_value="can3",
            description="CAN interface for left arm"
        ),
        DeclareLaunchArgument(
            "right_can_interface",
            default_value="can2",
            description="CAN interface for right arm"
        ),
        DeclareLaunchArgument(
            "allow_partial_joints",
            default_value="true",
            description="Allow partial joint trajectory updates (teleop mode)"
        ),
        DeclareLaunchArgument(
            "use_trac_ik",
            default_value="true",
            description="Use TRAC-IK solver instead of KDL (requires: sudo apt install ros-humble-trac-ik-kinematics-plugin)"
        ),
        DeclareLaunchArgument(
            "manage_can",
            default_value="true",
            description="Manage CAN interfaces (setup on start, disable motors + teardown on shutdown)"
        ),
        DeclareLaunchArgument(
            "enforce_follower_can",
            default_value="true",
            description="Force follower CAN mapping (left=can1, right=can0) for this launcher",
        ),
    ]
    
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
