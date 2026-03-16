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
OpenArm CAN Interface Manager

Provides CAN bus setup and teardown functionality for OpenArm robots.
This module handles:
- CAN interface configuration (bitrate, CAN-FD mode)
- Motor enable/disable commands (Damiao motors)
- Graceful shutdown with motor safety

Usage:
    from openarm_bringup.can_manager import CANManager
    
    # Create manager
    can_mgr = CANManager(left_can="can1", right_can="can0")
    
    # Setup CAN interfaces
    can_mgr.setup_interfaces()
    
    # ... run your ROS2 nodes ...
    
    # Shutdown (disable motors, bring down CAN)
    can_mgr.shutdown()
"""

import os
import subprocess
import signal
import atexit
import time
from typing import Optional, List

# CAN motor commands (Damiao protocol)
MOTOR_CMD_ENABLE = "FFFFFFFFFFFFFFFC"   # Enable motor torque
MOTOR_CMD_DISABLE = "FFFFFFFFFFFFFFFD"  # Disable motor (safe stop)
MOTOR_CMD_CLEAR_ERROR = "FFFFFFFFFFFFFFFE"  # Clear motor errors


class CANManager:
    """
    Manages CAN interface setup and teardown for OpenArm robots.
    
    Handles CAN-FD configuration and motor enable/disable commands.
    Registers shutdown handlers for graceful cleanup on SIGINT/SIGTERM.
    """
    
    # Default configuration (matches openarm_can tooling)
    DEFAULT_BITRATE = 1000000       # 1 Mbps nominal bitrate
    DEFAULT_DBITRATE = 5000000      # 5 Mbps data bitrate (CAN-FD)
    DEFAULT_FD_ENABLED = True       # Enable CAN-FD by default
    DEFAULT_TX_QUEUE_LEN = 65536    # Large TX queue for smooth operation
    
    # Motor IDs (1-8 for joints + gripper)
    MOTOR_IDS = list(range(1, 9))
    
    def __init__(
        self,
        left_can: str = "can1",
        right_can: str = "can0",
        bitrate: int = DEFAULT_BITRATE,
        dbitrate: int = DEFAULT_DBITRATE,
        fd_enabled: bool = DEFAULT_FD_ENABLED,
        auto_setup: bool = False,
        auto_shutdown: bool = True,
    ):
        """
        Initialize CAN Manager.
        
        Args:
            left_can: CAN interface for left arm (default: can1)
            right_can: CAN interface for right arm (default: can0)
            bitrate: CAN nominal bitrate (default: 1000000)
            dbitrate: CAN-FD data bitrate (default: 5000000)
            fd_enabled: Enable CAN-FD mode (default: True)
            auto_setup: Automatically setup interfaces on init (default: False)
            auto_shutdown: Register shutdown handlers (default: True)
        """
        self.left_can = left_can
        self.right_can = right_can
        self.bitrate = bitrate
        self.dbitrate = dbitrate
        self.fd_enabled = fd_enabled
        self.interfaces = [left_can, right_can]
        
        self._interfaces_configured = False
        self._motors_enabled = False
        self._shutdown_registered = False
        
        if auto_shutdown:
            self._register_shutdown_handlers()
        
        if auto_setup:
            self.setup_interfaces()
    
    def _run_cmd(self, cmd: List[str], check: bool = False, use_sudo: bool = False) -> subprocess.CompletedProcess:
        """Run a shell command, optionally with sudo."""
        if use_sudo and os.geteuid() != 0:
            cmd = ["sudo"] + cmd
        
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                check=check
            )
            return result
        except subprocess.CalledProcessError as e:
            print(f"[CAN Manager] Command failed: {' '.join(cmd)}")
            print(f"[CAN Manager] Error: {e.stderr}")
            raise
    
    def _interface_exists(self, iface: str) -> bool:
        """Check if a CAN interface exists."""
        result = self._run_cmd(["ip", "link", "show", iface])
        return result.returncode == 0
    
    def _interface_is_up(self, iface: str) -> bool:
        """Check if a CAN interface is UP."""
        result = self._run_cmd(["ip", "-br", "link", "show", iface])
        if result.returncode == 0:
            return "UP" in result.stdout
        return False
    
    def setup_interface(self, iface: str) -> bool:
        """
        Setup a single CAN interface.
        
        Args:
            iface: Interface name (e.g., "can0")
            
        Returns:
            True if successful, False otherwise
        """
        if not self._interface_exists(iface):
            print(f"[CAN Manager] WARNING: Interface {iface} not found, skipping")
            return False
        
        print(f"[CAN Manager] Configuring {iface}...")
        
        # CAN configuration requires root privileges
        needs_sudo = os.geteuid() != 0
        if needs_sudo:
            print(f"[CAN Manager]   (using sudo for privileged operations)")
        
        # Bring interface down first
        self._run_cmd(["ip", "link", "set", iface, "down"], use_sudo=True)
        
        # Configure bitrate and CAN-FD
        if self.fd_enabled:
            cmd = [
                "ip", "link", "set", iface, "type", "can",
                "bitrate", str(self.bitrate),
                "dbitrate", str(self.dbitrate),
                "fd", "on"
            ]
        else:
            cmd = [
                "ip", "link", "set", iface, "type", "can",
                "bitrate", str(self.bitrate)
            ]
        
        result = self._run_cmd(cmd, use_sudo=True)
        if result.returncode != 0:
            print(f"[CAN Manager] ERROR: Failed to configure {iface}")
            if result.stderr:
                print(f"[CAN Manager]   {result.stderr.strip()}")
            return False
        
        # Set TX queue length
        self._run_cmd(["ip", "link", "set", iface, "txqueuelen", str(self.DEFAULT_TX_QUEUE_LEN)], use_sudo=True)
        
        # Bring interface up
        result = self._run_cmd(["ip", "link", "set", iface, "up"], use_sudo=True)
        if result.returncode != 0:
            print(f"[CAN Manager] ERROR: Failed to bring up {iface}")
            if result.stderr:
                print(f"[CAN Manager]   {result.stderr.strip()}")
            return False
        
        print(f"[CAN Manager] ✅ {iface} configured (bitrate={self.bitrate}, dbitrate={self.dbitrate}, fd={'on' if self.fd_enabled else 'off'})")
        return True
    
    def setup_interfaces(self) -> bool:
        """
        Setup all CAN interfaces.
        
        If interfaces are already UP, skip configuration.
        If not running as root and sudo is not passwordless, configuration will fail.
        
        Returns:
            True if at least one interface is available (configured or already up)
        """
        print("\n" + "="*60)
        print("[CAN Manager] SETTING UP CAN INTERFACES")
        print("="*60)
        
        # First check if interfaces are already up
        already_up = []
        need_config = []
        
        for iface in self.interfaces:
            if not self._interface_exists(iface):
                print(f"[CAN Manager] ⚠️  Interface {iface} not found")
                continue
            
            if self._interface_is_up(iface):
                already_up.append(iface)
                print(f"[CAN Manager] ✅ {iface} already UP - skipping configuration")
            else:
                need_config.append(iface)
        
        # If all interfaces are already up, we're done
        if already_up and not need_config:
            print(f"[CAN Manager] ✅ All {len(already_up)} interfaces already configured")
            print("="*60 + "\n")
            self._interfaces_configured = True
            return True
        
        # Try to configure interfaces that need it
        if need_config:
            if os.geteuid() != 0:
                print(f"[CAN Manager] ⚠️  Not running as root - attempting sudo for {need_config}")
                print("[CAN Manager]   If this fails, either:")
                print("[CAN Manager]   1. Run with: sudo ros2 launch ...")
                print("[CAN Manager]   2. Start CAN first: sudo systemctl start openarm-can-setup.service")
                print("[CAN Manager]   3. Configure passwordless sudo for ip commands")
            
            success_count = 0
            for iface in need_config:
                if self.setup_interface(iface):
                    success_count += 1
            
            if success_count > 0:
                print(f"[CAN Manager] ✅ Configured {success_count}/{len(need_config)} interfaces")
        
        total_available = len(already_up) + (success_count if need_config else 0)
        self._interfaces_configured = total_available > 0
        
        if self._interfaces_configured:
            print(f"[CAN Manager] ✅ {total_available} interface(s) available")
        else:
            print("[CAN Manager] ❌ No interfaces available!")
            print("[CAN Manager]   Run: sudo systemctl start openarm-can-setup.service")
        
        print("="*60 + "\n")
        return self._interfaces_configured
    
    def send_motor_command(self, iface: str, motor_id: int, command: str) -> bool:
        """
        Send a command to a motor via CAN.
        
        Args:
            iface: CAN interface
            motor_id: Motor ID (1-8)
            command: Hex command string (e.g., "FFFFFFFFFFFFFFFD")
            
        Returns:
            True if successful
        """
        if not self._interface_is_up(iface):
            return False
        
        # Format motor ID as 3-digit hex
        motor_hex = f"{motor_id:03X}"
        can_frame = f"{motor_hex}#{command}"
        
        result = self._run_cmd(["cansend", iface, can_frame])
        return result.returncode == 0
    
    def disable_motors_on_interface(self, iface: str) -> int:
        """
        Disable all motors on a CAN interface.
        
        Args:
            iface: CAN interface name
            
        Returns:
            Number of motors successfully disabled
        """
        if not self._interface_is_up(iface):
            print(f"[CAN Manager] Interface {iface} is DOWN, skipping motor disable")
            return 0
        
        disabled_count = 0
        for motor_id in self.MOTOR_IDS:
            if self.send_motor_command(iface, motor_id, MOTOR_CMD_DISABLE):
                disabled_count += 1
            # Small delay to prevent CAN bus flooding
            time.sleep(0.01)
        
        return disabled_count
    
    def disable_all_motors(self) -> None:
        """Disable all motors on all configured interfaces."""
        print("\n" + "="*60)
        print("[CAN Manager] DISABLING ALL MOTORS")
        print("="*60)
        
        for iface in self.interfaces:
            print(f"[CAN Manager] Disabling motors on {iface}...")
            count = self.disable_motors_on_interface(iface)
            print(f"[CAN Manager]   Sent disable to {count} motors")
        
        self._motors_enabled = False
        print("[CAN Manager] ✅ All motor disable commands sent")
        print("="*60 + "\n")
    
    def bring_down_interface(self, iface: str) -> bool:
        """Bring down a CAN interface."""
        if not self._interface_exists(iface):
            return False
        
        result = self._run_cmd(["ip", "link", "set", iface, "down"], use_sudo=True)
        return result.returncode == 0
    
    def bring_down_interfaces(self) -> None:
        """Bring down all CAN interfaces."""
        print("[CAN Manager] Bringing down CAN interfaces...")
        for iface in self.interfaces:
            if self.bring_down_interface(iface):
                print(f"[CAN Manager]   ✅ {iface} down")
            else:
                print(f"[CAN Manager]   ⚠️  {iface} not found or already down")
        
        self._interfaces_configured = False
    
    def shutdown(self) -> None:
        """
        Perform graceful shutdown.
        
        1. Disable all motors (safety)
        2. Bring down CAN interfaces
        """
        print("\n" + "="*60)
        print("[CAN Manager] SHUTDOWN - DISABLING MOTORS & CAN")
        print("="*60)
        
        # Step 1: Disable motors (safety first!)
        self.disable_all_motors()
        
        # Small delay to ensure disable commands are processed
        time.sleep(0.1)
        
        # Step 2: Bring down interfaces
        self.bring_down_interfaces()
        
        print("[CAN Manager] ✅ Shutdown complete")
        print("="*60 + "\n")
    
    def _register_shutdown_handlers(self) -> None:
        """Register signal handlers and atexit for graceful shutdown."""
        if self._shutdown_registered:
            return
        
        def signal_handler(signum, frame):
            sig_name = signal.Signals(signum).name
            print(f"\n[CAN Manager] Received {sig_name}, initiating shutdown...")
            self.shutdown()
            # Re-raise to allow normal signal handling
            signal.signal(signum, signal.SIG_DFL)
            os.kill(os.getpid(), signum)
        
        # Register for SIGINT (Ctrl+C) and SIGTERM
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        # Also register with atexit for other exit scenarios
        atexit.register(self.shutdown)
        
        self._shutdown_registered = True
        print("[CAN Manager] Shutdown handlers registered (SIGINT, SIGTERM, atexit)")


# Singleton instance for use across the launch system
_global_can_manager: Optional[CANManager] = None


def get_can_manager(
    left_can: str = "can1",
    right_can: str = "can0",
    **kwargs
) -> CANManager:
    """
    Get or create the global CAN manager instance.
    
    This ensures only one manager exists and handles cleanup.
    """
    global _global_can_manager
    
    if _global_can_manager is None:
        _global_can_manager = CANManager(
            left_can=left_can,
            right_can=right_can,
            **kwargs
        )
    
    return _global_can_manager


def setup_can_for_launch(
    left_can: str = "can1",
    right_can: str = "can0",
    bitrate: int = CANManager.DEFAULT_BITRATE,
    dbitrate: int = CANManager.DEFAULT_DBITRATE,
    fd_enabled: bool = True,
) -> bool:
    """
    Convenience function to setup CAN for ROS2 launch files.
    
    This function:
    1. Creates/gets the global CAN manager
    2. Configures the CAN interfaces
    3. Registers shutdown handlers
    
    Call this at the beginning of your launch setup.
    
    Returns:
        True if at least one interface was configured successfully
    """
    mgr = get_can_manager(
        left_can=left_can,
        right_can=right_can,
        bitrate=bitrate,
        dbitrate=dbitrate,
        fd_enabled=fd_enabled,
        auto_setup=False,
        auto_shutdown=True,
    )
    
    return mgr.setup_interfaces()


if __name__ == "__main__":
    # Test the CAN manager
    import argparse
    
    parser = argparse.ArgumentParser(description="OpenArm CAN Manager")
    parser.add_argument("--left-can", default="can1", help="Left arm CAN interface")
    parser.add_argument("--right-can", default="can0", help="Right arm CAN interface")
    parser.add_argument("--setup", action="store_true", help="Setup CAN interfaces")
    parser.add_argument("--shutdown", action="store_true", help="Shutdown (disable motors, bring down CAN)")
    parser.add_argument("--disable-motors", action="store_true", help="Only disable motors")
    
    args = parser.parse_args()
    
    mgr = CANManager(
        left_can=args.left_can,
        right_can=args.right_can,
        auto_shutdown=False  # Manual control in CLI mode
    )
    
    if args.setup:
        mgr.setup_interfaces()
    elif args.shutdown:
        mgr.shutdown()
    elif args.disable_motors:
        mgr.disable_all_motors()
    else:
        parser.print_help()
