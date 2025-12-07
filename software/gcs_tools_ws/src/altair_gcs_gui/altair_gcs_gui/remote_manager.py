import subprocess
import threading
import time

class RemoteManager:
    def __init__(self, logger):
        self.logger = logger
        self.rpi_ip = "192.168.137.204" # Default, should be configurable or discovered
        self.user = "konn"
        self.ssh_key = None # Use agent or default key

    def set_target(self, ip, user="konn"):
        self.rpi_ip = ip
        self.user = user

    def _run_ssh_command(self, command):
        """Runs a command via SSH in a separate thread to avoid blocking GUI."""
        def task():
            full_cmd = f"ssh {self.user}@{self.rpi_ip} \"{command}\""
            self.logger.info(f"Executing: {full_cmd}")
            try:
                # Use shell=True for Windows to handle ssh command properly
                result = subprocess.run(full_cmd, shell=True, capture_output=True, text=True)
                if result.returncode == 0:
                    self.logger.info(f"Success: {result.stdout.strip()}")
                else:
                    self.logger.error(f"Error: {result.stderr.strip()}")
            except Exception as e:
                self.logger.error(f"SSH Exception: {e}")

        threading.Thread(target=task, daemon=True).start()

    def start_bridge_node(self):
        # Command to start the bridge node (Node B)
        # Use 'screen' to detach
        cmd = "screen -dmS bridge bash -c 'source ~/altair_project/software/RPi4_1_ws/install/setup.bash; ros2 launch rpi4_1_bringup bridge.launch.py'"
        self._run_ssh_command(cmd)

    def stop_bridge_node(self):
        # Kill the screen session
        cmd = "screen -X -S bridge quit"
        self._run_ssh_command(cmd)

    def start_controller_node(self):
        # Command to start the controller node (Node C)
        # Assuming Node C is accessed via Node B (konnpi) acting as jump host or direct connection
        # If Node C (konnpi2) is separate, we need to SSH into it.
        # Simple approach: SSH to konnpi, then SSH to konnpi2, then launch.
        # But 'screen' inside 'ssh' inside 'ssh' is complex.
        # Alternative: Just launch it on Node B if Node C isn't physically ready yet, OR
        # Assume user has set up SSH config so 'ssh konnpi2' works from GCS?
        # Let's assume we run it via jump host from Node B.
        
        # Command executed on Node B: "ssh konnpi2 'screen -dmS controller ...'"
        # Note: This requires Node B to have passwordless SSH to Node C.
        
        nested_cmd = "screen -dmS controller bash -c 'source ~/altair_project/software/RPi4_2_ws/install/setup.bash; ros2 launch altair_controller controller.launch.py'"
        full_ssh_cmd = f"ssh konn@konnpi2 \"{nested_cmd}\""
        
        self.logger.info("Attempting to launch Controller on Node C via Node B...")
        self._run_ssh_command(full_ssh_cmd)

    def stop_controller_node(self):
        nested_cmd = "screen -X -S controller quit"
        full_ssh_cmd = f"ssh konn@konnpi2 \"{nested_cmd}\""
        self._run_ssh_command(full_ssh_cmd)

    def reboot_rpi(self):
        self._run_ssh_command("sudo reboot")

    def check_connection(self):
        """Simple ping check (synchronous for now, or use separate thread if blocking)"""
        # On Windows 'ping -n 1'
        try:
            cmd = ["ping", "-n", "1", self.rpi_ip]
            result = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return result.returncode == 0
        except:
            return False
