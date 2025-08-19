"""
Refined sim2real deployment script for the Hector robot.

Features a keyboard-controlled state machine and a non-blocking, threaded
MuJoCo visualizer.

Keyboard Controls (in visualizer window):
- 'D': Switch to Damping state
- 'S': Switch to PD Standing state
- 'P': Switch to Policy state
"""

import enum
import time
import select
import threading
from dataclasses import dataclass, field

import numpy as np
import onnxruntime as rt
import lcm
from scipy.spatial.transform import Rotation
import glfw

from exlcm import low_cmd_t, low_state_t
from py_utils.rt_loop import loop_func

import mujoco
import mujoco.viewer

# Import the pynput listener
from pynput import keyboard

# --- Constants and Configuration ---
# LCM Channels
_CHN_LOW_STATE = "HECTOR_HW_LOW_STATE"
_CHN_LOW_COMMAND = "HECTOR_HW_LOW_CMD"

# Robot Parameters
_NUM_JOINTS = 18
_DEFAULT_Q = np.array([
    0.00, 0.00, 0.785, -1.57, 0.785,  # Left Front
    0.00, 0.00, 0.785, -1.57, 0.785,  # Right Front
    0.00, 0.785, 0.000, -1.57,        # Left Hind
    0.00, 0.785, 0.000, -1.57         # Right Hind
])

# Standing Pose
_STANDING_Q = np.array([
    0.00, 0.20, 0.785, -1.57, 0.785 + 0.15,
    0.00, -0.20, 0.785, -1.57, 0.785 + 0.15,
    0.00, 0.785, 0.000, -1.57,
    0.00, 0.785, 0.000, -1.57
])
_STANDING_KP = np.array([
    50.0, 50.0, 50.0, 50.0, 6.0,
    50.0, 50.0, 50.0, 50.0, 6.0,
    5.0, 5.0, 5.0, 5.0,
    5.0, 5.0, 5.0, 5.0
])
_STANDING_KD = np.array([
    0.75, 0.75, 0.75, 0.75, 0.15,
    0.75, 0.75, 0.75, 0.75, 0.15,
    0.1, 0.1, 0.1, 0.1,
    0.1, 0.1, 0.1, 0.1
])

# Policy Gains
_POLICY_KP = _STANDING_KP * 1.1
_POLICY_KD = _STANDING_KD * 2.0

_INTERPOLATION_DURATION = 2.0

# --- Enums and Dataclasses ---

class ControlState(enum.Enum):
    """Defines the robot's control state."""
    DAMPING = 0
    STANDING = 1
    POLICY = 2

@dataclass
class ImuState:
    """Holds IMU sensor data."""
    rpy: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gyro: np.ndarray = field(default_factory=lambda: np.zeros(3))
    acc: np.ndarray = field(default_factory=lambda: np.zeros(3))

@dataclass
class JointState:
    """Holds joint sensor data."""
    q: np.ndarray = field(default_factory=lambda: np.zeros(_NUM_JOINTS))
    dq: np.ndarray = field(default_factory=lambda: np.zeros(_NUM_JOINTS))

@dataclass
class JointCommand:
    """Holds the desired joint commands to be sent to the hardware."""
    q_des: np.ndarray = field(default_factory=lambda: np.zeros(_NUM_JOINTS))
    kp: np.ndarray = field(default_factory=lambda: np.zeros(_NUM_JOINTS))
    kd: np.ndarray = field(default_factory=lambda: np.zeros(_NUM_JOINTS))
    enable: bool = True

# --- Main Controller Class ---

class HectorController:
    """Manages robot state, communication, and policy execution."""

    def __init__(self, policy_path: str, ctrl_dt: float, action_scale: float):
        # Control parameters
        self._ctrl_dt = ctrl_dt
        self._action_scale = action_scale

        # --- NEW: Central dictionary for target gains ---
        self._TARGET_GAINS = {
            ControlState.DAMPING: (np.zeros(_NUM_JOINTS), np.full(_NUM_JOINTS, 3.0)),
            ControlState.STANDING: (_STANDING_KP, _STANDING_KD),
            ControlState.POLICY: (_POLICY_KP, _POLICY_KD),
        }
        # --- NEW: Interpolation attributes ---
        self._is_interpolating = False
        self._interpolation_start_time = 0.0
        self._kp_start = np.zeros(_NUM_JOINTS)
        self._kd_start = np.zeros(_NUM_JOINTS)
        self._kp_target = np.zeros(_NUM_JOINTS)
        self._kd_target = np.zeros(_NUM_JOINTS)
        # Initialize current gains to the DAMPING state target
        self.joint_cmd = JointCommand()
        self.joint_cmd.kp, self.joint_cmd.kd = self._TARGET_GAINS[ControlState.DAMPING]
        # State machine
        self.state = ControlState.DAMPING
        #self._state_lock = threading.Lock()
        self._state_start_time = time.time()
        print(f"🚀 Starting in {self.state.name} state.")

        # LCM setup
        self.lc = lcm.LCM()
        self.lc.subscribe(_CHN_LOW_STATE, self._lcm_state_callback)
        self._low_state_msg = low_state_t()
        self._lcm_data_received = False

        # Robot state buffers
        self.imu = ImuState()
        self.joint_state = JointState()
        self.joint_cmd = JointCommand()

        # ONNX Policy setup
        self._policy = rt.InferenceSession(policy_path, providers=["CPUExecutionProvider"])
        self._output_names = ["continuous_actions"]
        print("✅ ONNX policy loaded successfully.")

        # Observation history buffers
        self._obs_size_n = 82
        self._last_action = np.zeros(_NUM_JOINTS, dtype=np.float32)
        self._last_obs_1 = np.zeros(self._obs_size_n, dtype=np.float32)
        self._last_obs_2 = np.zeros(self._obs_size_n, dtype=np.float32)
        self._last_obs_3 = np.zeros(self._obs_size_n, dtype=np.float32)

        # Gait parameters
        self._phase = np.array([0.0, np.pi])
        self._gait_freq = 1.8
        self._phase_dt = 2 * np.pi * self._gait_freq * self._ctrl_dt

        # User commands
        self._twist_command = np.array([1.0, 0.0, 0.0])
        self._body_command = np.zeros(12)
        self._body_command[0] = 0.55
        
        # Low pass filters hw obs
        self._gyro_last = np.zeros(3)
        self._acc_last = np.zeros(3)
        self._q_last = np.zeros(_NUM_JOINTS)
        self._dq_last = np.zeros(_NUM_JOINTS)


    def switch_state(self, new_state: ControlState):
        """Triggers a smooth transition to a new control state."""
        if self.state == new_state and not self._is_interpolating:
            return
        print(f"\n---> Requesting transition to {new_state.name} state...")
        self._is_interpolating = True
        self._interpolation_start_time = time.time()
        
        # Store current and target gains for interpolation
        self._kp_start = self.joint_cmd.kp.copy()
        self._kd_start = self.joint_cmd.kd.copy()
        self._kp_target, self._kd_target = self._TARGET_GAINS[new_state]
        
        # Store the state we are transitioning to
        self._next_state = new_state

    def _lcm_state_callback(self, channel, data):
        self._low_state_msg = low_state_t.decode(data)
        if not self._lcm_data_received:
            self.joint_cmd.q_des = np.array(self._low_state_msg.q)
        self._lcm_data_received = True

    def _update_internal_state(self):
        self.imu.rpy = np.array(self._low_state_msg.rpy)
        self.imu.gyro = np.array(self._low_state_msg.gyroscope)
        self.imu.acc = np.array(self._low_state_msg.accelerometer)
        self.joint_state.q = np.array(self._low_state_msg.q)
        self.joint_state.dq = np.array(self._low_state_msg.dq)

    def _get_observation(self) -> np.ndarray:
        
        imu_xmat = Rotation.from_euler('xyz', self.imu.rpy, degrees=False).as_matrix()
        gravity_vec = imu_xmat.T @ np.array([0, 0, -1])
        phase = np.concatenate([np.cos(self._phase), np.sin(self._phase)])
        command = np.hstack([self._twist_command, self._body_command])
        
        # why phase is not updating?
        print(self._phase)
        
        # Low-pass filter some hw obs
        gyro_f = 0.0*self._gyro_last + 1.0*self.imu.gyro
        acc_f = 0.0*self._acc_last + 1.0*self.imu.acc
        q_f = 0.0*self._q_last + 1.0*self.joint_state.q
        dq_f = 0.5*self._dq_last + 0.5*self.joint_state.dq

        obs_n = np.hstack([
            gyro_f,
            acc_f,
            gravity_vec,
            q_f - _DEFAULT_Q,
            dq_f,
            self._last_action,
            phase,
            command
        ]).astype(np.float32)
        
        #print(obs_n)

        obs = np.hstack([
            obs_n,
            self._last_obs_1,
            self._last_obs_2,
            self._last_obs_3
            ])
        
        # Update buffer
        self._gyro_last = self.imu.gyro
        self._acc_last = self.imu.acc
        self._q_last = self.joint_state.q
        self._dq_last = self.joint_state.dq
        
        # Update history obs
        self._last_obs_3, self._last_obs_2, self._last_obs_1 = self._last_obs_2, self._last_obs_1, obs_n
        
        return obs
    
    def _update_gains(self):
        """Handles the linear interpolation of gains if a transition is active."""
        if not self._is_interpolating:
            return

        elapsed_time = time.time() - self._interpolation_start_time
        progress = min(elapsed_time / _INTERPOLATION_DURATION, 1.0)

        # Linearly interpolate kp and kd
        self.joint_cmd.kp = self._kp_start + (self._kp_target - self._kp_start) * progress
        self.joint_cmd.kd = self._kd_start + (self._kd_target - self._kd_start) * progress

        # If interpolation is complete, finalize the state change
        if progress >= 1.0:
            self._is_interpolating = False
            self.state = self._next_state
            # Ensure gains are exactly the target values
            #self.joint_cmd.kp = self._kp_target.copy()
            #self.joint_cmd.kd = self._kd_target.copy()
            print(f"✔️ Transition complete. Now in {self.state.name} state.")


    def run_control_tick(self):
        """The main control loop callback, executed at `ctrl_dt`."""
        if not self._lcm_data_received:
            return

        self._update_internal_state()
        self._get_observation()
        
        # --- NEW: Handle gain updates every tick ---
        self._update_gains()

        # Determine desired joint positions based on the CURRENT state
        current_state = self.state
        
        if current_state == ControlState.DAMPING:
            self.joint_cmd.q_des = self.joint_state.q
            
        elif current_state == ControlState.STANDING:
            self.joint_cmd.q_des = _STANDING_Q
            
        elif current_state == ControlState.POLICY:
            obs = self._get_observation()
            onnx_input = {"obs": obs.reshape(1, -1)}
            action = self._policy.run(self._output_names, onnx_input)[0][0]
            # Low pass filter for action
            action_f = 0.2*self._last_action + 0.8*action
            self._last_action = action.copy()
            
            self.joint_cmd.q_des = action_f * self._action_scale + _DEFAULT_Q   
            phase_tp1 = self._phase + self._phase_dt
            self._phase = np.fmod(phase_tp1 + np.pi, 2 * np.pi) - np.pi         

    def send_command(self):
        if not self._lcm_data_received: return
        cmd_msg = low_cmd_t()
        cmd_msg.q = self.joint_cmd.q_des
        cmd_msg.dq = np.zeros(_NUM_JOINTS, dtype=np.float32)
        cmd_msg.tau = np.zeros(_NUM_JOINTS, dtype=np.float32)
        cmd_msg.kp = self.joint_cmd.kp
        cmd_msg.kd = self.joint_cmd.kd
        val_enable = 1 if self.joint_cmd.enable else 0
        for i in range(_NUM_JOINTS):
            cmd_msg.enable[i] = val_enable
            cmd_msg.calibrate[i] = 0
        self.lc.publish(_CHN_LOW_COMMAND, cmd_msg.encode())

    def lcm_handle_loop(self):
        rfds, _, _ = select.select([self.lc.fileno()], [], [], 0)
        if rfds: self.lc.handle()

def visualizer_thread_func(controller, model, data):
    """Function to run the MuJoCo viewer in a separate thread."""
    # Launch the viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        
        print("\nVisualizer running. Press keys in the viewer window.")
        print("  D: Damping | S: Standing | P: Policy")
        
        # The rest of the function remains the same
        while viewer.is_running():
            if controller._lcm_data_received:
                hw_q = controller.joint_state.q
                hw_rpy = controller.imu.rpy
                
                R_sensor = Rotation.from_euler('xyz', hw_rpy, degrees=False)
                qx, qy, qz, qw = R_sensor.as_quat()
                quat_wxyz = (qw, qx, qy, qz)
                
                data.qpos[3:7] = quat_wxyz
                data.qpos[7:7 + _NUM_JOINTS] = hw_q
                mujoco.mj_fwdPosition(model, data)
                viewer.sync()
            time.sleep(0.01)


if __name__ == "__main__":
    policy_path = "onnx/hector_wbc_s2_lows_0819_2.onnx"
    ctrl_dt = 0.02
    action_scale = 0.60

    controller = HectorController(
        policy_path=policy_path, ctrl_dt=ctrl_dt, action_scale=action_scale
    )

    lcm_loop = loop_func(name="lcm_handle", period=0.001, cb=controller.lcm_handle_loop)
    cmd_loop = loop_func(name="lcm_command", period=0.001, cb=controller.send_command)
    control_loop = loop_func(name="main_control", period=ctrl_dt, cb=controller.run_control_tick)

    # --- Define the key callback function in the main scope ---
    def on_press(key):
        print(controller._twist_command)
        try:
            char = key.char
            if char == 'd':
                controller.switch_state(ControlState.DAMPING)
            elif char == 's':
                controller.switch_state(ControlState.STANDING)
            elif char == 'p':
                controller.switch_state(ControlState.POLICY)
            elif char == 'w':
                controller._twist_command[0] += 0.5
            elif char == 'x':
                controller._twist_command[0] -= 0.5
        except AttributeError:
            # This handles special keys (e.g., Shift, Ctrl) which don't have a 'char' attribute
            pass
    # --- Start the pynput listener in a non-blocking thread ---
    key_listener = keyboard.Listener(on_press=on_press)
    key_listener.start()
    
    lcm_loop = loop_func(name="lcm_handle", period=0.001, cb=controller.lcm_handle_loop)
    cmd_loop = loop_func(name="lcm_command", period=0.001, cb=controller.send_command)
    control_loop = loop_func(name="main_control", period=ctrl_dt, cb=controller.run_control_tick)
    
    mjcf_path = "xmls/hector_v2.xml"
    model = mujoco.MjModel.from_xml_path(mjcf_path)
    data = mujoco.MjData(model)
    vis_thread = threading.Thread(target=visualizer_thread_func, args=(controller, model, data), daemon=True)
    
    try:
        control_loop.start()
        lcm_loop.start()
        cmd_loop.start()
        vis_thread.start()
        print("\n✅ All control loops running.")
        while True: time.sleep(1)
    except KeyboardInterrupt:
        print("\nCtrl+C received. Shutting down...")
    finally:
        cmd_loop.shutdown()
        lcm_loop.shutdown()
        control_loop.shutdown()
        vis_thread.join()
        print("\nScript finished.")