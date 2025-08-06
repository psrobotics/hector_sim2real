# Simple python wrapper for the hardware control

from etils import epath
import onnxruntime as rt

import os
import select
import numpy as np
from scipy.spatial.transform import Rotation
import time

import lcm
from exlcm import low_cmd_t
from exlcm import low_state_t

from dataclasses import dataclass, field

from py_utils.rt_loop import loop_func

import mujoco
import mujoco.viewer
import time

_HERE = epath.Path(__file__).parent
_ONNX_DIR = _HERE / "onnx"

@dataclass
class imu_state:
    rpy: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gyro: np.ndarray = field(default_factory=lambda: np.zeros(3))
    acc: np.ndarray = field(default_factory=lambda: np.zeros(3))

@dataclass
class joint_state:
    q: np.ndarray = field(default_factory=lambda: np.zeros(6))
    dq: np.ndarray = field(default_factory=lambda: np.zeros(6))
    tau_est: np.ndarray = field(default_factory=lambda: np.zeros(6))

@dataclass
class joint_act:
    q: np.ndarray = field(default_factory=lambda: np.zeros(6))
    kp: float = 0.0
    kd: float = 0.0
    calibrate: bool = False
    enable: bool = True
     
class hw_wrapper:
    '''The robot hardware wrapper class'''
    def __init__(
      self,
      default_q: np.ndarray,
      policy_path: str,
      ctrl_dt: float = 0.02,
      act_scale: float = 0.5,
      kp: float = 5.0,
      kd: float = 0.05,
    ):
        # lcm topic names
        self._chn_low_state = "HECTOR_HW_LOW_STATE"
        self._chn_low_command = "HECTOR_HW_LOW_CMD"

        # lcm init
        self.lc = lcm.LCM()
        self.state_sub = self.lc.subscribe(self._chn_low_state, self.low_state_handle)

        self.low_state_msg = low_state_t()
        self.low_cmd_msg = low_cmd_t()
        
        # hw states
        self.imu = imu_state()
        self.joint_state = joint_state()
        self.joint_act = joint_act()
        
        # hw params
        self.mot_dir = np.array([1, 1, 1, 1, 1, 1])
        self.joint_offset = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Init motor kp
        self.joint_act.kp = 0.7
        self.joint_act.kd = 0.08
        
        # Policy onnx params
        self._output_names = ["continuous_actions"]
        self._policy = rt.InferenceSession(
            policy_path, providers=["CPUExecutionProvider"]
        )
        print("ONNX policy init done")

        # Local obs buffer
        self._default_q = default_q
        self._act_scale = act_scale
        
        # Act, obs dim
        self._nu = 18
        self._obs_dim = 70
        
        self._last_action = np.zeros(self._nu, dtype=np.float32)
        self._last_obs = np.zeros(self._obs_dim, dtype=np.float32)
        self._last_last_obs = np.zeros(self._obs_dim, dtype=np.float32)

        # Pre-set gait frequency
        self._phase = np.array([0.0, np.pi])
        self._gait_freq = 1.0
        self._phase_dt = 2 * np.pi * self._gait_freq * ctrl_dt
        
        # Global command
        self.command = np.array([0.3, 0.0, 0.0])
        
    # Receive low-state from C++ handle, all ik has been solved in the C++ script
    def low_state_handle(self, channel, data):
        msg = low_state_t.decode(data)
        self.low_state_msg = msg
        #print("Received message on channel \"%s\"" % channel)
        #print("   rpy = %s" % str(msg.rpy))
        #print("   gyro = %s" % str(msg.gyro))
        #print("   acc = %s" % str(msg.acc))
        #print("\n")
        
    def lcm_loop(self):
        # Non-blocking reading all lcm subs
        rfds, _, _ = select.select([self.lc.fileno()], [], [], 0)
        if rfds:
            self.lc.handle()
    
    # Get hardware observation from sensors
    def get_obs(self):
        gyro = self.imu.gyro
        acc = self.imu.acc
        # Check axis alignment of this with hector
        imu_xmat_obj = Rotation.from_euler('yxz', self.imu.rpy, degrees=False)
        imu_xmat = imu_xmat_obj.as_matrix()
        gravity = imu_xmat.T @ np.array([0, 0, -1])
        
        q = self.joint_state.q - self._default_q
        dq = self.joint_state.dq
        
        phase = np.concatenate([np.cos(self._phase), np.sin(self._phase)])
        
        obs_n = np.hstack([
            gyro,                   #3
            acc,                    #3
            gravity,                #3
            q,                      #18
            dq,                     #18
            self._last_action,      #18
            phase,                  #4
            self.command            #3
        ])
        
        obs = np.hstack([
            obs_n, 
            self._last_obs,
            self._last_last_obs
        ])

        # Update obs history
        self._last_last_obs = self._last_obs
        self._last_obs = obs_n
        
        return obs.astype(np.float32)
    
        
    def get_ctrl(self):
        # First update internal states
        self.update_state()
        
        # Get obs
        obs = self.get_obs()
        onnx_input = {"obs": obs.reshape(1, -1)}
        act_pred = self._policy.run(self._output_names, onnx_input)[0][0]

        # Update action buffer
        self._last_action = act_pred.copy()
        
        # Get target joint
        q_tar = act_pred*self._act_scale + self._default_q
        
        print(q_tar)
      
        # Set initial zero position, debug
        self.joint_act.q = np.zeros(self._nu)
        
        #self.joint_act.q = q_tar

        
    def update_state(self):
        # Update state variable from received low-level states msg to internal buffer
        # Copy over imu states, history, cehck hector's rad/axis limit
        self.imu.rpy = np.array(self.low_state_msg.rpy) / 180.0* np.pi
        self.imu.gyro = np.array(self.low_state_msg.gyroscope)
        self.imu.acc = np.array(self.low_state_msg.accelerometer) 
        # Low_state to joint state
        self.joint_state.q = np.array(self.low_state_msg.q)
        self.joint_state.dq = np.array(self.low_state_msg.dq)
        self.joint_state.tau_est = np.array(self.low_state_msg.tau_est)


    def send_low_cmd(self):
        # Send low cmd to C++ wrapper
        # Also check joint limit
        self.low_cmd_msg.q = self.joint_act.q
        self.low_cmd_msg.dq = np.zeros(self._nu, dtype=np.float32)
        self.low_cmd_msg.tau = np.zeros(self._nu, dtype=np.float32)
        self.low_cmd_msg.kp = self.joint_act.kp
        self.low_cmd_msg.kd = self.joint_act.kd
        
        self.low_cmd_msg.enable[:] = self.joint_act.enable
        self.low_cmd_msg.calibrate[:] = self.joint_act.calibrate
        # TODO: Joint limit safety check
        
        # Send to lcm
        self.lc.publish(self._chn_low_command, self.low_cmd_msg.encode())
        
    
if __name__ == "__main__":
    
    # Init the hw wrapper
    default_q = np.array([0.6, 0.12, 0.9, -0.6, -0.12, -0.9])
    onnx_path = "onnx/hector_policy.onnx"
    dt = 0.02
    act_scale = 0.5
    kp = 5.0
    kd = 0.1
    
    hector_hw = hw_wrapper(default_q, onnx_path, dt, act_scale, kp, kd)
    
    # Add periodic tasks
    # Low_state loop, running 500hz
    low_state_loop = loop_func(name="low_state", period=0.002, cb=hector_hw.lcm_loop)
    # Low_cmd loop, running 500hz
    low_cmd_loop = loop_func(name="low_cmd", period=0.002, cb=hector_hw.send_low_cmd)
    # Main control loop, 50hz
    main_ctrl_loop = loop_func(name="main_ctrl", period=0.02, cb=hector_hw.get_ctrl)
    

    try:
        main_ctrl_loop.start()
        low_state_loop.start()
        low_cmd_loop.start()
        print("Running all control loops. Press Ctrl+C to exit early.")
        
        # Init mujoco visualizer for debug
        mjcf_path = "xmls/hector_v2.xml"
        model = mujoco.MjModel.from_xml_path(mjcf_path)
        data = mujoco.MjData(model)
        
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                # Get the latest joint state from the hardware wrapper
                hw_q = hector_hw.joint_state.q
                hw_rpy = hector_hw.imu.rpy
                
                rot: Rotation = Rotation.from_euler('yxz', hw_rpy, degrees=False)
                qx, qy, qz, qw = rot.as_quat()
                quat_wxyz = (qw, qx, qy, qz)
                
                data.qpos[0:3] = np.array([0.0, 0.0, 0.0])
                data.qpos[3:7] = quat_wxyz #np.array([1.0, 0.0, 0.0, 0.0])
                data.qpos[7:] = hw_q
                mujoco.mj_step(model, data)
                viewer.sync()
                
                # Sleep to not overwhelm the CPU
                time.sleep(0.02) 
                
    except KeyboardInterrupt:
        print("\nCtrl+C received.")
    finally:
        low_cmd_loop.shutdown()
        low_state_loop.shutdown()
        main_ctrl_loop.shutdown()
        
    print("\nExample script finished.")