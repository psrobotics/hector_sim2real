import numpy as np
from imumaster.math.Quaternion import Quaternion
from typing import Tuple

"""
Extended Kalman Filter
======================
In this module, we will use the Extended Kalman Filter to compute the attitude in quaternion form data 
from an inertial sensor that integrates a gyroscope, accelerometer(and magnetometer).
"""


class EKF_AHRS:
    """
    Extended Kalman Filter to estimate orientation as Quaternion.

    Parameters
    ----------
    sample_rate : float, optional
        The sampling rate of sensor data in Hz. Default is 200.0 Hz.

    frame : str, optional
        The coordinate frame used for orientation estimation.
        Options are 'ENU' (East-North-Up) and 'NED' (North-East-Down) and 'NWU' (North-West-Up).
        Default is 'NED'.

    **kwargs
        Additional keyword arguments:

        - sample_dt : float, optional
            The time interval between samples in seconds. Default is calculated as 1.0 / sample_rate.
        - q0 : list or numpy.ndarray, optional
            Initial quaternion state. Default is [1, 0, 0, 0].
        - P : numpy.ndarray, optional
            Initial state covariance matrix. Default is 0.001 times the identity matrix.

        Noise Parameters:
        - Q_noise : numpy.ndarray, optional
            Process noise covariance matrix. Default is 0.1 times the identity matrix.
        - accel_noise : numpy.ndarray, optional
            Accelerometer noise covariance matrix. Default is 3.8 times the identity matrix.
        - mag_noise : numpy.ndarray, optional
            Magnetometer noise covariance matrix. Default is 0.5 times the identity matrix.

    Attributes
    ----------
    sample_rate : float
        The sampling rate of sensor data in Hz.

    frame : str
        The coordinate frame used for orientation estimation.

    dt : float
        The time interval between samples in seconds.

    x_k_minus_1 : numpy.ndarray
        Initial prior estimate of the state variables as a quaternion.

    P_k_minus_1 : numpy.ndarray
        Initial state covariance matrix.

    P_k_prior : numpy.ndarray
        Prior estimate covariance matrix.

    x_k_prior : numpy.ndarray
        Prior estimate of the state variables.

    x_k_posterior : numpy.ndarray
        Posterior estimate of the state variables.

    P_k_posterior : numpy.ndarray
        Posterior estimate covariance matrix.

    Q_noise : numpy.ndarray
        Process noise covariance matrix.

    accel_noise : numpy.ndarray
        Accelerometer noise covariance matrix.

    mag_noise : numpy.ndarray
        Magnetometer noise covariance matrix.

    a_ref : numpy.ndarray
        Reference vector representing the gravity acceleration based on the chosen coordinate frame.

    """

    def __init__(self,
                 sample_rate: float = 200.0,
                 frame: str = 'NED',
                 **kwargs):
        self.sample_rate = sample_rate
        self.frame = frame  # Local tangent plane coordinate frame
        self.dt = kwargs.get('sample_dt', 1.0 / self.sample_rate)

        # Initial values of state variables
        self.x_k_minus_1 = np.array(kwargs.get('q0', [1, 0, 0, 0])).reshape(-1, 1)
        self.P_k_minus_1 = kwargs.get('P', 0.001 * np.identity(4))  # Initial state covariance
        # Prior Estimate
        self.P_k_prior = np.identity(4)
        self.x_k_prior = np.array([1, 0, 0, 0])
        # Posterior Estimate
        self.x_k_posterior = np.array([1, 0, 0, 0])
        self.P_k_posterior = np.identity(4)

        # Process Noise Covariance Matrix
        self.Q_noise = kwargs.get('Q_noise', 0.1 * np.eye(4))

        # Observation Noise Covariance Matrix
        self.accel_noise = kwargs.get('accel_noise', 3.8 * np.eye(3))
        self.mag_noise = kwargs.get('mag_noise', 0.5 * np.eye(3))

        if frame.upper() == 'NED':
            self.a_ref = np.array([0.0, 0.0, -1.0]).reshape(-1, 1)
        elif frame.upper() == 'ENU' or frame.upper() == 'NWU':
            self.a_ref = np.array([0.0, 0.0, 1.0]).reshape(-1, 1)

    def Omega(self, gyro: np.ndarray) -> np.ndarray:
        """
        This operator is constantly used at different steps of the EKF.

        Parameters
        ----------
        gyro : numpy.ndarray
            Sample of tri-axial Gyroscope in rad/s.

        Returns
        -------
        Omega : numpy.ndarray
            Omega matrix.
        """
        wx, wy, wz = gyro
        Omega = np.array([
            [0, -wx, -wy, -wz],
            [wx, 0, wz, -wy],
            [wy, -wz, 0, wx],
            [wz, wy, -wx, 0]
        ])

        return Omega

    def Jacobian_predicted(self, gyro: np.ndarray) -> np.ndarray:
        """
        Jacobian of linearized predicted state.

        Parameters
        ----------
        gyro : numpy.ndarray
            Angular velocity in rad/s.

        Returns
        -------
        Jacobian_predicted : numpy.ndarray
            Jacobian of state.
        """
        Omega_t = self.Omega(gyro)
        Jacobian_predicted = np.identity(4) + 0.5 * self.dt * Omega_t
        return Jacobian_predicted

    def State_Transition_Model(self, q: np.ndarray, gyro: np.ndarray) -> np.ndarray:
        """
        Linearized function of Process Model (Prediction)

        Parameters
        ----------
        q : numpy.ndarray
            A-priori quaternion.
        gyro : numpy.ndarray
            Angular velocity, in rad/s.

        Returns
        -------
        update_result : numpy.ndarray
            Linearized estimated quaternion in **Prediction** step.
        """
        Omega_t = self.Omega(gyro)
        update_result = (np.identity(4) + 0.5 * self.dt * Omega_t) @ q
        update_result = Quaternion.normalize(update_result)
        return update_result

    def ekf_update(self, accel: np.ndarray, gyro: np.ndarray, mag: np.ndarray = None) -> np.ndarray:
        """
        Estimate the quaternions given sensor data.

        Attributes ``gyr``, ``acc`` MUST contain data. Attribute ``mag`` is optional.

        Parameters
        ----------
        accel : numpy.ndarray
            Sample of tri-axial Accelerometer in m/s^2.

        gyro : numpy.ndarray
            Sample of tri-axial Gyroscope in rad/s.

        mag : numpy.ndarray
            Sample of tri-axial Magnetometer in uT.

        Returns
        -------
        Q : numpy.ndarray
             Array with posterior estimation results (estimated quaternion).

        """
        # Measureing Data
        gyro = np.array(gyro, dtype=np.float64)
        accel = np.array(accel, dtype=np.float64).reshape(-1, 1)
        accel = accel / np.linalg.norm(accel)
        # ----- Prediction -----
        self.x_k_prior = self.State_Transition_Model(self.x_k_minus_1, gyro)
        A_k = self.Jacobian_predicted(gyro)
        self.P_k_prior = A_k @ self.P_k_minus_1 @ A_k.T + self.Q_noise
        # ----- Correction -----
        h1, H1 = self.Jacobian_H1(self.x_k_prior)
        Kk1 = self.P_k_prior @ H1.T @ np.linalg.inv(H1 @ self.P_k_prior @ H1.T + self.accel_noise)
        state1 = Kk1 @ (accel - h1)

        if mag is None:
            self.x_k_posterior = self.x_k_prior + state1
            self.P_k_posterior = (np.identity(4) - Kk1 @ H1) @ self.P_k_prior
        else:
            state1[3][0] = 0  # No effect on yaw angle. Set q3 to zero.
            self.x_k_posterior = self.x_k_prior + state1
            self.P_k_posterior = (np.identity(4) - Kk1 @ H1) @ self.P_k_prior

            mag = np.array(mag, dtype=np.float64).reshape(-1, 1)
            mag = mag / np.linalg.norm(mag)
            h2, H2 = self.Jacobian_H2(mag, self.x_k_prior)
            Kk2 = self.P_k_prior @ H2.T @ np.linalg.inv(H2 @ self.P_k_prior @ H2.T + self.mag_noise)
            state2 = Kk2 @ (mag - h2)
            state2[1][0] = 0  # No effect on roll. Set q1 to zero.
            state2[2][0] = 0  # No effect on pitch. Set q2 to zero.
            self.x_k_posterior = self.x_k_posterior + state2
            self.P_k_posterior = (np.identity(4) - Kk2 @ H2) @ self.P_k_posterior
            self.x_k_posterior = Quaternion.normalize(self.x_k_posterior)
        # ----- update params -----
        self.x_k_minus_1 = self.x_k_posterior
        self.P_k_minus_1 = self.P_k_posterior

        return self.x_k_posterior

    def Jacobian_H1(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Calculate the predicted observation vector and Jacobian matrix for the accelerometer measurement.

        Parameters
        ----------
        q : numpy.ndarray
            Quaternion representing the orientation.As a-priori-value.

        Returns
        -------
        Tuple[numpy.ndarray, numpy.ndarray]
            Predicted observation vector and Jacobian matrix corresponding for the accelerometer measurement.
        """

        h1 = Quaternion.ground_to_sensor(self.a_ref, q)
        q0, q1, q2, q3 = q.squeeze()
        if self.frame.upper() == 'NED':
            H1 = np.array([
                [-2 * q2, 2 * q3, -2 * q0, 2 * q1],
                [2 * q1, 2 * q0, 2 * q3, 2 * q2],
                [2 * q0, -2 * q1, -2 * q2, 2 * q3],
            ])
            H1 = -H1
        elif self.frame.upper() == 'ENU' or self.frame.upper() == 'NWU':
            H1 = np.array([
                [-2 * q2, 2 * q3, -2 * q0, 2 * q1],
                [2 * q1, 2 * q0, 2 * q3, 2 * q2],
                [2 * q0, -2 * q1, -2 * q2, 2 * q3],
            ])

        return h1, H1

    def Jacobian_H2(self, mag: np.ndarray, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Calculate the predicted observation vector and Jacobian matrix for the magnetometer  measurement.

        Parameters
        ----------
        mag : numpy.ndarray
            Sample of tri-axial Magnetometer in uT.
        q : numpy.ndarray
            Quaternion representing the orientation.As a-priori-value.

        Returns
        -------
        Tuple[numpy.ndarray, numpy.ndarray]
            Predicted observation vector and Jacobian matrix corresponding for the magnetometer measurement.
        """
        q0, q1, q2, q3 = q.squeeze()
        if self.frame.upper() == 'NED' or self.frame.upper() == 'NWU':
            h_ground = Quaternion.sensor_to_ground(mag, q)
            b_ground = np.array([
                [np.sqrt(h_ground[0][0] ** 2 + h_ground[1][0] ** 2)],
                [0],
                [h_ground[2][0]],
            ])
            bx, by, bz = b_ground.squeeze()
            h2 = Quaternion.ground_to_sensor(b_ground, q)
            H2 = np.array([
                [2 * q0 * bx - 2 * q2 * bz, 2 * q1 * bx + 2 * q3 * bz, -2 * q2 * bx - 2 * q0 * bz,
                 -2 * q3 * bx + 2 * q1 * bz],
                [-2 * q3 * bx + 2 * q1 * bz, 2 * q2 * bx + 2 * q0 * bz, 2 * q1 * bx + 2 * q3 * bz,
                 -2 * q0 * bx + 2 * q2 * bz],
                [2 * q2 * bx + 2 * q0 * bz, 2 * q3 * bx - 2 * q1 * bz, 2 * q0 * bx - 2 * q2 * bz,
                 2 * q1 * bx + 2 * q3 * bz]
            ])
        elif self.frame.upper() == 'ENU':
            h_ground = Quaternion.sensor_to_ground(mag, q)
            b_ground = np.array([
                [0],
                [np.sqrt(h_ground[0][0] ** 2 + h_ground[1][0] ** 2)],
                [h_ground[2][0]],
            ])
            bx, by, bz = b_ground.squeeze()
            h2 = Quaternion.ground_to_sensor(b_ground, q)
            H2 = np.array([
                [2 * by * q3 - 2 * bz * q2, 2 * by * q2 + 2 * bz * q3, -4 * bx * q2 + 2 * by * q1 - 2 * bz * q0,
                 -4 * bx * q3 + 2 * by * q0 + 2 * bz * q1],
                [-2 * bx * q3 + 2 * bz * q1, 2 * bx * q2 - 4 * by * q1 + 2 * bz * q0, 2 * bx * q1 + 2 * bz * q3,
                 -2 * bx * q0 - 4 * by * q3 + 2 * bz * q2],
                [2 * bx * q2 - 2 * by * q1, 2 * bx * q3 - 2 * by * q0 - 4 * bz * q1,
                 2 * bx * q0 + 2 * by * q3 - 4 * bz * q2, 2 * bx * q1 + 2 * by * q2]
            ])

        return h2, H2
