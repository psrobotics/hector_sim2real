import numpy as np
from .EKF import EKF_AHRS


class Orientation:
    """
    Class for estimating orientation using different sensor fusion algorithms like Extended Kalman Filter (EKF).

    Parameters
    ----------
    sample_rate : float
        Sampling rate in Hz. Default is 200 Hz.

    frame : str
        Coordinate frame used for orientation estimation. Default is 'NED' (North-East-Down).

    method : str
        Sensor fusion algorithm method. Default is 'EKF' (Extended Kalman Filter).

    **kwargs : dict
        Additional keyword arguments for specific sensor fusion algorithms.

    Attributes
    ----------
    sample_rate : float
        Sampling rate in Hz.

    dt : float
        Time interval between samples.

    frame : str
        Coordinate frame used for orientation estimation.

    method : str
        Sensor fusion algorithm method.

    ekf : EKF_AHRS
        Instance of the Extended Kalman Filter (EKF) AHRS class for orientation estimation.

    Examples
    --------
    >>> orientation = Orientation(sample_rate=100, frame='ENU', method='EKF')
    >>> accel_data = np.array([0.06, 0.03, 0.99])
    >>> gyro_data = np.array([0.1, 0.2, 0.3])
    >>> mag_data = np.array([0.3, 0.1, 0.2])
    >>> q_estimation = orientation.EKF(accel_data, gyro_data, mag_data)
    >>> eulerangle = orientation.eulerangle(q_estimation)
    """
    def __init__(self,
                 sample_rate: float,
                 frame: str = 'NED',
                 method: str = 'EKF',
                 **kwargs):
        # Sample_rate and Sample_dt
        self.sample_rate = sample_rate
        self.dt = kwargs.get('sample_dt', 1.0 / self.sample_rate)
        # Method and coordinate frame.
        self.frame = frame  # Local tangent plane coordinate frame
        self.method = method  # Sensor Fusion Algorithm

        # EKF_methods
        EKF_methods = {'EKF', 'Ekf', 'ekf'}
        if self.method in EKF_methods:
            self.ekf = EKF_AHRS(sample_rate, frame, **kwargs)

    def EKF(self, accel: np.ndarray, gyro: np.ndarray, mag: np.ndarray = None, ) -> np.ndarray:
        """
        Extended Kalman Filter to estimate orientation as a quaternion.

        Parameters
        ----------
        accel : numpy.ndarray
            Sample of tri-axial Accelerometer in m/s^2.

        gyro : numpy.ndarray
            Sample of tri-axial Gyroscope in rad/s.

        mag : numpy.ndarray, optional
            Sample of tri-axial Magnetometer in uT.

        Returns
        -------
        q : numpy.ndarray
            Array with Extended Kalman Filter (EKF) estimation results as a quaternion.
        """
        q = self.ekf.ekf_update(accel, gyro, mag)
        return q

    def eulerangle(self, Quat: np.ndarray, scalar_first: bool = True, degrees: bool = True) -> np.ndarray:
        """
        Calculate the Euler angles from a quaternion.

        Parameters
        ----------
        Quat : numpy.ndarray
            The input quaternion. If scalar_first is True, the quaternion should be in the format [w, x, y, z]
            with the scalar part first. Otherwise, it should be in the format [x, y, z, w] with the scalar part last.

        scalar_first : bool, optional
            Determines the position of the scalar part in the quaternion. Default is True.

        degrees : bool, optional
            Determines whether the result should be in degrees or radians. Default is True (degrees).

        Returns
        -------
        eulerangle_result : numpy.ndarray
            Array with Euler angles [roll, pitch, yaw] in degrees or radians based on the degrees parameter.
        """
        if scalar_first:
            w, x, y, z = np.array(Quat).squeeze()
        else:
            x, y, z, w = np.array(Quat).squeeze()

        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x ** 2 + y ** 2))
        pitch = -np.arcsin(2 * (z * x - w * y))
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y ** 2 + z ** 2))

        if degrees:
            roll = np.degrees(roll)
            pitch = np.degrees(pitch)
            yaw = np.degrees(yaw)

        eulerangle_result = np.array([roll, pitch, yaw])

        return eulerangle_result
