import numpy as np
from numbers import Complex, Real


class Quaternion:
    """
    A class representing a quaternion, which is a four-dimensional
    hypercomplex number used in 3D rotations and other applications.

    Attributes:
        w (float): The scalar (real) part of the quaternion.
        x (float): The first imaginary part of the quaternion.
        y (float): The second imaginary part of the quaternion.
        z (float): The third imaginary part of the quaternion.

    Properties:
        value (numpy.ndarray): Returns the quaternion as a numpy array of [w, x, y, z].
        conj (numpy.ndarray): Returns the conjugate of the quaternion as a numpy array.
        inv (numpy.ndarray): Returns the inverse of the quaternion as a numpy array.

    Methods:
        __init__: Initializes a Quaternion object.
    """

    def __init__(self, scalar_first: bool = True, *args, **kwargs):
        """
        Initializes a Quaternion object.

        Args:
            scalar_first (bool): A boolean flag indicating whether the scalar part
                                 is provided as the first argument. Default is True.
            *args: Variable length argument list. The arguments can be either:
                - A single list or numpy array containing four elements representing
                  the scalar, x, y, and z components of the quaternion respectively.
                - Four individual real numbers representing the scalar, x, y, and z
                  components of the quaternion respectively.

            **kwargs: Additional keyword arguments.


        Raises:
            TypeError: If the number of arguments provided exceeds four or if the
                       arguments are not of the correct type.
        """
        self.scalar_first = scalar_first
        if len(args) + len(kwargs) > 4:
            raise TypeError(
                '%s() takes at most 4 arguments, %d given' % (self.__class__.__name__, len(args) + len(kwargs)))
        # set defaults
        self.w = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        if args:
            # If only one argument is provided
            if len(args) == 1:
                if isinstance(args[0], (list, np.ndarray)) and len(args[0]) == 4:
                    if scalar_first:
                        self.w = float(args[0][0])
                        self.x = float(args[0][1])
                        self.y = float(args[0][2])
                        self.z = float(args[0][3])
                    else:
                        self.w = float(args[0][3])
                        self.x = float(args[0][0])
                        self.y = float(args[0][1])
                        self.z = float(args[0][2])

            # If four arguments are provided
            if len(args) == 4:
                if all(isinstance(arg, Real) for arg in args):
                    if scalar_first:
                        self.w = float(args[0])
                        self.x = float(args[1])
                        self.y = float(args[2])
                        self.z = float(args[3])
                    else:
                        self.x = float(args[0])
                        self.y = float(args[1])
                        self.z = float(args[2])
                        self.w = float(args[3])
                else:
                    raise TypeError('All arguments should be of type Real.')

    @property
    def value(self):
        """
        Returns the quaternion as a numpy array of [w, x, y, z] or [x, y, z, w].

        Returns:
            numpy.ndarray: An array representing the quaternion components [w, x, y, z] or [x, y, z, w].
        """
        if self.scalar_first:
            return np.array([self.w, self.x, self.y, self.z])
        else:
            return np.array([self.x, self.y, self.z, self.w])

    @property
    def conj(self):
        """
        Returns the conjugate of the quaternion.

        Returns:
            numpy.ndarray: The conjugate quaternion [w, -x, -y, -z]  or [-x, -y, -z, w].
        """
        if self.scalar_first:
            return np.array([self.w, -self.x, -self.y, -self.z])
        else:
            return np.array([-self.x, -self.y, -self.z, self.w])

    @property  # 四元数的逆
    def inv(self):
        """
        Returns the inverse of the quaternion.

        Returns:
            numpy.ndarray: The inverse quaternion.
        """
        Quat = np.array([self.w, self.x, self.y, self.z])
        conjugate = np.array([self.w, -self.x, -self.y, -self.z])
        norm = np.linalg.norm(Quat)

        if self.scalar_first:
            return conjugate / (norm*norm)
        else:
            return conjugate[::-1] / (norm*norm)

    # @property
    # def norm(self):
    #     return np.linalg.norm(np.array([self.w, self.x, self.y, self.z]))

    # @property
    # def normalize(self):
    #     return np.array([self.w, self.x, self.y, self.z])/np.linalg.norm(np.array([self.w, self.x, self.y, self.z]))

    # Quaternion Multiplication
    @staticmethod
    def multiply(Q1, Q2, scalar_first=True):
        if scalar_first:
            w1, x1, y1, z1 = Q1
            w2, x2, y2, z2 = Q2
            w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
            x = x1 * w2 + w1 * x2 - z1 * y2 + y1 * z2
            y = y1 * w2 + z1 * x2 + w1 * y2 - x1 * z2
            z = z1 * w2 - y1 * x2 + x1 * y2 + w1 * z2
            multiply_result = np.array([w, x, y, z])
        else:
            x1, y1, z1, w1 = Q1
            x2, y2, z2, w2 = Q2
            w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
            x = x1 * w2 + w1 * x2 - z1 * y2 + y1 * z2
            y = y1 * w2 + z1 * x2 + w1 * y2 - x1 * z2
            z = z1 * w2 - y1 * x2 + x1 * y2 + w1 * z2
            multiply_result = np.array([x, y, z, w])

        return multiply_result

    # Quaternion Addition
    @staticmethod
    def add(Q1, Q2):
        a1, b1, c1, d1 = Q1
        a2, b2, c2, d2 = Q2
        a = a1 + a2
        b = b1 + b2
        c = c1 + c2
        d = d1 + d2
        add_result = np.array([a, b, c, d])

        return add_result

    # Quaternion Subtraction
    @staticmethod
    def substract(Q1, Q2):
        a1, b1, c1, d1 = Q1
        a2, b2, c2, d2 = Q2
        a = a1 - a2
        b = b1 - b2
        c = c1 - c2
        d = d1 - d2
        substract_result = np.array([a, b, c, d])

        return substract_result

    # Quaternion Norm
    @staticmethod
    def norm(Quat):
        norm_result = np.linalg.norm(Quat)
        return norm_result

    # Quaternion Normalize
    @staticmethod
    def normalize(Quat):
        norm = np.linalg.norm(Quat)
        normalize_result = Quat / norm
        return normalize_result

    # Quaternion Conjugate
    @staticmethod
    def conjugate(Quat, scalar_first=True):
        if scalar_first:
            # Quat[w,x,y,z]
            w, x, y, z = np.array(Quat)
            conjugate_result = np.array([w, -x, -y, -z])

        else:
            # Quat[x,y,z,w]
            x, y, z, w = np.array(Quat)
            conjugate_result = np.array([-x, -y, -z, w])

        return conjugate_result

    # 四元数的逆-Quaternion Inverse
    @staticmethod
    def inverse(Quat, scalar_first=True):
        if scalar_first:
            # Quat[w,x,y,z]
            w, x, y, z = np.array(Quat)
            conjugate = np.array([w, -x, -y, -z])
            norm = np.linalg.norm(Quat)
            inverse_result = conjugate / (norm*norm)
        else:
            # Quat[x,y,z,w]
            x, y, z, w = np.array(Quat)
            conjugate = np.array([-x, -y, -z, w])
            norm = np.linalg.norm(Quat)
            inverse_result = conjugate / (norm*norm)

        return inverse_result

    @staticmethod
    def ground_to_sensor(ground_vector, Quat, scalar_first=True):
        if scalar_first:
            # Quat[w,x,y,z]
            w, x, y, z = np.array(Quat).squeeze()
        else:
            # Quat[x,y,z,w]
            x, y, z, w = np.array(Quat).squeeze()

        ground_to_sensor = np.array([
            [1 - 2 * y ** 2 - 2 * z ** 2, 2 * x * y + 2 * w * z, 2 * x * z - 2 * w * y],
            [2 * x * y - 2 * w * z, 1 - 2 * x ** 2 - 2 * z ** 2, 2 * y * z + 2 * w * x],
            [2 * x * z + 2 * w * y, 2 * y * z - 2 * w * x, 1 - 2 * x ** 2 - 2 * y ** 2]
        ])
        sensor_vector = np.dot(ground_to_sensor, ground_vector)
        return sensor_vector

    @staticmethod
    def sensor_to_ground(sensor_vector, Quat, scalar_first=True):
        if scalar_first:
            # Quat[w,x,y,z]
            w, x, y, z = np.array(Quat).squeeze()
        else:
            # Quat[x,y,z,w]
            x, y, z, w = np.array(Quat).squeeze()

        sensor_to_ground = np.array([
            [1 - 2 * y ** 2 - 2 * z ** 2, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y],
            [2 * x * y + 2 * w * z, 1 - 2 * x ** 2 - 2 * z ** 2, 2 * y * z - 2 * w * x],
            [2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, 1 - 2 * x ** 2 - 2 * y ** 2]
        ])

        ground_vector = np.dot(sensor_to_ground, sensor_vector)
        return ground_vector

    @staticmethod  # sensor_to_ground matrix
    def rotation_matrix(Quat, scalar_first=True):
        if scalar_first:
            # Quat[w,x,y,z]
            w, x, y, z = np.array(Quat).squeeze()
        else:
            # Quat[x,y,z,w]
            x, y, z, w = np.array(Quat).squeeze()
        R = np.zeros((3, 3))
        R[0, 0] = 1 - 2 * (y * y + z * z)
        R[0, 1] = 2 * (x * y - w * z)
        R[0, 2] = 2 * (x * z + w * y)

        R[1, 0] = 2 * (x * y + w * z)
        R[1, 1] = 1 - 2 * (x * x + z * z)
        R[1, 2] = 2 * (y * z - w * x)

        R[2, 0] = 2 * (x * z - w * y)
        R[2, 1] = 2 * (y * z + w * x)
        R[2, 2] = 1 - 2 * (x * x + y * y)

        return R

    @staticmethod
    def eulerangle(Quat, degrees: bool = True, scalar_first=True):
        if scalar_first:
            # Quat[w,x,y,z]
            w, x, y, z = np.array(Quat)
        else:
            # Quat[x,y,z,w]
            x, y, z, w = np.array(Quat)

        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x ** 2 + y ** 2))
        pitch = -np.arcsin(2 * (z * x - w * y))
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y ** 2 + z ** 2))
        if degrees:
            roll = np.degrees(roll)
            pitch = np.degrees(pitch)
            yaw = np.degrees(yaw)

        eulerangle_result = np.array([roll, pitch, yaw])

        return eulerangle_result
