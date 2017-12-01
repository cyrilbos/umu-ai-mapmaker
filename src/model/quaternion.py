import numpy

from .vector import Vector


class Quaternion:
    """
    Class that represents quaternions with 4 float numbers.
    Implements useful functions such as heading, rotations...
    """
    __w = 0

    def __init__(self, w, vector):
        """
        Initialization of the Quaternion class given a float and a Vector
        :param w: value of w to assign
        :type w: float
        :param vector: unit vector to assign
        :type vector: Vector
        """
        if not isinstance(vector, Vector):
            raise (TypeError('Parameter vector of init is not a vector'))
        self.__unit_vector = vector
        self.__w = w

    def __mul__(self, other):
        """
        multiplies the Quaternion with another, in that order
        :param other: the other quaternion for operation
        :type other: Quaternion
        """
        if isinstance(other, Quaternion):
            return Quaternion(self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
                              Vector(self.__w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
                                     self.__w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
                                     self.__w * other.z + self.x * other.y - self.y * other.x + self.z * other.w))
        else:
            raise TypeError('trying to multiply a quaternion by something else than a quaternion')

    def __str__(self):
        """
        Computes a string giving information about the quaternion
        """
        return "Quaternion<{}, [{},{},{}]>".format(self.w, self.x, self.y, self.z)

    @property
    def unit_vector(self):
        return self.__unit_vector

    @property
    def x(self):
        return self.__unit_vector.x

    @property
    def y(self):
        return self.__unit_vector.y

    @property
    def z(self):
        return self.__unit_vector.z

    @property
    def w(self):
        return self.__w

    @staticmethod
    def from_dict(dict):
        """
            Returns a new instance of Quaternion from the dict representation used in the path files.
        """
        return Quaternion(dict['W'], Vector(dict['X'], dict['Y'], dict['Z']))

    def normalize(self):
        """
        Returns the norm of quaternion
        """
        return (self.__w + self.x + self.y + self.z) / \
               numpy.sqrt(self.__w * self.__w + self.x * self.x + self.y * self.y + self.z * self.z)

    def conjugate(self):
        """
        Returns the conjugate of quaternion
        """
        return Quaternion(self.w, Vector(-self.x, -self.y, -self.z))

    def heading(self):
        """
        Returns the heading of this quaternion from the X-axis
        """
        return self.rotate(Vector.x_forward())

    def rotate(self, v):
        """
        Returns the vector v rotated by the quaternion in a new Vector instance
        :param v: vector to rotate
        :type v: Vector
        """
        rotated = (self * Quaternion(0, Vector(v.x, v.y, v.z))) * self.conjugate()
        return Vector(rotated.x, rotated.y, rotated.z)


