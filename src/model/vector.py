from math import atan2, sqrt, pow, cos, sin


class Vector:
    """
    Class that represents vectors with 3 float numbers.
    Implements useful functions to use these objects
    """

    def __init__(self, x, y, z):
        """
        Initialization of the Vector class given 3 floats
        :param x: x position
        :type x: float
        :param y: y position
        :type y: float
        :param z: z postion
        :type z: float
        """
        self._x = x
        self._y = y
        self._z = z

    def __eq__(self, other):
        """
        Returns whether the Vector object and another are equal or not
        :param other: the other vector for comparison
        :type other: Vector
        """
        return isinstance(other, Vector) and self.x == other.x and self.y == other.y and self.z == other.z

    def __str__(self):
        """
        Computes a string giving information about the vector
        """
        return "Vector<[{},{},{}]>".format(self.x, self.y, self.z)

    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, value):
        self._x = value

    @property
    def y(self):
        return self._y

    @y.setter
    def y(self, value):
        self._y = value

    @property
    def z(self):
        return self._z

    @z.setter
    def z(self, value):
        self._z = value

    @staticmethod
    def from_dict(vec_dict):
        """
        Returns a new Vector instance from the dict representation used in the path files
        """
        return Vector(vec_dict['X'], vec_dict['Y'], vec_dict['Z'])

    @staticmethod
    def x_forward():
        """
        Returns the x-axis unit vector
        """
        return Vector(1.0, 0.0, 0.0)

    def get_angle(self, vec):
        """
        Returns the angle between self and another Vector
        :param vec: the other Vector
        :type vec: Vector
        """
        return atan2(vec.x - self.x, vec.y - self.y)

    def distance_to(self, vec):
        """
        Returns the angle between self and another Vector
        :param vec: the other Vector
        :type vec: Vector
        """
        return sqrt(pow(vec.x - self.x, 2) + pow(vec.y - self.y, 2) + pow(vec.z - self.z, 2))
