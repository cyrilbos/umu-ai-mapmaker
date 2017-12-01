import json
from model import Vector, Quaternion


class PathLoader:
    """
    Class that loads the path files into lists of Vector used by our program or dictionaries used by the unit tests.
    """

    def __init__(self, filename):
        # Load the path from a file and convert it into a list of coordinates
        self.loadPath(filename)
        self.vecPath = self.positionPath(dict=True)

    def loadPath(self, file_name):
        with open(file_name) as path_file:
            data = json.load(path_file)

        self.path = data

    def positionPath(self, dict=False):
        """
        Parses the positions in the loaded file into a list of either Vector instances or dictionaries depending on the
        value of the dict parameter.
        :param dict: if set to True, will parse into dictionaries instead of into Vector instances
        :type dict: bool
        """
        if dict:
            return [{'X': p['Pose']['Position']['X'],
                     'Y': p['Pose']['Position']['Y'],
                     'Z': p['Pose']['Position']['Z']}
                    for p in self.path]
        else:
            return [Vector.from_dict(p['Pose']['Position'])
                    for p in self.path]

    def orientationPath(self, dict=False):
        """
        Parses the orientations in the loaded file into a list of either Quaternion instances or dictionaries depending on the
        value of the dict parameter.
        :param dict: if set to True, will parse into dictionaries instead of into Quaternion instances
        :type dict: bool
        """
        if dict:
            return [{'W': p['Pose']['Orientation']['W'],
                     'X': p['Pose']['Orientation']['X'],
                     'Y': p['Pose']['Orientation']['Y'],
                     'Z': p['Pose']['Orientation']['Z']} for p in self.path]
        else:
            return [Quaternion(p['Pose']['Orientation']['W'],
                               Vector(p['Pose']['Orientation']['X'],
                                      p['Pose']['Orientation']['Y'],
                                      p['Pose']['Orientation']['Z']))
                    for p in self.path]
