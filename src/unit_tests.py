import unittest

from lokarriaexample import qmult, conjugate, rotate, heading
from controller import PathLoader


def are_vect_dict_equal(quat, quat_dict):
    return quat.x == quat_dict['X'] and quat.y == quat_dict['Y'] and quat.z == quat_dict['Z']

def are_quat_dict_equal(quat, quat_dict):
    return quat.x == quat_dict['X'] and quat.y == quat_dict['Y'] and quat.z == quat_dict['Z'] and quat.w == quat_dict['W']

class TestMathsModule(unittest.TestCase):
    p = PathLoader('paths/Path-around-table-and-back.json')
    vect_dicts = p.positionPath(dict=True)
    vects = p.positionPath()
    quat_dicts = p.orientationPath(dict=True)
    quats = p.orientationPath()

    def test_loading(self):
        for i in range(len(self.quats)):
            self.assertTrue(are_quat_dict_equal(self.quats[i], self.quat_dicts[i]))
        for i in range(len(self.vects)):
            self.assertTrue(are_vect_dict_equal(self.vects[i], self.vect_dicts[i]))

    def test_conjugation(self):
        self.assertTrue(are_quat_dict_equal(self.quats[0].conjugate(), conjugate(self.quat_dicts[0])))

    def test_multplication(self):
        self.assertTrue(are_quat_dict_equal(self.quats[0] * self.quats[1], qmult(self.quat_dicts[0], self.quat_dicts[1])))

    def test_rotation(self):
        self.assertTrue(are_vect_dict_equal(self.quats[0].rotate(self.vects[0]), rotate(self.quat_dicts[0], self.vect_dicts[0])))

    def test_heading(self):
        self.assertTrue(are_vect_dict_equal(self.quats[0].heading(), heading(self.quat_dicts[0])))

if __name__ == '__main__':
    unittest.main()
