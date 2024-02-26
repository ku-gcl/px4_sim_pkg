import px4_sim_pkg.DMD as DMD
import unittest
import numpy as np


class Test(unittest.TestCase):
    def setUp(self):
        self.dmd = DMD.DMD()
        self.y = np.random.rand(6, 10)
        self.u = np.random.rand(3, 10)
        
    def test_concatenate(self):
        data = self.dmd.concatenate(self.y, self.u)
        data_ans = np.concatenate([self.y, self.u], axis=0)
        # dataとdata_ansが近似的に等しいかどうかを検証
        self.assertTrue(np.allclose(data, data_ans), "The concatenated arrays are not approximately equal")



if __name__ == '__main__':
    unittest.main()
