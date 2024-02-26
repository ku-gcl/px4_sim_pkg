import px4_sim_pkg.DMD as DMD
import unittest
import numpy as np


class Test(unittest.TestCase):
    def setUp(self):
        self.dmd = DMD.DMD()
        # self.y = np.random.rand(6, 10)
        # self.u = np.random.rand(3, 10)
        self.y = np.arange(1, 61).reshape(6, 10)
        self.u = np.arange(1, 31).reshape(3, 10)

    def test_array(self):
        imu_data = []
        for i in range(0, 2):
            # imu = list(range(1, 7))
            imu = [x * (i + 1) for x in range(1, 7)]
            imu_data.append(imu)
        imu_data_np = np.array(imu_data).T
        print("shape of imu_data")
        print(imu_data_np.shape)
        print(imu_data_np)
        
    # def test_concatenate(self):
    #     data = self.dmd.concatenate(self.y, self.u)
    #     data_ans = np.concatenate([self.y, self.u], axis=0)
    #     # dataとdata_ansが近似的に等しいかどうかを検証
    #     self.assertTrue(np.allclose(data, data_ans), "The concatenated arrays are not approximately equal")

    #     print(data_ans.shape)

    # def test_predict(self):
    #     data = self.dmd.concatenate(self.y, self.u)
    #     self.dmd.splitdata(data, stateDim=6, aug=1)
    #     self.A = self.dmd.DMD()
    #     print(self.A.shape)

    #     X = np.random.rand(9, 1)
    #     x_k1 = self.dmd.predictstate(X)
    #     print(x_k1)

    # def test_predict2(self):
    #     data = self.dmd.concatenate(self.y, self.u)
    #     self.dmd.splitdata(data, stateDim=6, aug=1)
    #     self.A = self.dmd.DMD()

    #     print("----------------------------")
    #     print("----------------------------")
    #     print(data)
    #     print("----------------------------")
    #     print("----------------------------")
    #     print(data[0:6, 1])
    #     print("----------------------------")
    #     print("----------------------------")
    #     X = data[:, 0]
    #     x_k1 = self.dmd.predictstate(X)
    #     print(x_k1)
    #     self.assertTrue(np.allclose(data[0:6, 1], x_k1), "The concatenated arrays are not approximately equal")


if __name__ == '__main__':
    unittest.main()
