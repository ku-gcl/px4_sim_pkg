import rospy
import px4_sim_pkg.MavrosNode as MavrosNode
import unittest


class RCOut:
    # RCOutメッセージを模擬するためのクラス
    def __init__(self, channels):
        self.channels = channels


class Test(unittest.TestCase):
    def setUp(self):
        self.mav = MavrosNode.MavrosNode()

    def test_rcout_normalize_0(self):
        rcout_message = RCOut([1500, 1500, 1500, 1500])
             
        # rcout_cbメソッドを呼び出し
        self.mav.rcout_cb(rcout_message)
        
        # self.rcout_normalizedがすべて0になっているか確認
        for value in self.mav.rcout_norm:
            self.assertEqual(value, 0)
        print(self.mav.rcout)
        print(self.mav.force_and_torque)
    
    def test_rcout_normalize_m1(self):
        rcout_message = RCOut([1000, 1000, 1000, 1000])
        self.mav.rcout_cb(rcout_message)
        for value in self.mav.rcout_norm:
            self.assertEqual(value, -1)
        print(self.mav.rcout)
        print(self.mav.force_and_torque)

    def test_rcout_normalize_p1(self):
        rcout_message = RCOut([2000, 2000, 2000, 2000])
        self.mav.rcout_cb(rcout_message)
        for value in self.mav.rcout_norm:
            self.assertEqual(value, 1)
        print(self.mav.rcout)
        print(self.mav.force_and_torque)
    
    def test_rcout(self):
        rcout_message = RCOut([1200, 1500, 2000, 1000])
        self.mav.rcout_cb(rcout_message)

        c1, c2, c3, c4 = self.mav.rcout_normalize(rcout_message)
        thrust =  (c1 + c2 + c3 + c4)/4
        roll   = (-c1 + c2 + c3 - c4)/4
        pitch  =  (c1 - c2 + c3 - c4)/4
        yaw    =  (c1 + c2 - c3 - c4)/4
        force_and_torque = [thrust, roll, pitch, yaw]

        for i, value in enumerate(self.mav.force_and_torque):
            self.assertEqual(value, force_and_torque[i])
        
        print(self.mav.rcout)
        print(self.mav.rcout_norm)
        print(self.mav.force_and_torque)

if __name__ == '__main__':
    unittest.main()
