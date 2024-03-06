
import rospy
import math
import numpy as np
import px4_sim_pkg.MavrosNode as MavrosNode
import px4_sim_pkg.Trajectory as Trajectory
import px4_sim_pkg.DMD as DMD
from std_msgs.msg import Float64MultiArray  # 予測された状態をパブリッシュするために使用

rospy.init_node('offb_node', anonymous=True)
mav = MavrosNode.MavrosNode()
rate = rospy.Rate(20.0)
trajectory = Trajectory.Trajectory()
altitude = 0.5

pred_state_pub = rospy.Publisher('dmd/predict_state', Float64MultiArray, queue_size=10)


rospy.loginfo("Waiting for connection...")
# FCU接続を待つ
while not rospy.is_shutdown() and not mav.current_state.connected:
    rospy.loginfo("current_state.connected: %s" % (mav.current_state.connected))
    rospy.sleep(0.1)

# グローバルポジションの原点を設定
rospy.loginfo("Set Global Position ...")
for i in range(10):
    mav.set_gp_position(latitude=33.595270, longitude=130.215496)
    rospy.sleep(0.1)
rospy.loginfo("Done Global Position ...")

# GYM_OFFSETの計算
rospy.sleep(3)  # 現在のheadingを取得するための時間
mav.set_gym_offset()

# セットポイントを送信
rospy.loginfo("Sending setpoint ...")
for i in range(100):
    # mav.set_destination(x=0, y=0, z=0.5)
    mav.set_local_position(x=0, y=0, z=altitude)
    mav.pub_local_position()
    rate.sleep()
rospy.loginfo("Done sending setpoint ...")

# GUIDEDモードに設定
# mav.set_drone_to_guided_mode_auto()
mav.set_drone_to_guided_mode_manual()

# 機体のアーム
mav.arm_vehicle()

# 離陸
mav.vehicle_takeoff(altitude)

# rospy.sleep(5)  # 安定を待つ
# mav.set_heading(0)

rospy.sleep(3)
x, y, z = trajectory.hover(0, x=0, y=0, altitude=altitude)
mav.set_local_position(x=x, y=y, z=z)
rospy.sleep(3)



# 着陸
rospy.loginfo("Sending setpoint ...")
for i in range(100):
    x, y, z = trajectory.hover(time_sec=0, x=0, y=0, altitude=altitude)
    mav.set_local_position(x=0, y=0, z=altitude)
    mav.pub_local_position()
    rate.sleep()
rospy.loginfo("Done sending setpoint ...")


mav.send_land_command()

