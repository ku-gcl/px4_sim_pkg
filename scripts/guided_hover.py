
import rospy
import math
import numpy as np
import px4_sim_pkg.MavrosNode as MavrosNode
import px4_sim_pkg.Trajectory as Trajectory
import px4_sim_pkg.DMD as DMD
from std_msgs.msg import Float64MultiArray  # 予測された状態をパブリッシュするために使用

rospy.init_node('offb_node', anonymous=True)

pred_state_pub = rospy.Publisher('dmd/predict_state', Float64MultiArray, queue_size=10)


rate = rospy.Rate(20.0)
mav = MavrosNode.MavrosNode()
trajectory = Trajectory.Trajectory()


rospy.loginfo("Initializing ...")
# FCU接続を待つ
while not rospy.is_shutdown() and not mav.current_state.connected:
    rospy.loginfo("current_state.connected: %s" % (mav.current_state.connected))
    rospy.sleep(0.1)
    
rospy.loginfo("Waiting for connection...")

# グローバルポジションの原点を設定
rospy.loginfo("Set Global Position ...")
latitude = 33.595270
longitude = 130.215496
for i in range(100):
    mav.set_gp_position(latitude, longitude)
    rate.sleep()
rospy.loginfo("Done Global Position ...")

altitude = 0.5

# セットポイントを送信
rospy.loginfo("Sending setpoint ...")
for i in range(100):
    mav.set_local_position(0, 0, altitude)
    rate.sleep()
rospy.loginfo("Done sending setpoint ...")

# GUIDEDモードに設定
# mav.set_drone_to_guided_mode_auto()
mav.set_drone_to_guided_mode_manual()

# 機体のアーム
mav.arm_vehicle()

# 離陸
mav.vehicle_takeoff(altitude)

rospy.sleep(5)
x, y, z = trajectory.hover(0, x=0, y=0, altitude=altitude)
mav.set_local_position(x, y, z)
rospy.sleep(5)


# データ格納の配列を用意
imu_data = []
rcout_data = []
force_and_torque = []


# メインループ
start_time = rospy.Time.now()
duration = 10.0
rate_ctrl = rospy.Rate(10)

rospy.loginfo("Start circle trajectory")
while (not rospy.is_shutdown() 
        and (rospy.Time.now() - start_time) < rospy.Duration(duration)):
    
    time_sec = (rospy.Time.now() - start_time).to_sec()

    x, y, z = x, y, z = trajectory.hover(time_sec, x=0, y=0, altitude=altitude)
    mav.set_local_position(x, y, z)
    
    # data collecting
    imu_data.append(mav.imu)
    rcout_data.append(mav.rcout_norm)
    force_and_torque.append(mav.force_and_torque)

    rate_ctrl.sleep()
    # rospy.sleep(0.1)



# 着陸
rospy.loginfo("Sending setpoint ...")
for i in range(100):
    mav.set_local_position(0, 0, altitude)
    rate.sleep()
rospy.loginfo("Done sending setpoint ...")
mav.send_land_command()

