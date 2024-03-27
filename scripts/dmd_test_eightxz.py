from datetime import datetime
import glob
import os
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray  # 予測された状態をパブリッシュするために使用
import px4_sim_pkg.MavrosNode as MavrosNode
import px4_sim_pkg.Trajectory as Trajectory
import px4_sim_pkg.DMD as DMD

rospy.init_node('dmd_test_node', anonymous=True)

pred_state_pub = rospy.Publisher('dmd/predict_state', Float64MultiArray, queue_size=10)
data_pub = rospy.Publisher('dmd/data', Float64MultiArray, queue_size=10)

date = datetime.now().strftime("%Y-%m-%d")
rate = rospy.Rate(10.0)
mav = MavrosNode.MavrosNode()
trajectory = Trajectory.Trajectory()
altitude = 1.25
ampx = 1.25
ampz = 1.0
w = 0.63
duration_dmd = 60
rate_pred = rospy.Rate(25.0)


# ホームディレクトリを展開
home_directory = os.path.expanduser('~')
pattern = os.path.join(home_directory, 'rosbag', date, 'A_matrix*.csv')
files = glob.glob(pattern)
latest_file = max(files, key=os.path.getmtime)      # 最新のファイルを見つける
A_loaded = np.loadtxt(latest_file, delimiter=',')

dmd = DMD.DMD()
dmd.A = A_loaded

# -------------------------------------------------------------
# FCUの接続を待つ
rospy.loginfo("Waiting for connection...")
while not rospy.is_shutdown() and not mav.current_state.connected:
    rate.sleep()

# allow the subscribers to initialize
rospy.sleep(5)

# グローバルポジションの原点を設定
rospy.loginfo("Set Global Position ...")
for i in range(10):
    mav.set_gp_position(latitude=33.595270, longitude=130.215496)
    rate.sleep()
rospy.loginfo("Done Global Position ...")

# GYM_OFFSETの計算
rospy.sleep(3)  # 現在のheadingを取得するための時間
mav.set_gym_offset()

# 開始地点をlocal座標で設定
rospy.loginfo("Sending setpoint ...")
for i in range(10):
    mav.set_destination(x=0, y=0, z=altitude)
    # mav.set_local_position(x=0, y=0, z=altitude)
    mav.pub_local_position()
    rate.sleep()
rospy.loginfo("Done sending setpoint ...")


# -------------------------------------------------------------
# GUIDEDモードに変更
mav.set_drone_to_guided_mode_manual()

# 機体をアーム
mav.arm_vehicle()

# 離陸
mav.vehicle_takeoff(altitude)

# -------------------------------------------------------------
# 初期位置に移動
rospy.sleep(5)
mav.set_heading(0)
x, y, z = trajectory.eightXZ(time_sec=0, ampx=ampx, ampz=ampz, altitude=altitude, w=w)
mav.set_destination(x=x, y=y, z=z)
mav.pub_local_position()
rospy.sleep(5)


# メインループ
start_time = rospy.Time.now()


# flight with prediction
start_time = rospy.Time.now()

rospy.loginfo("Start circle trajectory with DMD")
while (not rospy.is_shutdown() 
        and (rospy.Time.now() - start_time) < rospy.Duration(duration_dmd)):
    
    time_sec = (rospy.Time.now() - start_time).to_sec()

    x, y, z = trajectory.eightXZ(time_sec=time_sec, ampx=ampx, ampz=ampz, altitude=altitude, w=w)
    mav.set_destination(x, y, z)
    mav.pub_local_position()
    
    imu_ = mav.imu
    rcout_ = mav.rcout_norm
    force_and_torque_ = mav.force_and_torque
    
    # mav.imu, mav.rcout_norm, mav.force_and_torqueをNumPy配列に変換
    imu_np = np.array(imu_)
    force_and_torque_np = np.array(force_and_torque_)[1:4]  # [1:4]で特定の要素を抽出
    # 1行のベクトルに変換
    data_vector = np.concatenate([imu_np, force_and_torque_np])
    # DMDによる状態予測
    x_k1 = dmd.predictstate(data_vector)  # reshape(-1, 1)で2次元配列に変換
    
    data_array = imu_ + rcout_ + force_and_torque_
    data_msg = Float64MultiArray(data=data_array)
    data_pub.publish(data_msg)

    # 予測された状態x_k1を/dmd/predict_stateとしてpublish
    pred_msg = Float64MultiArray()
    pred_msg.data = x_k1.flatten()  # flatten()で1次元配列に変換
    pred_state_pub.publish(pred_msg)

    rate_pred.sleep()


# 着陸
rospy.loginfo("Sending setpoint ...")
for i in range(10):
    mav.set_destination(x=0, y=0, z=altitude)
    mav.pub_local_position()
    rate.sleep()
rospy.loginfo("Done sending setpoint ...")
rospy.sleep(2.0)
mav.send_land_command()

