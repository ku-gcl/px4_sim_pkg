from datetime import datetime
import os
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray  # 予測された状態をパブリッシュするために使用
import px4_sim_pkg.MavrosNode as MavrosNode
import px4_sim_pkg.Trajectory as Trajectory
import px4_sim_pkg.DMD as DMD


rospy.init_node('dmd_training_node', anonymous=True)

# pred_state_pub = rospy.Publisher('dmd/predict_state', Float64MultiArray, queue_size=10)
data_pub = rospy.Publisher('dmd/data', Float64MultiArray, queue_size=10)
dmd_info_pub = rospy.Publisher('dmd/info', Float64MultiArray, queue_size=10)
dmd_A_pub = rospy.Publisher('dmd/A', Float64MultiArray, queue_size=10)


rate = rospy.Rate(10.0)
mav = MavrosNode.MavrosNode()
trajectory = Trajectory.Trajectory()
altitude = 1.25
radius = 1.0
w = 0.63
duration = 30.0
rate_pred = rospy.Rate(25.0)

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
x, y, z = trajectory.circleXY(time_sec=0, radius=radius, w=w, altitude=altitude)
mav.set_destination(x=x, y=y, z=z)
mav.pub_local_position()
rospy.sleep(5)


# データ格納の配列を用意
imu_data = []
rcout_data = []
force_and_torque = []


# メインループ
start_time = rospy.Time.now()


rospy.loginfo("Sending setpoint and collecting data...")
while (not rospy.is_shutdown() 
        and (rospy.Time.now() - start_time) < rospy.Duration(duration)):
    
    time_sec = (rospy.Time.now() - start_time).to_sec()

    x, y, z = trajectory.circleXY(time_sec=time_sec, radius=radius, w=w, altitude=altitude)
    mav.set_destination(x, y, z)
    mav.pub_local_position()  
    
    # data collecting
    imu_data.append(mav.imu)
    rcout_data.append(mav.rcout_norm)
    force_and_torque.append(mav.force_and_torque)
    
    data_array = mav.imu + mav.rcout_norm + mav.force_and_torque
    data_msg = Float64MultiArray(data=data_array)
    data_pub.publish(data_msg)

    rate_pred.sleep()

x, y, z = trajectory.circleXZ(time_sec=0, radius=radius, w=w, altitude=altitude)
mav.set_destination(x=x, y=y, z=z)
mav.pub_local_position()
rospy.sleep(2)
start_time = rospy.Time.now()

while (not rospy.is_shutdown() 
        and (rospy.Time.now() - start_time) < rospy.Duration(duration)):
    
    time_sec = (rospy.Time.now() - start_time).to_sec()

    x, y, z = trajectory.circleXZ(time_sec=time_sec, radius=radius, w=w, altitude=altitude)
    mav.set_destination(x, y, z)
    mav.pub_local_position()  
    
    # data collecting
    imu_data.append(mav.imu)
    rcout_data.append(mav.rcout_norm)
    force_and_torque.append(mav.force_and_torque)
    
    data_array = mav.imu + mav.rcout_norm + mav.force_and_torque
    data_msg = Float64MultiArray(data=data_array)
    data_pub.publish(data_msg)

    rate_pred.sleep()


# hovering for calculation
rospy.loginfo("Hover for DMD calculation")
rospy.sleep(2.0)
x, y, z = trajectory.hover(time_sec, x=0.0, y=0.0, altitude=altitude)
mav.set_destination(x, y, z)
mav.pub_local_position()
rospy.sleep(2.0)


# preprocessing and DMD implementation
rospy.loginfo("DMD calculation start")
start_dmd_cal = rospy.Time.now()
# list to nparray
y = np.array(imu_data).T
u = np.array(force_and_torque).T
u = u[1:4, :]      # extract torque only
stateDim, _ = y.shape
inputDim, _ = u.shape
aug = 1 

dmd = DMD.DMD()
Y = dmd.concatenate(y, u)
dmd.splitdata(XX=Y, stateDim=stateDim, aug=aug)
A = dmd.DMD()

# DMD情報のpublish
finish_dmd_cal = rospy.Time.now()
calc_time = (finish_dmd_cal - start_dmd_cal).to_sec()
dmd_info = [calc_time, len(imu_data), len(force_and_torque)]
dmd_info_msg = Float64MultiArray()
dmd_info_msg.data = dmd_info
dmd_info_pub.publish(dmd_info_msg)

# A行列のpublishとファイル保存
dmd_A_msg = Float64MultiArray(data=A.flatten())
dmd_A_pub.publish(dmd_A_msg)


# A行列を現在の日時を含むファイル名で保存
home_directory = os.path.expanduser('~')
current_date_str = datetime.now().strftime("%Y-%m-%d")
data_directory = os.path.join(home_directory, 'rosbag', current_date_str)
# dataディレクトリが存在しない場合は作成
if not os.path.exists(data_directory):
    os.makedirs(data_directory)
current_time_str = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
filename = os.path.join(data_directory, f"A_matrix_{current_time_str}.csv")
np.savetxt(filename, A, delimiter=',')
rospy.loginfo(f"A matrix saved to {filename}")

rospy.loginfo(A)

rospy.sleep(5)


# 着陸
rospy.loginfo("Sending setpoint ...")
for i in range(10):
    mav.set_destination(x=0, y=0, z=altitude)
    mav.pub_local_position()
    rate.sleep()
rospy.loginfo("Done sending setpoint ...")
rospy.sleep(2.0)
mav.send_land_command()

