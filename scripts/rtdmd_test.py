from datetime import datetime
import glob
import os
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray  # 予測された状態をパブリッシュするために使用
import px4_sim_pkg.MavrosNode as MavrosNode
import px4_sim_pkg.Trajectory as Trajectory
import px4_sim_pkg.DMD as DMD
import px4_sim_pkg.RTDMD as RTDMD


rospy.init_node("rtdmd_node", anonymous=True)

dmd_pred_state_pub = rospy.Publisher('dmd/predict_state', Float64MultiArray, queue_size=10)
dmd_data_pub = rospy.Publisher('dmd/data', Float64MultiArray, queue_size=10)
dmd_info_pub = rospy.Publisher('dmd/info', Float64MultiArray, queue_size=10)
dmd_A_pub = rospy.Publisher('dmd/A', Float64MultiArray, queue_size=10)

rtdmd_pred_state_pub = rospy.Publisher('rtdmd/predict_state', Float64MultiArray, queue_size=10)
rtdmd_data_pub = rospy.Publisher('rtdmd/data', Float64MultiArray, queue_size=10)
rtdmd_info_pub = rospy.Publisher('rtdmd/info', Float64MultiArray, queue_size=10)
rtdmd_A_pub = rospy.Publisher('rtdmd/A', Float64MultiArray, queue_size=10)


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
x, y, z = trajectory.circleXY(time_sec=0, radius=radius, altitude=altitude, w=w)
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

    x, y, z = trajectory.circleXY(time_sec=time_sec, radius=radius, altitude=altitude, w=w)
    mav.set_destination(x, y, z)
    mav.pub_local_position()  
    
    # data collecting
    imu_data.append(mav.imu)
    rcout_data.append(mav.rcout_norm)
    force_and_torque.append(mav.force_and_torque)
    
    data_array = mav.imu + mav.rcout_norm + mav.force_and_torque
    data_msg = Float64MultiArray(data=data_array)
    dmd_data_pub.publish(data_msg)

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

# RTDMD parameter
delta = 10**(-3)
lam = 0.97
aug = 2*stateDim+1
rtdmd = RTDMD.RTDMD(delta, lam, aug, stateDim, inputDim)

# DMD calculation
dmd = DMD.DMD()
train_data_delay_y = rtdmd.create_delay_coordinates(y)
train_data_delay_u = rtdmd.create_delay_coordinates(u)
Y = dmd.concatenate(train_data_delay_y, train_data_delay_u)
dmd.splitdata(XX=Y, stateDim=stateDim, aug=aug)
A = dmd.DMD()
_, nTime = y.shape
# xPredict = rtdmd.predict_state(A, dmd.X, nTime)
# Error = rtdmd.calculate_fit_error(train_data_y, xPredict)

# DMD情報のpublish
finish_dmd_cal = rospy.Time.now()
calc_time = (finish_dmd_cal - start_dmd_cal).to_sec()
dmd_info = [calc_time, len(imu_data), len(force_and_torque), "DMD"]
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


##########################
# RTDMDメインループ
start_time = rospy.Time.now()


# RTDMD 初期化
rtdmd.rtdmdResult["AB"] = A
prevData = np.zeros(rtdmd.augStateInputDim)
newData = np.zeros(rtdmd.augStateInputDim)

# ログ記録用
dmdlog = []
# 予測値格納用の配列初期化
xPredictV = []

imu_data_rtdmd = []
rcout_data_rtdmd = []
force_and_torque_rtdmd = []

i = 0
rospy.loginfo("Sending setpoint and construct model using RTDMD...")
while (not rospy.is_shutdown() 
        and (rospy.Time.now() - start_time) < rospy.Duration(duration)):

    loop_start_time = rospy.Time.now()
    time_sec = (rospy.Time.now() - start_time).to_sec()

    x, y, z = trajectory.circleXY(time_sec=time_sec, radius=radius, altitude=altitude, w=w)
    mav.set_destination(x, y, z)
    mav.pub_local_position()
    
    # data collecting
    imu_data_rtdmd.append(mav.imu)
    rcout_data_rtdmd.append(mav.rcout_norm)
    force_and_torque_rtdmd.append(mav.force_and_torque)
    
    data_array = mav.imu + mav.rcout_norm + mav.force_and_torque
    data_msg = Float64MultiArray(data=data_array)
    rtdmd_data_pub.publish(data_msg)

    x_data = np.array(imu_data_rtdmd).T
    u_temp = np.array(force_and_torque_rtdmd).T
    u_data = u[1:4, :]

    if i < aug:
        newData[i*stateDim:(i+1)*stateDim] = x_data
        u_start = stateDim * aug
        newData[u_start+i*inputDim:u_start+(i+1)*inputDim] = u_data
        continue

    # store prevData
    prevData = newData.copy()

    ## 遅延座標の作成
    # decompose data to state and input
    aug_state_data = prevData[:aug*stateDim].copy()
    aug_input_data = prevData[aug*stateDim:].copy()

    # shift
    aug_state_data[:(aug-1)*stateDim] = aug_state_data[stateDim:]
    aug_state_data[(aug-1)*stateDim:] = x_data

    aug_input_data[:(aug-1)*inputDim] = aug_input_data[inputDim:]
    aug_input_data[(aug-1)*inputDim:] = u_data

    # concatenate aug_state_data and aug_input_data
    newData = np.concatenate([aug_state_data, aug_input_data], axis=0)

    ## 更新
    start_rtdmd_cal = rospy.Time.now()
    rtdmd.update_rtdmd(prevData, newData, i+1)
    finish_rtdmd_cal = rospy.Time.now()
    # RTDMDによる状態予測
    x_k1 = rtdmd.rtdmdResult['y_hat']
    xPredictV.append(x_k1)

    dmdlog.append(rtdmd.rtdmdResult)
    
    # RTDMD情報のpublish
    loop_finish_time = rospy.Time.now()
    calc_time = (finish_rtdmd_cal - start_rtdmd_cal).to_sec()
    loop_time = (loop_finish_time - loop_start_time).to_sec()
    rtdmd_info = [calc_time, len(imu_data), len(force_and_torque), "RTDMD", loop_time]
    rtdmd_info_msg = Float64MultiArray()
    rtdmd_info_msg.data = rtdmd_info
    rtdmd_info_pub.publish(rtdmd_info_msg)
    # AB行列のpublish
    rtdmd_A_msg = Float64MultiArray(data=rtdmd.rtdmdResult.AB.flatten())
    rtdmd_A_pub.publish(rtdmd_A_msg)

    # 予測された状態x_k1を/rtdmd/predict_stateとしてpublish
    rtdmd_pred_msg = Float64MultiArray()
    rtdmd_pred_msg.data = x_k1.flatten()  # flatten()で1次元配列に変換
    rtdmd_pred_state_pub.publish(rtdmd_pred_msg)

    i = i + 1
    rate_pred.sleep()


y_rtdmd = np.array(imu_data_rtdmd).T
u_rtdmd = np.array(force_and_torque_rtdmd).T
u_rtdmd = u_rtdmd[1:4, :]      # extract torque only

# xPredictV2リストをNumPy配列に変換
xPredictV = np.array(xPredictV).T
# 予測前の1~aug列までは0埋めしておく
xPredictV = np.concatenate((np.zeros((stateDim, aug)), xPredictV[:, 0:]), axis=1)
# 誤差計算
ErrorV = rtdmd.calculate_fit_error(y_rtdmd[:, aug:], xPredictV[:, aug:])
print(ErrorV)






# 着陸
rospy.loginfo("Sending setpoint ...")
for i in range(10):
    mav.set_destination(x=0, y=0, z=altitude)
    mav.pub_local_position()
    rate.sleep()
rospy.loginfo("Done sending setpoint ...")
rospy.sleep(2.0)
mav.send_land_command()

