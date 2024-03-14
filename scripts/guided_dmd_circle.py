
import rospy
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
altitude = 1.0
radius = 1.0
duration = 20.0
duration_dmd = 60
rate_ctrl = rospy.Rate(10)

# -------------------------------------------------------------
# FCUの接続を待つ
rospy.loginfo("Waiting for connection...")
while not rospy.is_shutdown() and not mav.current_state.connected:
    rospy.sleep(0.1)

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
# mav.set_drone_to_guided_mode_auto()

# 機体をアーム
mav.arm_vehicle()

# 離陸
mav.vehicle_takeoff(altitude)

# -------------------------------------------------------------
# 初期位置に移動
rospy.sleep(5)
mav.set_heading(0)
x, y, z = trajectory.circle(time_sec=0, radius=radius, altitude=altitude)
mav.set_destination(x=x, y=y, z=z)
# mav.set_local_position(x=x, y=y, z=z)
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

    x, y, z = trajectory.circle(time_sec=time_sec, radius=radius, altitude=altitude)
    mav.set_destination(x, y, z)
    # mav.set_local_position(x, y, z)
    mav.pub_local_position()  
    
    # data collecting
    imu_data.append(mav.imu)
    rcout_data.append(mav.rcout_norm)
    force_and_torque.append(mav.force_and_torque)

    rate_ctrl.sleep()


# hovering for calculation
rospy.loginfo("Hover for DMD calculation")
rospy.sleep(2.0)
x, y, z = trajectory.hover(time_sec, x=1.0, y=0.0, altitude=altitude)
mav.set_destination(x, y, z)
# mav.set_local_position(x, y, z)
mav.pub_local_position()
rospy.sleep(2.0)


# preprocessing and DMD implementation
rospy.loginfo("DMD calculation start")
start_dmd_cal = rospy.Time.now()
# TODO: add data preprocessing
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
finish_dmd_cal = rospy.Time.now()
rospy.loginfo("DMD calculation end, time: %5.3f", (finish_dmd_cal-start_dmd_cal).to_sec())
rospy.sleep(5)

rospy.loginfo(A)


# flight with prediction
start_time = rospy.Time.now()

rospy.loginfo("Start circle trajectory with DMD")
while (not rospy.is_shutdown() 
        and (rospy.Time.now() - start_time) < rospy.Duration(duration_dmd)):
    
    start = rospy.Time.now()
    
    time_sec = (rospy.Time.now() - start_time).to_sec()

    x, y, z = trajectory.circle(time_sec=time_sec, radius=radius, altitude=altitude)
    mav.set_destination(x, y, z)
    # mav.set_local_position(x=x, y=y, z=z)
    mav.pub_local_position()
    
    # mav.imu, mav.rcout_norm, mav.force_and_torqueをNumPy配列に変換
    imu_np = np.array(mav.imu)
    force_and_torque_np = np.array(mav.force_and_torque)[1:4]  # [1:4]で特定の要素を抽出
    # 1行のベクトルに変換
    data_vector = np.concatenate([imu_np, force_and_torque_np])
    # DMDによる状態予測
    x_k1 = dmd.predictstate(data_vector)  # reshape(-1, 1)で2次元配列に変換

    # 予測された状態x_k1を/dmd/predict_stateとしてpublish
    pred_msg = Float64MultiArray()
    pred_msg.data = x_k1.flatten()  # flatten()で1次元配列に変換
    pred_state_pub.publish(pred_msg)

    rate_ctrl.sleep()


# 着陸
rospy.loginfo("Sending setpoint ...")
for i in range(10):
    mav.set_destination(x, y, z)
    # mav.set_local_position(x=0, y=0, z=altitude)
    mav.pub_local_position()
    rate.sleep()
rospy.loginfo("Done sending setpoint ...")
mav.send_land_command()

