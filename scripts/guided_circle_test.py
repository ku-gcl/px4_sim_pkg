import rospy
import px4_sim_pkg.MavrosNode as MavrosNode
import px4_sim_pkg.Trajectory as Trajectory
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPointStamped


# ノードの初期化後の処理や変数の設定
rospy.init_node('offb_node', anonymous=True)
mav = MavrosNode.MavrosNode()
trajectory = Trajectory.Trajectory()
rate = rospy.Rate(10.0)    
rate_ctrl = rospy.Rate(20)
MODE = "hovering"  # 飛行モードの選択: circle, updown, eight, hovering
altitude = 0.5
radius = 0.5
duration = 20.0

# FCUの接続を待つ
rospy.loginfo("Waiting for connection...")
while not mav.current_state.connected:
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
mav.set_destination(x=1, y=0, z=altitude)
mav.pub_local_position()

# GUIDEDモードに変更
mav.set_drone_to_guided_mode_manual()
# mav.set_drone_to_guided_mode_auto()

# 機体をアーム
mav.arm_vehicle()

# 離陸
mav.vehicle_takeoff(0.5)

rospy.sleep(5)  # 安定を待つ
mav.set_heading(0)

# 特定の期間ホバリングを実行
start_time = rospy.Time.now()

rospy.loginfo("Circle")
while not rospy.is_shutdown() and (rospy.Time.now() - start_time) < rospy.Duration(duration):
    time_sec = (rospy.Time.now() - start_time).to_sec()
    x, y, z = trajectory.circle(time_sec=time_sec, radius=radius, altitude=altitude)
    # mav.set_heading(0)
    mav.set_destination(x, y, z)
    mav.pub_local_position()        
    rate_ctrl.sleep()  # 0.1秒ごとにループ
rospy.loginfo("end hovering")

# 着陸
mav.send_land_command()