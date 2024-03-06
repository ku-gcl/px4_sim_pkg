import rospy
import px4_sim_pkg.MavrosNode as MavrosNode
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPointStamped

def main():
    # ノードの初期化後の処理や変数の設定
    rospy.init_node('offb_node', anonymous=True)
    mavros_node = MavrosNode.MavrosNode()
    rate = rospy.Rate(10.0)    
    MODE = "hovering"  # 飛行モードの選択: circle, updown, eight, hovering

    # FCUの接続を待つ
    while not mavros_node.current_state.connected:
        rospy.sleep(0.1)
        
    rospy.loginfo("Waiting for connection...")

    # グローバルポジションの原点を設定
    rospy.loginfo("Set Global Position ...")
    latitude = 33.595270
    longitude = 130.215496
    for i in range(100):
        mavros_node.set_gp_position(latitude, longitude)
        rate.sleep()
    rospy.loginfo("Done Global Position ...")

    # GYM_OFFSETの計算
    rospy.sleep(3)  # 現在のheadingを取得するための時間
    GYM_OFFSET = 0
    total_heading = 0
    for i in range(1, 31):
        rate.sleep()  # 0.1秒待機
        total_heading += mavros_node.current_heading.data
        rospy.loginfo("current heading%d: %f", i, total_heading / i)
    GYM_OFFSET = total_heading / 30
    rospy.loginfo(f"the N' axis is facing: {GYM_OFFSET}")

    # 開始地点をlocal座標で設定
    start_pose = mavros_node.set_destination(0, 0, 0.5, GYM_OFFSET)
    mavros_node.set_local_position(start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z)

    # GUIDEDモードに変更
    mavros_node.set_drone_to_guided_mode_auto()

    # 機体をアーム
    mavros_node.arm_vehicle()

    # 離陸
    mavros_node.vehicle_takeoff(0.5)

    rospy.sleep(5)  # 安定を待つ
    mavros_node.set_heading(0)

    # 特定の期間ホバリングを実行
    while not rospy.is_shutdown() and (rospy.Time.now() - start_time) < rospy.Duration(10.0):
        rate.sleep()  # 0.1秒ごとにループ
        if MODE == "hovering":
            rospy.loginfo("HOVERING")
            HEIGHT = 0.5
            hovering_pose = mavros_node.set_destination(0, 0, HEIGHT, GYM_OFFSET)
            mavros_node.set_local_position(hovering_pose.pose.position.x, hovering_pose.pose.position.y, hovering_pose.pose.position.z)
    rospy.loginfo("end hovering")

    # 着陸
    mavros_node.send_land_command()

if __name__ == '__main__':
    main()
