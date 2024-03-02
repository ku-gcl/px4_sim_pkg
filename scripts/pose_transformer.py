#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

def callback(data):
    transformed_pose = PoseStamped()

    # ヘッダーをコピー
    transformed_pose.header = data.header

    # 座標変換を適用
    transformed_pose.pose.position.x = -data.pose.position.y
    transformed_pose.pose.position.y = data.pose.position.x
    transformed_pose.pose.position.z = data.pose.position.z

    transformed_pose.pose.orientation.x = -data.pose.orientation.y
    transformed_pose.pose.orientation.y = data.pose.orientation.x
    transformed_pose.pose.orientation.z = data.pose.orientation.z
    transformed_pose.pose.orientation.w = data.pose.orientation.w

    # 変換後のポーズをパブリッシュ
    pub.publish(transformed_pose)

if __name__ == '__main__':
    rospy.init_node('pose_transformer_node', anonymous=True)
    pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
    rospy.Subscriber('/mocap_node/Robot_1/pose', PoseStamped, callback)
    rospy.spin()
