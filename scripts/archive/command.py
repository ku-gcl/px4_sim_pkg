import rospy
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from geometry_msgs.msg import PoseStamped



# SetMode サービスのクライアントをグローバルで定義（適切に初期化する必要があります）
set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
land_client = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)


def set_local_position(x, y, z):
    """
    ローカル座標で指定された位置にドローンを移動させる
    """
    pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    rospy.sleep(1)  # パブリッシャーがセットアップされるのを待つ

    pose = PoseStamped()
    pose.header = Header()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z


def set_drone_to_guided_mode():
    """
    ドローンをGUIDEDモードに設定する関数。
    """
    try:
        # GUIDEDモードに設定するリクエストを送信
        response = set_mode_client(custom_mode="GUIDED")
        if response.mode_sent:
            rospy.loginfo("Guided mode enabled")
        else:
            rospy.logwarn("Guided mode not enabled, but no exception was thrown.")
    except rospy.ServiceException as e:
        rospy.logerr("Set mode failed: %s" % e)


def arm_vehicle():
    """
    機体をアームする関数。
    """
    try:
        # Trueを指定して機体をアーム
        response = arming_client(True)
        if response.success:
            rospy.loginfo("Vehicle armed")
        else:
            rospy.logwarn("Vehicle not armed, but no exception was thrown.")
    except rospy.ServiceException as e:
        rospy.logerr("Arming failed: %s" % e)


def vehicle_takeoff(altitude):
    """
    機体を指定した高度に離陸させる関数。

    Parameters:
    altitude (float): 離陸後の高度（メートル）
    """
    try:
        response = takeoff_client(altitude=altitude)
        if response.success:
            rospy.loginfo("Takeoff sent")
        else:
            rospy.logwarn("Takeoff not sent, but no exception was thrown.")
    except rospy.ServiceException as e:
        rospy.logerr("Takeoff failed: %s" % e)


def send_land_command():
    """
    MAVROSを使用して着陸コマンドを送信する関数。
    """
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        # 着陸コマンドの送信
        # response = land_client(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
        response = land_client()
        if response.success:
            rospy.loginfo("Land sent successfully")
        else:
            rospy.logwarn("Land command sent, but not successful")
    except rospy.ServiceException as e:
        rospy.logerr("Landing failed: %s" % e)
