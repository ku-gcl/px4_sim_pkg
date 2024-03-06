import rospy
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State, RCOut
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion


class MavrosNode():
    def __init__(self):
        self.current_state = State()
        self.current_rcout = RCOut()
        self.current_pose = PoseStamped()
        self.current_heading = Float64()
        self.geo = GeoPointStamped()
        self.pose = PoseStamped()
        
        self.state_sub = rospy.Subscriber("mavros/state", State, self.state_cb)     # arming events
        self.rcout_sub = rospy.Subscriber("mavros/rc/out", RCOut, self.rcout_cb)
        self.imu_sub = rospy.Subscriber("mavros/imu/data", Imu, self.imu_cb)
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.set_gp_origin_pub = rospy.Publisher("mavros/global_position/set_gp_origin", GeoPointStamped, queue_size=10)
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
        self.takeoff_client = rospy.ServiceProxy("mavros/cmd/takeoff", CommandTOL)
        self.land_client = rospy.ServiceProxy("mavros/cmd/land", CommandTOL)
        
        self.imu = [0, 0, 0, 0, 0, 0]
        self.rcout = [0, 0, 0, 0]
        self.rcout_norm = [0, 0, 0, 0]
        self.force_and_torque = [0, 0, 0, 0]
        self.GYM_OFFSET = 0


    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)

        qw = cy * cr * cp + sy * sr * sp
        qx = cy * sr * cp - sy * cr * sp
        qy = cy * cr * sp + sy * sr * cp
        qz = sy * cr * cp - cy * sr * sp

        # return {'w': qw, 'x': qx, 'y': qy, 'z': qz}
    
    def state_cb(self, msg):
        self.current_state = msg
        # rospy.loginfo(f"Connected: {msg.connected}, Armed: {msg.armed}")
        
    def rcout_normalize(self, rcout):
        # f = lambda c: 1/500*c-3, f(1000)
        C1, C2, C3, C4 = rcout.channels[0], rcout.channels[1], rcout.channels[2], rcout.channels[3]
        c1 = 1/500*C1-3
        c2 = 1/500*C2-3
        c3 = 1/500*C3-3
        c4 = 1/500*C4-3
        return c1, c2, c3, c4
        
    def rcout_cb(self, msg):
        rcout = msg
        # normalize from -1 to 1
        c1, c2, c3, c4 = self.rcout_normalize(rcout)
        thrust =  (c1 + c2 + c3 + c4)/4
        roll   = (-c1 + c2 + c3 - c4)/4
        pitch  =  (c1 - c2 + c3 - c4)/4
        yaw    =  (c1 + c2 - c3 - c4)/4
        
        self.rcout = [rcout.channels[0], rcout.channels[1], rcout.channels[2], rcout.channels[3]]
        self.rcout_norm = [c1, c2, c3, c4]
        self.force_and_torque = [thrust, roll, pitch, yaw]
        

    def imu_cb(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        # rospy.loginfo("R: %5.2f, P: %5.2f, Y: %5.2f" % (roll, pitch, yaw))
        
        # 角速度
        angvel = msg.angular_velocity
        
        # 姿勢角と角速度をimuに格納
        self.imu = [roll, pitch, yaw, angvel.x, angvel.y, angvel.z]
    
    
    def pose_cb(self, msg):
        self.current_pose = msg
        rospy.loginfo(f"x: {msg.pose.position.x}, y: {msg.pose.position.y}, z: {msg.pose.position.z}")

    def heading_cb(self, msg):
        self.current_heading = msg
        rospy.loginfo(f"Current heading: {msg.data}")
    
    
    def set_gym_offset(self):
        rate = rospy.Rate(10.0)
        total_heading = 0
        for i in range(1, 31):
            rate.sleep()  # 0.1秒待機
            total_heading += self.current_heading.data
            rospy.loginfo("current heading%d: %f", i, total_heading / i)
        self.GYM_OFFSET = total_heading / 30
        rospy.loginfo(f"the N' axis is facing: {self.GYM_OFFSET}")
    
    
    def set_gp_position(self, latitude, longitude):
        self.geo.position.latitude = latitude
        self.geo.position.longitude = longitude
        self.set_gp_origin_pub.publish(self.geo)
    
    
    # destination
    def set_heading(self, heading):
        heading = -heading + 90 - self.GYM_OFFSET
        yaw = math.radians(heading)
        self.pose.pose.orientation = self.quaternion_from_euler(0, 0, yaw)

    def set_destination(self, x, y, z):
        deg2rad = math.pi / 180
        X = x * math.cos(-self.GYM_OFFSET * deg2rad) - y * math.sin(-self.GYM_OFFSET * deg2rad)
        Y = x * math.sin(-self.GYM_OFFSET * deg2rad) + y * math.cos(-self.GYM_OFFSET * deg2rad)
        Z = z
        self.pose.pose.position.x = X
        self.pose.pose.position.y = Y
        self.pose.pose.position.z = Z
        rospy.loginfo(f"Destination set to x: {X}, y: {Y}, z: {Z}")
    
    
    def set_local_position(self, x, y, z):
        """
        ローカル座標で指定された位置にドローンを移動させる
        """

        self.pose.pose.position.x = x
        self.pose.pose.position.y = y
        self.pose.pose.position.z = z
    
    def pub_local_position(self):
        self.local_pos_pub.publish(self.pose)


    def set_drone_to_guided_mode_auto(self):
        """
        ドローンをGUIDEDモードに自動で変更する関数。
        """
        try:
            # GUIDEDモードに設定するリクエストを送信
            response = self.set_mode_client(custom_mode="GUIDED")
            if response.mode_sent:
                rospy.loginfo("Guided mode enabled")
            else:
                rospy.logwarn("Guided mode not enabled, but no exception was thrown.")
        except rospy.ServiceException as e:
            rospy.logerr("Set mode failed: %s" % e)


    def set_drone_to_guided_mode_manual(self):
        """
        ドローンをGUIDEDモードに手動で設定する関数。
        """
        rospy.loginfo("Switched to GUIDED Mode")

        while self.current_state.mode != "GUIDED":
            rospy.sleep(0.1)


    def arm_vehicle(self):
        """
        機体をアームする関数。
        """
        try:
            # Trueを指定して機体をアーム
            response = self.arming_client(True)
            if response.success:
                rospy.loginfo("Vehicle armed")
            else:
                rospy.logwarn("Vehicle not armed, but no exception was thrown.")
        except rospy.ServiceException as e:
            rospy.logerr("Arming failed: %s" % e)


    def vehicle_takeoff(self, altitude):
        """
        機体を指定した高度に離陸させる関数。

        Parameters:
        altitude (float): 離陸後の高度（メートル）
        """
        try:
            response = self.takeoff_client(altitude=altitude)
            if response.success:
                rospy.loginfo("Takeoff sent")
            else:
                rospy.logwarn("Takeoff not sent, but no exception was thrown.")
        except rospy.ServiceException as e:
            rospy.logerr("Takeoff failed: %s" % e)


    def send_land_command(self):
        """
        MAVROSを使用して着陸コマンドを送信する関数。
        """
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            # 着陸コマンドの送信
            # response = land_client(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
            response = self.land_client()
            if response.success:
                rospy.loginfo("Land sent successfully")
            else:
                rospy.logwarn("Land command sent, but not successful")
        except rospy.ServiceException as e:
            rospy.logerr("Landing failed: %s" % e)


# if __name__ == '__main__':
#     rospy.init_node('offb_node', anonymous=True)

#     rate = rospy.Rate(20.0)
#     mav = MavrosNode()
#     rospy.loginfo("Initializing ...")
#     # FCU接続を待つ
#     while not rospy.is_shutdown() and not mav.current_state.connected:
#         rospy.loginfo("current_state.connected: %s" % (mav.current_state.connected))
#         rospy.sleep(0.1)
        
#     rospy.loginfo("Waiting for connection...")

#     # グローバルポジションの原点を設定
#     rospy.loginfo("Set Global Position ...")
#     latitude = 33.595270
#     longitude = 130.215496
#     mav.set_gp_position(latitude, longitude)


#     # セットポイントを送信
#     rospy.loginfo("Sending setpoint ...")
#     for i in range(100):
#         mav.set_local_position(1, 0, 1.5)
#         rate.sleep()
#     rospy.loginfo("Done sending setpoint ...")

#     # GUIDEDモードに設定
#     mav.set_drone_to_guided_mode()

#     # 機体のアーム
#     mav.arm_vehicle()

#     # 離陸
#     mav.vehicle_takeoff(1.5)

#     rospy.sleep(5)


#     # メインループ
#     start_time = rospy.Time.now()
#     while not rospy.is_shutdown() and (rospy.Time.now() - start_time) < rospy.Duration(120.0):
#         time_sec = (rospy.Time.now() - start_time).to_sec()
#         x = 1.0 * math.cos(time_sec)
#         y = 1.0 * math.sin(time_sec)
#         z = 1.5
#         mav.set_local_position(x, y, z)
#         rospy.sleep(0.1)

#     # 着陸
#     mav.send_land_command()

