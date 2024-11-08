import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import math
from geometry_msgs.msg import Quaternion

from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped

from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
import time

class GoalAndMoveNode(Node):
    def __init__(self):
        super().__init__('goal_and_move_node')
        self.rectangle_marker_publisher = self.create_publisher(Marker, '/rectangle_marker', 2)
        self.circle_marker_publisher = self.create_publisher(Marker, '/circle_marker', 2)
        self.goal_marker_publisher = self.create_publisher(Marker, '/goal_marker', 2)
        timer_period = 0.5  # 0.5秒ごとに配信
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.rectangle_marker = Marker()
        self.circle_marker = Marker()
        self.goal_marker = Marker()

        self.amount_of_movement_x = 0.0
        self.amount_of_movement_y = 0.0

        self.local_costmap_sub = self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.local_costmap_callback, 2)
        self.local_costmap = None

        # costmap to map from odom
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        #self.costmap_point_to_map = PointStamped()
        self.point_in_map = PointStamped()
        self.costmap_point = PointStamped().point

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.stop_trigger = False

    def timer_callback(self):
        self.create_goal_marker()
        self.create_circle_marker()
        self.create_rectangle_marker()
        #self.set_goal_pose()
        #self.send_goal()
        self.amount_of_movement()
        if self.local_costmap is None:
            return
        else:
            self.check_collision_callback()
            self.set_goal_pose()
            self.send_goal()
        #self.get_logger().info("markers published")
        #self.get_logger().info(f'cos: {math.cos(self.calculation_radian() + math.radians(90))}')

    def local_costmap_callback(self, msg):
        self.local_costmap = msg
        
    def create_rectangle_marker(self):
       # self.rectangle_marker = Marker()
       self.rectangle_marker.header.frame_id = "map"  # 座標系を指定
       self.rectangle_marker.header.stamp = self.get_clock().now().to_msg()
       self.rectangle_marker.ns = "rectangle_namespace"     # 名前空間を設定
       self.rectangle_marker.id = 0                         # マーカーのID
       self.rectangle_marker.type = Marker.CUBE             # 長方形には CUBE タイプを使用
       self.rectangle_marker.action = Marker.ADD            # マーカーを表示
    
       # 位置と姿勢を設定
       self.rectangle_marker.pose.position.x = 1.0 - math.sin(self.calculation_radian() - math.radians(90)) * (self.rectangle_marker.scale.x / 2.0) + self.amount_of_movement_x 
       self.rectangle_marker.pose.position.y = 1.0 - math.cos(self.calculation_radian() + math.radians(90)) * (self.rectangle_marker.scale.x / 2.0) + self.amount_of_movement_y
       self.rectangle_marker.pose.position.z = 0.5

       # quaternion
       self.rectangle_marker.pose.orientation = self.calculation_quaternion()
    
       # サイズを設定 (長方形の幅、高さ、奥行き)
       self.rectangle_marker.scale.x = 0.5  # 長方形の長さ
       self.rectangle_marker.scale.y = 0.05  # 長方形の幅
       self.rectangle_marker.scale.z = 0.01  # 長方形の高さ

       # 色を設定 (RGBA)
       self.rectangle_marker.color.r = 0.0
       self.rectangle_marker.color.g = 1.0
       self.rectangle_marker.color.b = 0.0
       self.rectangle_marker.color.a = 0.8  # 不透明度

       # publish rectangle marker
       self.rectangle_marker_publisher.publish(self.rectangle_marker)

    def create_circle_marker(self):
       self.circle_marker.header.frame_id = "map"  # 座標系を指定
       self.circle_marker.header.stamp = self.get_clock().now().to_msg()
       self.circle_marker.ns = "circle_namespace"     # 名前空間を設定
       self.circle_marker.id = 0                         # マーカーのID
       self.circle_marker.type = Marker.CYLINDER             # 円には CYLINDER タイプを使用
       self.circle_marker.action = Marker.ADD            # マーカーを表示

       # 位置と姿勢を設定
       self.circle_marker.pose.position.x = 1.0 + self.amount_of_movement_x
       self.circle_marker.pose.position.y = 1.0 + self.amount_of_movement_y
       self.circle_marker.pose.position.z = 0.5
       self.circle_marker.pose.orientation.x = 0.0
       self.circle_marker.pose.orientation.y = 0.0
       self.circle_marker.pose.orientation.z = 0.0
       self.circle_marker.pose.orientation.w = 1.0

       # サイズを設定 (長方形の幅、高さ、奥行き)
       self.circle_marker.scale.x = 0.5  # 長方形の長さ
       self.circle_marker.scale.y = 0.5  # 長方形の幅
       self.circle_marker.scale.z = 0.01  # 長方形の高さ

       # 色を設定 (RGBA)
       self.circle_marker.color.r = 0.0
       self.circle_marker.color.g = 0.0
       self.circle_marker.color.b = 1.0
       self.circle_marker.color.a = 0.8  # 不透明度

       # publish rectangle marker
       self.circle_marker_publisher.publish(self.circle_marker)

    def create_goal_marker(self):
       self.goal_marker.header.frame_id = "map"  # 座標系を指定
       self.goal_marker.header.stamp = self.get_clock().now().to_msg()
       self.goal_marker.ns = "circle_namespace"     # 名前空間を設定
       self.goal_marker.id = 0                         # マーカーのID
       self.goal_marker.type = Marker.CYLINDER             # 円には CYLINDER タイプを使用
       self.goal_marker.action = Marker.ADD            # マーカーを表示

       # 位置と姿勢を設定
       self.goal_marker.pose.position.x = 3.0
       self.goal_marker.pose.position.y = 1.0
       self.goal_marker.pose.position.z = 0.5
       self.goal_marker.pose.orientation.x = 0.0
       self.goal_marker.pose.orientation.y = 0.0
       self.goal_marker.pose.orientation.z = 0.0
       self.goal_marker.pose.orientation.w = 1.0

       # サイズを設定 (円の幅、高さ、奥行き)
       self.goal_marker.scale.x = 0.5  # 円の長さ
       self.goal_marker.scale.y = 0.5  # 円の幅
       self.goal_marker.scale.z = 0.01  # 円の高さ

       # 色を設定 (RGBA)
       self.goal_marker.color.r = 1.0
       self.goal_marker.color.g = 0.0
       self.goal_marker.color.b = 0.0
       self.goal_marker.color.a = 0.8  # 不透明度

       # publish rectangle marker
       self.goal_marker_publisher.publish(self.goal_marker)

    def calculation_radian(self):

        # goal_position
        goal_x = self.goal_marker.pose.position.x
        goal_y = self.goal_marker.pose.position.y

        # follow_marker_position
        follow_marker_x = self.circle_marker.pose.position.x
        follow_marker_y = self.circle_marker.pose.position.y

        # radian
        radian = 0.0

        radian = math.atan2((goal_y - follow_marker_y), (goal_x - follow_marker_x))

        return radian

    def calculation_quaternion(self):
        # yaw, pitch, roll をラジアンで指定
        yaw = self.calculation_radian()
        pitch = 0.0
        roll = 0.0

        # クォータニオンを手動で計算
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        return q

    def distance_between_follow_marker_and_goal(self):

        # goal_position
        goal_x = self.goal_marker.pose.position.x
        goal_y = self.goal_marker.pose.position.y

        # follow_marker_position
        follow_marker_x = self.circle_marker.pose.position.x
        follow_marker_y = self.circle_marker.pose.position.y


        sub_distance = (goal_y - follow_marker_y)**2 + (goal_x - follow_marker_x)**2
        return math.sqrt(sub_distance)

    def amount_of_movement(self):

        if self.distance_between_follow_marker_and_goal() > 0.03 and not self.stop_trigger:
            self.amount_of_movement_x += math.sin(self.calculation_radian() + math.radians(90)) * 0.03
            self.amount_of_movement_y -= math.cos(self.calculation_radian() + math.radians(90)) * 0.03

        #else:
            #time.sleep(3)
            #self.amount_of_movement_x = 0.0
            #self.amount_of_movement_y = 0.0

    def check_collision_callback(self):
        # 矩形マーカーの位置・大きさ・回転角度
        center = (self.rectangle_marker.pose.position.x, self.rectangle_marker.pose.position.y)  # 中心位置
        width = self.rectangle_marker.scale.x  # 幅
        height = self.rectangle_marker.scale.y  # 高さ
        angle = self.calculation_radian()  # 回転角度

        # 矩形マーカーの頂点を計算
        vertices = self.calculate_rectangle_vertices(center, width, height, angle)

        #self.get_logger().info(f'vertices: {self.vertices}')

        # 重なり判定
        if self.check_overlap_with_local_cost(vertices):
            self.stop_trigger = True
            self.get_logger().info("Rectangle marker overlaps with the local cost.")
        else:
            self.stop_trigger = False
            self.get_logger().info("No overlap with the local cost.")

    def check_overlap_with_local_cost(self, vertices):
        #矩形の頂点とローカルコストの重なりを判定

        if self.local_costmap is None:
            return

        # costmap transform
        self.costmap_point.x = self.local_costmap.info.origin.position.x
        self.costmap_point.y = self.local_costmap.info.origin.position.y
        self.costmap_point.z = self.local_costmap.info.origin.position.z
        self.costmap_stamp = self.local_costmap.header.stamp
        self.transform_to_map()

        # Costmap properties
        costmap_resolution = self.local_costmap.info.resolution
        #costmap_origin_x = self.local_costmap.info.origin.position.x
        #costmap_origin_y = self.local_costmap.info.origin.position.y
        costmap_origin_x = self.point_in_map.point.x # costmap to map from odom
        costmap_origin_y = self.point_in_map.point.y # costmap to map from odom
        costmap_width = self.local_costmap.info.width
        costmap_height = self.local_costmap.info.height

        #self.get_logger().info(f'costmap_x_y: {self.costmap_point.x, self.costmap_point.y, costmap_resolution}')

        for (vx, vy) in vertices:
            # グリッド座標に変換
            grid_x = int((vx - costmap_origin_x) / costmap_resolution)
            grid_y = int((vy - costmap_origin_y) / costmap_resolution)
            #grid_x = int(vy / costmap_resolution)
            #grid_y = int(vx / costmap_resolution)

            # ローカルコストマップの範囲内であるか確認
            if -(costmap_width / 2) <= grid_x < (costmap_width / 2) and -(costmap_height / 2) <= grid_y < (costmap_height/ 2):
                #index = grid_y * costmap_width + grid_x
                index = grid_y * costmap_width + (costmap_height - grid_x)
                if self.local_costmap.data[index] >= 70:  # 50以上なら重なりがあるとみなす
                    return True
                #else:
                    #return False
        return False

    def calculate_rectangle_vertices(self, center, width, height, angle):
        #中心位置、幅、高さ、回転角度から矩形の頂点を計算
        dx = width / 2
        dy = height / 2

        corners = [
            (center[0], -dy),
            (center[0], dy),
            (-dx, dy),
            (-dx, -dy)
        ]

        rotated_corners = []
        for (x, y) in corners:
            rotated_x = center[0] + x * math.cos(angle) - y * math.sin(angle)
            rotated_y = center[1] + x * math.sin(angle) + y * math.cos(angle)
            rotated_corners.append((rotated_x, rotated_y))

        return rotated_corners

    def transform_to_map(self):
        try:
            # costmap 座標（odom 基準）の PointStamped を作成
            point_in_odom = PointStamped()
            point_in_odom.header.frame_id = "odom"
            #point_in_odom.header.stamp = self.get_clock().now().to_msg()
            point_in_odom.header.stamp = self.local_costmap.header.stamp
            point_in_odom.point.x = self.costmap_point.x
            point_in_odom.point.y = self.costmap_point.y
            point_in_odom.point.z = self.costmap_point.z
            # odom から map への変換を適用
            self.point_in_map = self.tf_buffer.transform(point_in_odom, "map")
        except Exception as e:
            self.get_logger().warn(f"Transform failed: {e}")
            return None

    def set_goal_pose(self):
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.pose.position.x = self.circle_marker.pose.position.x  # ゴールのx座標
        self.goal_pose.pose.position.y = self.circle_marker.pose.position.y  # ゴールのy座標
        self.goal_pose.pose.orientation = self.calculation_quaternion()  # ゴールの方向

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.goal_pose
        self._action_client.wait_for_server()

        # 目標を送信
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        #self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        #self.get_logger().info('Goal accepted')
        #self._get_result_future = goal_handle.get_result_async()
        #self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().status

        # 結果が成功ならば、もう一度目標を再送信する
        if result == 4:  # 4はナビゲーション成功のステータス
            self.get_logger().info('Goal reached, sending again...')
            #time.sleep(1)  # 必要に応じて少し待機
            #self.send_goal()
        else:
            self.get_logger().info(f'Goal failed with status: {result}')
            # 必要に応じて再試行や他の処理を追加

def main(args=None):
    rclpy.init(args=args)
    node = GoalAndMoveNode()
    node.set_goal_pose()
    node.send_goal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
