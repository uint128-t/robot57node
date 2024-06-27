import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped,Twist,PoseStamped,Pose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros.transform_broadcaster import TransformBroadcaster
from rclpy.qos import QoSProfile
import PIL.Image
from io import BytesIO
import numpy as np
import base64
import requests
import math
import queue

goals = queue.Queue()
vels = queue.Queue()

class MyNode(Node):
    def __init__(self):
        super().__init__('node1')
        self.subscription_map = self.create_subscription(OccupancyGrid,'map',self.map_callback,10)
        # self.subscription_pos = self.create_subscription(Pose,'robot_position',self.pose_callback,10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=ReentrantCallbackGroup())

        self.get_logger().info('Node1 started!')

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return

        self.get_logger().info('Navigating to goal')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()


    def timer_callback(self):
        # poll for pipe
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
            rotation = trans.transform.rotation
            yaw = math.atan2(2.0 * (rotation.w * rotation.z + rotation.x * rotation.y), 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z))
            # self.get_logger().info(f"Robot position: x={trans.transform.translation.x}, y={trans.transform.translation.y}, r={yaw}")
            requests.get("http://localhost:4809/msg/pos",{"x":trans.transform.translation.x,"y":trans.transform.translation.y,"r":yaw})
        except Exception as e:
            self.get_logger().info(f'Could not transform: {e}')

    def pose_callback(self,msg):
        self.get_logger().info(f"Pose: {msg.position.x}, {msg.position.z}")

    def map_callback(self,msg):
        grid = msg.data
        gridw = msg.info.width
        gridh = msg.info.height
        self.get_logger().info('Received map with width: %d, height: %d' % (msg.info.width, msg.info.height))
        self.get_logger().info(f"Origin: {msg.info.origin.position.x},{msg.info.origin.position.y}")

        image = np.full((gridw,gridh),0xde,dtype=np.uint8)
        for i in range(len(grid)):
            image[i%gridw,i//gridw] = 127 if grid[i]==-1 else 255*grid[i]
        img = PIL.Image.fromarray(image)
        bio = BytesIO()
        img.save(bio,"PNG")
        
        bio.seek(0)
        b64dt = base64.b64encode(bio.read())
        
        requests.get("http://localhost:4809/msg/map",{"map":b64dt.decode(),"x":msg.info.origin.position.x,"y":msg.info.origin.position.y,"r":msg.info.resolution})

def main(args=None):
    print("Start ROS2 node")
    while True:
        try:
            assert requests.get("http://localhost:4809/ping").content == b"online"
            break
        except:
            print("Waiting for server...")
            pass
    rclpy.init(args=args)
    node = MyNode()
    twistpublisher = rclpy.create_node("webcontrol").create_publisher(Twist,"cmd_vel",QoSProfile(depth=10))
    node.get_logger().info("Server started")
    while True:
        rclpy.spin_once(node)
        if not goals.empty():
            goal = goals.get_nowait()
            node.get_logger().info("Goal received")

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = node.get_clock().now().to_msg()

            pose.pose.position.x = float(goal["x"])
            pose.pose.position.y = float(goal["y"])
            pose.pose.position.z = float(goal["z"])

            pose.pose.orientation.x = float(goal["ox"])
            pose.pose.orientation.y = float(goal["oy"])
            pose.pose.orientation.z = float(goal["oz"])
            pose.pose.orientation.w = float(goal["ow"])
            
            node.send_goal(pose)
        if not vels.empty():
            vel = vels.get_nowait()
            print("controlling velocity:",vel)
            twist = Twist()
            twist.linear.x = float(vel["f"])
            twist.linear.y = float(vel["h"])
            twist.linear.z = 0.0

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = float(vel["r"])

            twistpublisher.publish(twist)
            
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()