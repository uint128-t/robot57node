import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
import queue

goals = queue.Queue()

class SenderNode(Node):

    def __init__(self):
        super().__init__('goal_sender')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=ReentrantCallbackGroup())

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

def main():
    print("Start goal sender")
    rclpy.init()
    goal_sender = SenderNode()

    while True:
        rclpy.spin_once(goal_sender)
        print("spin")
        try:
            goal = goals.popleft()
            print("Received goal")

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = goal_sender.get_clock().now().to_msg()

            pose.pose.position.x = float(goal["x"])
            pose.pose.position.y = float(goal["y"])
            pose.pose.position.z = float(goal["z"])

            pose.pose.orientation.x = float(goal["ox"])
            pose.pose.orientation.y = float(goal["oy"])
            pose.pose.orientation.z = float(goal["oz"])
            pose.pose.orientation.w = float(goal["ow"])
            
            goal_sender.send_goal(pose)
        except:
            pass