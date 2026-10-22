import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class ROSBridgePublisher(Node):
    def __init__(self):
        super().__init__('ros_action_bridge')
        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.get_logger().info('AutoFleet Edge ROS 2 Bridge Initialized.')

    def publish_goal(self, x: float, y: float):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0  # Neutral orientation quaternion
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published Navigation Goal -> X: {x}, Y: {y}')

if __name__ == '__main__':
    rclpy.init()
    bridge = ROSBridgePublisher()
    bridge.publish_goal(1.5, 1.0)
    bridge.destroy_node()
    rclpy.shutdown()
