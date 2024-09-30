import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SaveMap

class SaveMapNode(Node):
    def __init__(self):
        super().__init__('save_map_node')
        self.cli = self.create_client(SaveMap, '/slam_toolbox/save_map')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.req = SaveMap.Request()
        self.req.name = '/home/max/ros2_ws/my_map'

    def save_map(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            self.get_logger().info('Map saved successfully.')
        else:
            self.get_logger().error('Failed to save map.')

def main(args=None):
    rclpy.init(args=args)
    save_map_node = SaveMapNode()
    save_map_node.save_map()
    rclpy.shutdown()

if __name__ == '__main__':
    main()