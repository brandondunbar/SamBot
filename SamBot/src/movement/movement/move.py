from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MoveService(Node):
    """Handles robot movement.
    """

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'move', self.move)

    def move(self, request, response):
        response.sum = 0
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response


def main(args=None):
    rclpy.init(args=args)
    move_service = MoveService()
    rclpy.spin(move_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

