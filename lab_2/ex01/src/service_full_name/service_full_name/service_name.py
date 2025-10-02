from full_name_interface.srv import SummFullName

import rclpy
from rclpy.node import Node


class FullNameService(Node):

    def __init__(self):
        super().__init__('service_name')
        self.srv = self.create_service(SummFullName, 'SummFullName', self.summ_full_name_callback)

    def summ_full_name_callback(self, request, response):
        response.full_name = request.last_name + ' ' + request.first_name + ' ' + request.patronymic

        return response


def main():
    rclpy.init()

    full_name_service = FullNameService()

    rclpy.spin(full_name_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()