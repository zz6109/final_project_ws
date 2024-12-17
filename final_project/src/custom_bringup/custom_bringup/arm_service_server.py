import rclpy
from rclpy.node import Node
# from rclpy.executors import MultiThreadedExecutor
from custom_interface.srv import ArmService
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionClient

class ArmPosition():
    mode1 = [0.0, -1.5, 1.5, 0.3]
    mode2 = [0.0, 1.5, -0.8, -0.8]
    mode3 = [3.2, -0.5, 0.6, 1.5]

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_service_server')
        
        self.create_service(ArmService, 'arm_control_service', self.handle_service_request)
        self.action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.get_logger().info('Service node initialized and waiting for requests...')
    
    def handle_service_request(self, request, response):
        # mode가 요청되었을 때만 동작
        self.get_logger().info('Received service request to execute action...')
        if request.mode == 'mode1':
            response.success = True
            response.message = 'Mode1 executed successfully!'
            self.send_goal(ArmPosition.mode1)
        elif request.mode == 'mode2':
            response.success = True
            response.message = 'Mode2 executed successfully!'
            self.send_goal(ArmPosition.mode2)
        elif request.mode == 'mode3':
            response.success = True
            response.message = 'Mode3 executed successfully!'
            self.send_goal(ArmPosition.mode3)
        else:
            response.success = False
            response.message = f'Invalid mode: {request.mode}'
            self.get_logger().info('select in mode1, mode2, mode3')
        return response
    
    def send_goal(self, mode):
        # 목표 trajectory 설정
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        
        point = JointTrajectoryPoint()
        point.positions = mode
        point.velocities = [0.0, 0.0, 0.0, 0.0]
        point.time_from_start.sec = 10
        point.time_from_start.nanosec = 0
        
        goal_msg.trajectory.points = [point]
        goal_msg.goal_time_tolerance.sec = 1
        goal_msg.goal_time_tolerance.nanosec = 0

        # 액션 서버 호출
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Sending goal to action server...')
        
        self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
    
    def feedback_callback(self, feedback):
        # self.get_logger().info(f'Received feedback: {feedback}')
        pass

def main(args=None):
    rclpy.init(args=args)
    arm_controller = ArmController()
    try:
        rclpy.spin(arm_controller)
    except KeyboardInterrupt:
        arm_controller.get_logger().info('Shutting down node...')
    finally:
        arm_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

