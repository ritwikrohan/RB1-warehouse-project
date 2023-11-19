import rclpy
from rclpy.node import Node
from attach_shelf.srv import GoToLoading
from geometry_msgs.msg import Twist
import time
from copy import deepcopy
from geometry_msgs.msg import PoseStamped, Polygon, Point32
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from math import cos, sin, pi
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
class RobotStateMachine(Node):


    def __init__(self):
        super().__init__('statemachine_node')
        client_cb_group = MutuallyExclusiveCallbackGroup()
        self.client = self.create_client(GoToLoading, "/approach_shelf", callback_group=client_cb_group)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GoToLoading.Request()
        self.publisher_ = self.create_publisher(Twist, "/robot/cmd_vel", 10)
        self.local_shelf_footprint_publisher = self.create_publisher(Polygon,"/local_costmap/footprint", 10)
        self.global_shelf_footprint_publisher = self.create_publisher(Polygon,"/global_costmap/footprint", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.req_to_attach = True
        self.nav = BasicNavigator()
        self.speed_msg = Twist() 
        self.initial_pose = PoseStamped()
        self.goal_pose = PoseStamped()
        self.footprint = Polygon()
        self.robot_stage = {
            "initial_stage": [0.0, 0.0, 0.0 , 1.0],
            "loading_stage": [5.821, 0.0, -0.7933533, 0.6087614],
            "final_position": [-3.791, 1.254]}
        self.stage_number = 1
        self.goal_reached = False

    def publish_footprint_shelf(self):

        footprint = Polygon()
        point1 = Point32()
        point1.x = 0.40
        point1.y = 0.40

        point2 = Point32()
        point2.x = 0.40
        point2.y = -0.40

        point3 = Point32()
        point3.x = -0.40
        point3.y = -0.40

        point4 = Point32()
        point4.x = -0.40
        point4.y = 0.40

        footprint.points = [point1, point2, point3, point4]
            
        self.local_shelf_footprint_publisher.publish(footprint)
        self.global_shelf_footprint_publisher.publish(footprint)

    def publish_footprint_robot(self):
        footprint = Polygon()
        points = []
        for angle in range(0, 360, 10):
            point = Point32()
            point.x = 0.25 * cos(angle * pi / 180)  
            point.y = 0.25 * sin(angle * pi / 180)  
            points.append(point)

            footprint.points = points
            
            self.local_shelf_footprint_publisher.publish(footprint)
            self.global_shelf_footprint_publisher.publish(footprint)

        

    def send_request(self, a):
        self.req.attach_to_shelf = a
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def set_init_pose(self):
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.nav.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = self.robot_stage["initial_stage"][0]
        self.initial_pose.pose.position.y = self.robot_stage["initial_stage"][1]
        self.initial_pose.pose.orientation.z = self.robot_stage["initial_stage"][2]
        self.initial_pose.pose.orientation.w = self.robot_stage["initial_stage"][3]
        self.nav.setInitialPose(self.initial_pose)
        print('Initial Position Set')
        # Wait for navigation to activate fully
        self.nav.waitUntilNav2Active()

    def go_to_pose(self, stages, stage_name):
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = stages[stage_name][0]
        self.goal_pose.pose.position.y = stages[stage_name][1]
        self.goal_pose.pose.orientation.z = stages[stage_name][2]
        self.goal_pose.pose.orientation.w = stages[stage_name][3]
        self.nav.goToPose(self.goal_pose)

        # Do something during your route
        # (e.x. queue up future tasks or detect person for fine-tuned positioning)
        # Print information for workers on the robot's ETA for the demonstration
        i = 0
        while not self.nav.isTaskComplete():
            i = i + 1
            feedback = self.nav.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival at ' + stage_name +
                    ' for worker: ' + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')
        
        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print('! Bringing product to shipping destination (' + stage_name + ')...')


        elif result == TaskResult.CANCELED:
            print('Task at ' + stage_name  +
                ' was canceled. Returning to staging point...')
            exit(-1)

        elif result == TaskResult.FAILED:
            print('Task at ' + stage_name + ' failed!')
            exit(-1)

        while not self.nav.isTaskComplete():
            pass

    def timer_callback(self):
        self.timer.cancel()
        self.control_loop()

    def control_loop(self):
        # rate = self.node.create_rate(1)
        while rclpy.ok() and not self.goal_reached:
            if self.stage_number == 1:
                self.set_init_pose()
                self.stage_number = 2
                # self.goal_reached=True
                
            # elif self.stage_number == 2:
            #     self.go_to_pose(self.robot_stage, "loading_stage")
            #     self.stage_number = 3
            # elif self.stage_number==3:
            #     response = self.send_request(True)
            #     print(response)
            #     self.publish_footprint_shelf()
            #     self.goal_reached=True
            
def main(args=None):
    rclpy.init(args=args)
    state_machine = RobotStateMachine()
    executor = MultiThreadedExecutor()
    executor.add_node(state_machine)
    executor.spin()
    # rclpy.spin(state_machine)
    state_machine.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()