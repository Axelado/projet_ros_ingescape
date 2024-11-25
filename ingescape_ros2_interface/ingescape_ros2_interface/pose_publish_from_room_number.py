import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Header

import json

import ingescape as igs

# agent_name = "room_number_ros"
# port = 5670
# network_device = "wlo1"


class PosePublishFromRoomNumber(Node):
    def __init__(self):
        super().__init__('pose_publish_from_room_number')
        self.room_data_path = os.path.join(get_package_share_directory("pizibot_navigation"), 'data','rooms_data.json')
        self.rooms_data = self.read_rooms_data()
        self.pose_publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        
        self.room_number_insgecape = None
        self.declare_parameter('agent_name', 'room_number_ros')
        self.declare_parameter('network_device', 'wlo1')
        self.declare_parameter('port', 5670)
        
        agent_name = self.get_parameter('agent_name').get_parameter_value().string_value
        network_device = self.get_parameter('network_device').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value

        self.init_ingescape_agent(agent_name, network_device, port)
        
        self.timer = self.create_timer(1.0, self.check_new_room_number_to_go)
        
        
        self.get_logger().info("pose_publish_from_room_number has started")
        
    def check_new_room_number_to_go(self):
        if self.room_number_insgecape:
            self.get_logger().info("Room number received")
            point = self.select_room(self.room_number_insgecape)
            self.room_number_insgecape = None
            if point: 
                self.publish_pose(point)
        
    def select_room(self, room_number):
        try:
            point = self.rooms_data[str(room_number)]
            self.get_logger().info(f"La salle {room_number} est à la position : {point}")
            return point
        except KeyError:
            self.get_logger().info(f"La salle {room_number} n'existe pas")
            return None
        
    def publish_pose(self, point):
        X = float(point['x'])
        Y = float(point['y'])
        
        msg = PoseStamped()
        # Set the header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg() 
        msg.header.frame_id = 'map'
        
        # Set the pose
        msg.pose.position = Point(x=X, y=Y, z=0.0)
        msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.pose_publisher_.publish(msg)
        self.get_logger().info("Goal pose published")
        
    
    def read_rooms_data(self):
        with open(self.room_data_path, 'r') as f:
            return json.load(f)
    
    def init_ingescape_agent(self, agent_name, network_device, port):
        igs.agent_set_name(agent_name)
        igs.definition_set_version("1.0")
        igs.log_set_console(True)
        igs.log_set_file(True, None)

        igs.debug(f"Ingescape version: {igs.version()} (protocol v{igs.protocol()})")
        
        igs.input_create("room_number", igs.INTEGER_T, None)

        igs.observe_input("room_number", self.room_number_ingescape_callback, None)

        igs.start_with_device(network_device, port)
        
    def room_number_ingescape_callback(self, iop_type, name, value_type, value, my_data):
        self.room_number_insgecape = value
        self.get_logger().info(f'new room number to go from ingescape {self.room_number_insgecape}')
    
    
    
def main(args=None):
    rclpy.init(args=args)
    node = PosePublishFromRoomNumber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
