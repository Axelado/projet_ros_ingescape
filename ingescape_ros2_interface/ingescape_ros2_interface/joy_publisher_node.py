import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import random
import json

import ingescape as igs

# agent_name = "joystick_ros"
# port = 5670
# network_device = "wlo1"

class JoyPublisherNode(Node):
    def __init__(self):
        super().__init__('joy_publisher')
        
        # Création du publisher pour le topic 'joy'
        self.publisher_ = self.create_publisher(Joy, 'joy', 10)
        
        # Timer pour publier un message chaque seconde
        self.timer = self.create_timer(0.05, self.publish_joy_message)
        self.get_logger().info('Joy Publisher Node has been started.')
        self.joystick_state_insgecape = None
        
        self.declare_parameter('agent_name', 'joystick_ros')
        self.declare_parameter('network_device', 'wlo1')
        self.declare_parameter('port', 5670)
        
        agent_name = self.get_parameter('agent_name').get_parameter_value().string_value
        network_device = self.get_parameter('network_device').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value

        self.init_ingescape_agent(agent_name, network_device, port)
        
        
    def publish_joy_message(self):
        
        if self.joystick_state_insgecape:    
            joy_msg = self.convert_json_to_joy(self.joystick_state_insgecape)
            self.joystick_state_insgecape = None
            
            if joy_msg:
                # Publier le messages
                self.publisher_.publish(joy_msg)
                self.get_logger().info(f'Published Joy message: {joy_msg}')
            else: 
                self.get_logger().error(f'Joy not publish')
            
    
    def convert_json_to_joy(self, json_string):
        
        try:
            # Charger la chaîne de caractères JSON en un dictionnaire
            joystick_data = json.loads(json_string)
            
            # Créer un message ROS 2 de type Joy
            joy_msg = Joy()
            
            # Assurer que les champs nécessaires sont dans les données
            if "timestamp" in joystick_data:
                joy_msg.header.stamp.sec = joystick_data["timestamp"]["sec"]
                joy_msg.header.stamp.nanosec = joystick_data["timestamp"]["nanosec"]
            
            if "axes" in joystick_data:
                for i in range(len(joystick_data["axes"])):
                    joystick_data["axes"][i] = -joystick_data["axes"][i]
                if "hats" in joystick_data:
                    joystick_data["axes"].append(float(joystick_data["hats"][0][0]))
                    joystick_data["axes"].append(float(joystick_data["hats"][0][1]))
                    print(joystick_data["hats"][0])
                
                joy_msg.axes = joystick_data["axes"]
                
            if "buttons" in joystick_data:
                joy_msg.buttons = joystick_data["buttons"]
            
            return joy_msg
        except Exception as e:
            self.get_logger().error(f'can not convert: {e}')
            return None
            
    def init_ingescape_agent(self, agent_name, network_device, port):
        igs.agent_set_name(agent_name)
        igs.definition_set_version("1.0")
        igs.log_set_console(True)
        igs.log_set_file(True, None)

        igs.debug(f"Ingescape version: {igs.version()} (protocol v{igs.protocol()})")
        
        igs.input_create("joystick_state", igs.STRING_T, None)

        igs.observe_input("joystick_state", self.joystick_ingescape_callback, None)

        igs.start_with_device(network_device, port)
    
    def joystick_ingescape_callback(self, iop_type, name, value_type, value, my_data):
        self.joystick_state_insgecape = value
        self.get_logger().info(f'new joystick state from ingescape {self.joystick_state_insgecape}')
        
        

def main(args=None):
    rclpy.init(args=args)
    node = JoyPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        igs.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
