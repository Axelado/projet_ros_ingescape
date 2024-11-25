import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import base64

from PIL import Image as PIL_Image
from PIL import ImageDraw
from geometry_msgs.msg import TransformStamped
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

import ingescape as igs

# agent_name = "image_on_whiteboard"
# port = 5670
# network_device = "wlo1"

class ImageOnWhiteboardNode(Node):
    def __init__(self):
        super().__init__('image_on_whiteboard')
        self.image_subscription = self.create_subscription(
            Image,
            '/pizibot_camera/image_raw',
            self.image_callback,
            10
        )
        
        self.tf_subscription = self.create_subscription(
            TransformStamped,
            '/base_link_to_map_transform',
            self.transform_callback,
            10
        )
        
        self.frame_counter = 0  # Initialisation du compteur
        self.frame_skip = 6  # Traite 1 image sur 6 par exemple
        self.last_item_id = [None, None]   # (camera, map)
        self.current_item_id = [None, None]
        
        self.bridge = CvBridge()
        
        self.declare_parameter('agent_name', 'image_on_whiteboard')
        self.declare_parameter('network_device', 'wlo1')
        self.declare_parameter('port', 5670)
        
        agent_name = self.get_parameter('agent_name').get_parameter_value().string_value
        network_device = self.get_parameter('network_device').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value

        self.init_ingescape_agent(agent_name, network_device, port)
        igs.service_init("elementCreated", self.update_element_id_callback, None)
        igs.service_arg_add("elementCreated", "elementID", igs.INTEGER_T)

        self.get_logger().info("Image On Whiteboard node started, waiting for images...")
        
    def update_element_id_callback(self, sender_agent_name, sender_agent_uuid, service_name, arguments, token, my_data):
        if token=="0":
            self.current_item_id[0] = arguments[0]
        else:
            self.current_item_id[1] = arguments[0]
        # self.get_logger().info(f" {arguments[0]}...")
        
    def image_callback(self, msg):
        
        # pour ne pas traiter toutes les images
        self.frame_counter += 1
        if self.frame_counter % self.frame_skip != 0:
            return  # Ne traite pas cette image et sort de la fonction
        
        try:
            # Conversion du message ROS Image en image OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.display_image(cv_image, (100.0, 200.0), 3, image_type=0)
            
        except Exception as e:
            self.get_logger().error(f"image non envoyé: {e}")
    
    def transform_callback(self, msg:TransformStamped):
        try:
            map_height = 386
            map_wigth = 399
            map_origin = (-10.4, -9.77)  
            map_resolution = 0.05
            
            x = msg.transform.translation.x
            y = msg.transform.translation.y
            x_r = x - map_origin[0]
            y_r = y - map_origin[1]
            
            x_p = x_r/map_resolution
            y_p = map_height - y_r/map_resolution
            point = (x_p, y_p)  # Position of the red dot (in pixels)
            
            
            map_file_pgm = os.path.join(get_package_share_directory("pizibot_navigation"), 'map', 'my_world_map_save.pgm')    
                    
            cv_image = self.convert_pgm_to_png_with_red_point(map_file_pgm, point)
            
            self.display_image(cv_image, (800.0, 200.0), 0.9, image_type=1)
        
        except Exception as e:
            self.get_logger().error(f"image non envoyé: {e}")

    def display_image(self, cv_image, position = (0,0), ratio=4, image_type = 0):  #image_type 0: camera, 1: map
    
        try:
            # Encodage de l'image en format JPEG
            _, buffer = cv2.imencode('.jpg', cv_image)

            image_base64 = base64.b64encode(buffer).decode('utf-8')

            height, width, _ = cv_image.shape
            x, y = position            
            arguments_list = (image_base64, x, y, width/ratio, height/ratio)    
            igs.service_call("Whiteboard", "addImage", arguments_list, str(image_type))
            
            if self.last_item_id[image_type]:
                igs.service_call("Whiteboard", "remove", self.last_item_id[image_type], "")
            
            self.last_item_id[image_type] = self.current_item_id[image_type]
            self.get_logger().info(f'Image envoyé ...') 
            
        except Exception as e:
            self.get_logger().error(f"image non envoyé: {e}")
                
            
    def init_ingescape_agent(self, agent_name, network_device, port):
        igs.agent_set_name(agent_name)
        igs.definition_set_version("1.0")
        igs.log_set_console(True)
        igs.log_set_file(True, None)

        igs.debug(f"Ingescape version: {igs.version()} (protocol v{igs.protocol()})")


        igs.start_with_device(network_device, port)


    def convert_pgm_to_png_with_red_point(self, input_pgm_path, point_position, point_radius=3):
        """
        Converts a PGM image to PNG and adds a red dot at the specified position.

        :param input_pgm_path: Path to the input .pgm file
        :param point_position: Tuple (x, y) representing the center pixel position of the red dot
        :param point_radius: Radius of the red dot (default is 3 pixels)
        """
        # Load the PGM image
        pgm_image = PIL_Image.open(input_pgm_path)

        # Convert the image to RGB mode to allow adding colors
        rgb_image = pgm_image.convert("RGB")

        # Add a red dot at the specified position
        draw = ImageDraw.Draw(rgb_image)
        x, y = point_position
        draw.ellipse(
            (x - point_radius, y - point_radius, x + point_radius, y + point_radius),
            fill="red",
            outline="red"
        )


        # Convert the PIL image to a NumPy array (format compatible with OpenCV)
        cv2_image = np.array(rgb_image)

        # Convert RGB to BGR (OpenCV uses BGR by default)
        cv2_image = cv2.cvtColor(cv2_image, cv2.COLOR_RGB2BGR)
        
        return cv2_image
    

def main(args=None):
    rclpy.init(args=args)
    node = ImageOnWhiteboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        igs.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



