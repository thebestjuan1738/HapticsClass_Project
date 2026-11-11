
import pygame
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Int8
from geometry_msgs.msg import Point

# Global screen dimensional parameters
SCREEN_WIDTH = 1000
SCREEN_HEIGHT = 800


class Interface_Node(Node):
    def __init__(self):
        super().__init__('interface_node')
        # Initialize Node
        self.subscriber_pos = self.create_subscription(Point,'pos_topic',self.callback_pos,1)
        self.subscriber_force = self.create_subscription(Point,'force_topic',self.callback_force,1)
        self.subscriber_button = self.create_subscription(Bool, 'button_topic',self.callback_button,1)

        self.publisher_screen = self.create_publisher(Int8,'screen_topic',10)

        self.position = [0, 0, 0]
        self.position_limits = 40      # i.e. allowable in each direction from haptic device [mm]

        self.force = [0, 0, 0]

        self.curr_screen_flag = 0

        self.button = False

        self.screen_timer = self.create_timer(0.05, self.publish_screen)

        

    def callback_pos(self, pos_msg):
        self.haptic_position = [pos_msg.x,pos_msg.y,pos_msg.z]
        self.position[0] = SCREEN_WIDTH*(self.haptic_position[0] + self.position_limits)/(2*self.position_limits)
        self.position[2] = SCREEN_HEIGHT*(self.haptic_position[2] + self.position_limits)/(2*self.position_limits)
        
        # self.get_logger().info('x position received: %f' % self.position[0])
        # self.get_logger().info('z position received: %f' % self.position[2])

    def callback_force(self, force_msg):
        self.force = [force_msg.x,force_msg.y,force_msg.z]

        # self.get_logger().info('Applied y-force: %f' % self.force[1])

    def callback_button(self, button_msg):
        self.button = button_msg.data

        # self.get_logger().info('Button pressed: %s' % self.button)

    def publish_screen(self):
        msg_screen = Int8(data=self.curr_screen_flag)
        self.publisher_screen.publish(msg_screen)

        # self.get_logger().info('Current game screen: %i' % msg_screen.data)



def main(args=None):
        rclpy.init(args=args)
        node = Interface_Node()

        pygame.init()   # Begins the Game/Defines the game object

        button_font = pygame.font.Font(None, 32)

        # Setting Screen Parameters
        screen_color = (255,255,255)
        screen = pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HEIGHT))

        # Setting user cursor Graphics
        cursor_len = 50
        cursor = pygame.Surface((cursor_len,cursor_len), pygame.SRCALPHA)
        cursor_center = pygame.Vector2(25,25)
        cursor_color = (0, 0, 0)

        center_dot_radius = 5
        pygame.draw.circle(cursor, cursor_color,cursor_center, center_dot_radius)       # Draws the center dot of the cursor

        pygame.mouse.set_visible(False)         # hides the regular mouse cursor from the game
        
        for i in range(4):      # Draws the radiating lines from the cursor center
            direction = pygame.Vector2(np.cos(90*i*np.pi/180),np.sin(90*i*np.pi/180))
            cursor_line_start = cursor_center + direction*10
            cursor_line_end = cursor_center + direction*20

            pygame.draw.line(cursor, cursor_color, cursor_line_start, cursor_line_end, width=3)

        cursor_pos = pygame.Vector2((SCREEN_WIDTH)/2 , (SCREEN_HEIGHT)/2)            # Sets the starting point for the cursor

        # Initializing a surface vector for drawn lines
        drawn_lines = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
        prev_drawn_line = 0
        line_width = 40

        # Creating a surface for a button to initialize next screen of the experiment
        NEXT_BUTTON_WIDTH = 200
        NEXT_BUTTON_HEIGHT = 100
        next_button_pos = (SCREEN_WIDTH/2 -NEXT_BUTTON_WIDTH/2, SCREEN_HEIGHT - NEXT_BUTTON_HEIGHT - 20)

        next_button = pygame.Surface((NEXT_BUTTON_WIDTH,NEXT_BUTTON_HEIGHT))
        next_button.fill((255,0,0))
        next_button_box = next_button.get_rect(topleft=next_button_pos)

        next_button_text = button_font.render("Evaluate Design", True, (0, 0, 0))
        next_button_text_box = next_button_text.get_rect(center=(NEXT_BUTTON_WIDTH//2,NEXT_BUTTON_HEIGHT//2))
        # next_button.blit(next_button_text,next_button_text_box)

        # Flag Variable to track current screen
        # curr_screen_flag = 0


        # Initializing running game loop
        running_flag = True
        try:
            while running_flag == True and rclpy.ok():
                
                # Checks for events in the pygame object
                for event in pygame.event.get():    
                    if event.type == pygame.QUIT:   # If the game is quit, the loop stops running
                        running_flag = False

                match node.curr_screen_flag:       # Screen display for task
                    case 0:
                        # Displays all current surfaces to update graphics
                        screen.fill(screen_color)
        
                        screen.blit(drawn_lines, (0,0))
                        
                        screen.blit(next_button, next_button_box)
                        next_button.blit(next_button_text,next_button_text_box)
        
                        rclpy.spin_once(node, timeout_sec=0.0)
                        haptic_position = node.position
                        cursor_pos = pygame.Rect(haptic_position[0],haptic_position[2],cursor_len,cursor_len)
                        screen.blit(cursor, cursor_pos)

                        # Sketching game drawing
                        # if node.force[1]>0 and node.button==True:          # If left mouse button is clicked, draws a connecting line on the drawn_lines surface
                        #     cursor_pos_center = pygame.Vector2(cursor_pos[0] + cursor_len/2,cursor_pos[1] + cursor_len/2)
                        #     if prev_drawn_line == 0:        # The first point of the stroke is made a circle to start
                        #         pygame.draw.circle(drawn_lines, cursor_color, cursor_pos_center,line_width/2)
                        #     else:               # Subsequent points of the stroke are connected with lines
                        #         pygame.draw.line(drawn_lines, cursor_color, prev_drawn_line, cursor_pos_center, width=line_width)
                        #     prev_drawn_line = cursor_pos_center     # Set current position as previous position to draw connecting line
                        # else:
                        #     prev_drawn_line = 0

                        # Truss design drawing
                        if node.force[1]>0 and node.button==True:          # If left mouse button is clicked, draws a connecting line on the drawn_lines surface
                            cursor_pos_center = pygame.Vector2(cursor_pos[0] + cursor_len/2,cursor_pos[1] + cursor_len/2)
                            if prev_drawn_line == 0:        # The first point of the stroke is made a circle to start
                                pygame.draw.circle(drawn_lines, cursor_color, cursor_pos_center,line_width/2)
                                start_point = cursor_pos_center
                            prev_drawn_line = cursor_pos_center     # Set current position as previous position to draw connecting line
                        elif prev_drawn_line is not 0:
                            end_point = pygame.Vector2(cursor_pos[0] + cursor_len/2,cursor_pos[1] + cursor_len/2)
                            pygame.draw.circle(drawn_lines,cursor_color,start_point, line_width/2)
                            pygame.draw.line(drawn_lines, cursor_color, start_point, end_point, width=line_width)
                            pygame.draw.circle(drawn_lines, cursor_color, end_point, line_width/2)
                            pygame.draw.circle(drawn_lines, (255,0,0), start_point, line_width/4)
                            pygame.draw.circle(drawn_lines, (255,0,0), end_point, line_width/4)
                            prev_drawn_line = 0

                        if next_button_box.collidepoint(cursor_pos.center):
                            # Change color of button
                            next_button.fill((155,0,0))
                            next_button.blit(next_button_text,next_button_text_box) 
                            if node.button == True:
                                # update to the next screen
                                node.curr_screen_flag = 1
                        else:
                            next_button.fill((255,0,0))
                            next_button.blit(next_button_text,next_button_text_box)

                    case 1:
                        # Displays all current surfaces to update graphics
                        screen.fill(screen_color)
        
                        rclpy.spin_once(node, timeout_sec=0.0)
                        haptic_position = node.position
                        cursor_pos = pygame.Rect(haptic_position[0],haptic_position[2],cursor_len,cursor_len)
                        screen.blit(cursor, cursor_pos)
                
                pygame.display.update()
        
        except KeyboardInterrupt:
            pass

        finally:
            pygame.quit()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()