
import pygame
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from user_interface.truss_eval import SolveTruss

from std_msgs.msg import Bool, String, Float32MultiArray
from geometry_msgs.msg import Point

# Global screen dimensional parameters
SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 800


class SevenPointSlider:
    def __init__(self, left, right, y, font):
        self.left = left
        self.right = right
        self.y = y
        self.font = font
        self.value = None

        # Include the track and number labels in the hover area.
        self.hitbox = pygame.Rect(left - 16, y - 16, right - left + 33, 60)

    def _x_for_value(self, value):
        return round(self.left + (value - 1) * (self.right - self.left) / 6)

    def _select_at(self, x):
        fraction = (x - self.left) / (self.right - self.left)
        fraction = max(0.0, min(1.0, fraction))
        self.value = 1 + round(fraction * 6)

    def update_position(self, cursor_center):
        if self.hitbox.collidepoint(cursor_center):
            self._select_at(cursor_center[0])

    def draw(self, surface):
        ink = (45, 50, 60)
        accent = (43, 74, 140)

        pygame.draw.line(
            surface, ink, (self.left, self.y), (self.right, self.y), width=3
        )

        for value in range(1, 8):
            x = self._x_for_value(value)
            selected = value == self.value
            color = accent if selected else ink

            pygame.draw.line(
                surface, color, (x, self.y - 6), (x, self.y + 6), width=2
            )
            label = self.font.render(str(value), True, color)
            surface.blit(label, label.get_rect(midtop=(x, self.y + 16)))

            if selected:
                pygame.draw.circle(surface, accent, (x, self.y), 10)


class Interface_Node(Node):
    def __init__(self):
        super().__init__('interface_node')
        # Initialize Node
        self.subscriber_haptics = self.create_subscription(Float32MultiArray, 'haptic_topic',self.callback_all, qos_profile_sensor_data)

        self.publisher_interface = self.create_publisher(String,'interface_topic', qos_profile_sensor_data)

        self.position = [0, 0, 0]
        self.position_limits = 40      # i.e. allowable in each direction from haptic device [mm]

        self.force = [0, 0, 0]

        self.curr_screen_flag = 'design'

        self.button = False

        self.time = 0.0

        self.screen_timer = self.create_timer(0.05, self.publish_interface)


    def callback_all(self, msg):
        self.haptic_position = msg.data[0:3]
        self.position[0] = SCREEN_WIDTH*(self.haptic_position[0] + self.position_limits)/(2*self.position_limits)
        self.position[2] = SCREEN_HEIGHT*(self.haptic_position[2] + self.position_limits)/(2*self.position_limits)
        self.force = msg.data[6:9]
        self.time = msg.data[9]
        self.button = bool(msg.data[10])

        # self.get_logger().info('Time (sec): %2f' % self.time)

    def publish_interface(self):
        msg_interface = String(data=self.curr_screen_flag)
        self.publisher_interface.publish(msg_interface)

        # self.get_logger().info('Current game screen: %i' % msg_screen.data)



def main(args=None):
        rclpy.init(args=args)
        node = Interface_Node()

        pygame.init()   # Begins the Game/Defines the game object
        now = node.time

        # Initializing fonts for different text displays
        button_font = pygame.font.Font(None, 32)
        force_font = pygame.font.Font(None, 40)
        stress_font = pygame.font.Font(None,36)
        timer_font = pygame.font.Font(None, 72)

        # Setting Screen Parameters
        screen_color = (232, 218, 181)
        screen = pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HEIGHT))
        # screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)

        # Initializing timer parameters
        trial_length = 20.0     # Time of each trial countdown (sec)
        report_length = 15.0    # Time until next self report (sec)
        start_time = []
        trial_start_flag = False
        report_started_at = None
        paused_total_ms = 0
        rest_length = 10.0  # Time of rest between trials (sec)
        rest_started_at = None

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
        line_color = (138, 141, 143)
        screw_color = (0,0,0)
        prev_drawn_line = 0
        line_width = 40
        draw_flag = False

        # Creating a surface for a button to initialize next screen of the experiment
        NEXT_BUTTON_WIDTH = SCREEN_WIDTH/5
        NEXT_BUTTON_HEIGHT = 100
        next_button_pos = (SCREEN_WIDTH*3/5, SCREEN_HEIGHT - NEXT_BUTTON_HEIGHT - 20)

        next_button = pygame.Surface((NEXT_BUTTON_WIDTH,NEXT_BUTTON_HEIGHT))
        next_button.fill((100,100,100))
        next_button_box = next_button.get_rect(topleft=next_button_pos)

        next_button_text = button_font.render("Evaluate Design", True, (0, 0, 0))
        next_button_text_box = next_button_text.get_rect(center=(NEXT_BUTTON_WIDTH//2,NEXT_BUTTON_HEIGHT//2))

        # Creating a button to return to design screen
        return_button_pos = (SCREEN_WIDTH*4/5, SCREEN_HEIGHT - NEXT_BUTTON_HEIGHT - 20)

        return_button = pygame.Surface((NEXT_BUTTON_WIDTH,NEXT_BUTTON_HEIGHT))
        return_button.fill((100,100,100))
        return_button_box = next_button.get_rect(topleft=return_button_pos)

        return_button_text = button_font.render("Return", True, (0, 0, 0))
        return_button_text_box = return_button_text.get_rect(center=(NEXT_BUTTON_WIDTH//2,NEXT_BUTTON_HEIGHT//2))

        # Creating a button to reset the drawing
        CLEAR_BUTTON_WIDTH = SCREEN_WIDTH/5
        CLEAR_BUTTON_HEIGHT = 100
        clear_button = pygame.Surface((CLEAR_BUTTON_WIDTH,CLEAR_BUTTON_HEIGHT))

        clear_button_pos = (SCREEN_WIDTH/5, SCREEN_HEIGHT - CLEAR_BUTTON_HEIGHT - 20)
        clear_button.fill((255,0,0))
        clear_button_box = clear_button.get_rect(topleft=clear_button_pos)

        clear_button_text = button_font.render("Clear Workspace",True,(0,0,0))
        clear_button_text_box = clear_button_text.get_rect(center=(CLEAR_BUTTON_WIDTH//2,CLEAR_BUTTON_HEIGHT//2))

        # Timed self-report controls and responses for this run.
        workload_slider = SevenPointSlider(300, 900, 330, button_font)
        confidence_slider = SevenPointSlider(300, 900, 525, button_font)

        report_text_color = (45, 50, 60)
        report_scales = ((workload_slider, force_font.render("How high was your mental workload?", True, report_text_color)),
                        (confidence_slider, force_font.render("How confident were you in your design?", True, report_text_color)))
        
        report_button_box = pygame.Rect(0, 0, 220, 70)
        report_button_box.midbottom = (SCREEN_WIDTH // 2, SCREEN_HEIGHT - 50)
        
        report_hint = button_font.render("Move the stylus over a number. Press the stylus button on Submit.", True, report_text_color)

        low_label = button_font.render("Very low", True, report_text_color)
        high_label = button_font.render("Very high", True, report_text_color)
        submit_label = button_font.render("Submit", True, (255, 255, 255))
        self_reports = []
        report_button_was_down = False

        # Creating a surface to display the evaluated truss system
        truss_eval = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
        eval_disp_flag = 0

        # Stores the nodes of truss end points and connecting members
        truss_nodes = []
        truss_members = []
        init_node_flag = 0

        # Creates a surface for truss environment (supports, forces, etc.) for each trial
        truss_supports = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
        truss_forces = pygame.Surface(screen.get_size(), pygame.SRCALPHA)

        support_size = 80
        support_color = (42, 83, 247)

        arrow_size = 20
        arrow_color = (255, 0, 0)
        arrow_len = 40
        arrow_th = 10
        force_text = []
        stress_text = []

        support_nodes = [((1*SCREEN_WIDTH//5 , 500), (3*SCREEN_WIDTH//5, 500)), ((250, 500), (750, 500))]
        f_nodes = [((1*SCREEN_WIDTH//5, 300),(SCREEN_WIDTH//2,100)),((200, 200), (600, 200))]
        f_scalars = [(150, -150), (100, 100)]
        f_dir = [("x", "y"), ("y", "x")]

        trial_num = 0


        # Initializing running game loop
        running_flag = True
        try:
            while running_flag == True and rclpy.ok():
                
                # Checks for events in the pygame object
                for event in pygame.event.get():    
                    if event.type == pygame.QUIT:   # If the game is quit, the loop stops running
                        running_flag = False

                match node.curr_screen_flag:       # Screen display for task
                    case 'design':     # Default truss design screen
                        # Displays all current surfaces to update graphics
                        screen.fill(screen_color)

                        screen.blit(truss_supports, (0,0))

                        screen.blit(drawn_lines, (0,0))

                        now = node.time
                        if not trial_start_flag:
                            start_time = now
                            last_report_time = now
                            trial_start_flag = True

                        design_elapsed_s = now - start_time - paused_total_ms
                        trial_countdown = max(0, trial_length - design_elapsed_s)
                        report_countdown = report_length - (now - last_report_time)

                        time_text = f"{str(trial_countdown):.2s}"
                        time_surface = timer_font.render(time_text, True, (255, 255, 255))
                        time_surface_box = time_surface.get_rect(center=(SCREEN_WIDTH//2, 50))
                        screen.blit(time_surface, time_surface_box)

                        screen.blit(truss_forces, (0,0))
                        for text, location in force_text:
                            truss_forces.blit(force_font.render(text, True, (0,0,0)), location)

                        # Displays starting nodes
                        if init_node_flag == 0:
                            # All initial truss nodes
                            init_support_nodes = support_nodes[trial_num]
                            init_f_nodes = f_nodes[trial_num]
                            init_force = f_scalars[trial_num]     # In kN
                            init_force_dir = f_dir[trial_num]
                            # Drawing initial nodes for supports
                            for init_node in init_support_nodes:
                                pygame.draw.circle(drawn_lines, line_color, init_node, line_width/2)
                                pygame.draw.circle(drawn_lines, screw_color, init_node, line_width/4)
                                # pygame.draw.polygon(truss_supports,support_color,[(init_node[0],init_node[1]-support_size),
                                #                                          (init_node[0]-support_size,init_node[1]+support_size),
                                #                                          (init_node[0]+support_size,init_node[1]+support_size)])
                                pygame.draw.rect(truss_supports, support_color,
                                                 (init_node[0]-support_size/2, init_node[1], support_size, support_size))
                                pygame.draw.circle(truss_supports, support_color, init_node, support_size/2)
                                pygame.draw.line(truss_supports,(0,0,0),
                                                 (init_node[0]-support_size, init_node[1]+support_size), 
                                                 (init_node[0]+support_size,init_node[1]+support_size),
                                                 width= 5)
                                
                                placement = np.linspace(0,support_size*2, 5)
                                gr_len = 20
                                for scale in placement:
                                    pygame.draw.line(truss_supports,(0,0,0),
                                                     (init_node[0]-support_size+scale,init_node[1]+support_size),
                                                     (init_node[0]-support_size+scale-gr_len*np.cos(np.pi/4),init_node[1]+support_size+gr_len*np.sin(np.pi/4)),
                                                     width= 5)
                                

                                truss_nodes.append(init_node)
                            
                            # Sketching forces on nodes
                            for i in range(len(init_force)):
                                pygame.draw.circle(drawn_lines, line_color, init_f_nodes[i], line_width/2)
                                pygame.draw.circle(drawn_lines, screw_color, init_f_nodes[i], line_width/4)
                                
                                if init_force_dir[i] == "x":
                                    if init_force[i] > 0:
                                        pygame.draw.polygon(truss_forces,arrow_color,[(init_f_nodes[i][0]-line_width/2,init_f_nodes[i][1]),
                                                                                    (init_f_nodes[i][0]-line_width/2-arrow_size, init_f_nodes[i][1]-line_width/2),
                                                                                    (init_f_nodes[i][0]-line_width/2-arrow_size, init_f_nodes[i][1]+line_width/2)])
                                        pygame.draw.line(truss_forces,arrow_color,
                                                         (init_f_nodes[i][0]-line_width/2-arrow_size-arrow_len,init_f_nodes[i][1]),
                                                         (init_f_nodes[i][0]-line_width/2-arrow_size,init_f_nodes[i][1]),
                                                         width=arrow_th)
                                        force_text.append((f'{abs(init_force[i])} kN',(init_f_nodes[i][0]-line_width/2-arrow_size-arrow_len,
                                                                                       init_f_nodes[i][1]-arrow_size*3)))
                                    else:
                                        pygame.draw.polygon(truss_forces,arrow_color,[(init_f_nodes[i][0]+line_width/2,init_f_nodes[i][1]),
                                                                                    (init_f_nodes[i][0]+line_width/2+arrow_size, init_f_nodes[i][1]-line_width/2),
                                                                                    (init_f_nodes[i][0]+line_width/2+arrow_size, init_f_nodes[i][1]+line_width/2)])
                                        pygame.draw.line(truss_forces,arrow_color,
                                                            (init_f_nodes[i][0]+line_width/2+arrow_size+arrow_len,init_f_nodes[i][1]),
                                                            (init_f_nodes[i][0]+line_width/2+arrow_size,init_f_nodes[i][1]),
                                                            width=arrow_th)
                                        force_text.append((f'{abs(init_force[i])} kN',(init_f_nodes[i][0]+line_width/2+arrow_size,
                                                                                        init_f_nodes[i][1]-arrow_size*3)))
                                        
                                elif init_force_dir[i] == "y":
                                    if init_force[i] > 0:
                                        pygame.draw.polygon(truss_forces,arrow_color,[(init_f_nodes[i][0],init_f_nodes[i][1]-line_width/2-arrow_len-arrow_size),
                                                                                    (init_f_nodes[i][0]-arrow_size, init_f_nodes[i][1]-line_width/2-arrow_len),
                                                                                    (init_f_nodes[i][0]+arrow_size, init_f_nodes[i][1]-line_width/2-arrow_len)])
                                        pygame.draw.line(truss_forces,arrow_color,
                                                            (init_f_nodes[i][0], init_f_nodes[i][1]-line_width/2),
                                                            (init_f_nodes[i][0],init_f_nodes[i][1]-line_width/2-arrow_len),
                                                            width=arrow_th)
                                        force_text.append((f'{abs(init_force[i])} kN',(init_f_nodes[i][0]+line_width/2+arrow_size,
                                                                                        init_f_nodes[i][1]-line_width/2-arrow_size*2)))
                                    elif init_force[i] < 0:
                                        pygame.draw.polygon(truss_forces,arrow_color,[(init_f_nodes[i][0],init_f_nodes[i][1]-line_width/2),
                                                                                    (init_f_nodes[i][0]-arrow_size, init_f_nodes[i][1]-line_width/2-arrow_size),
                                                                                    (init_f_nodes[i][0]+arrow_size, init_f_nodes[i][1]-line_width/2-arrow_size)])
                                        pygame.draw.line(truss_forces,arrow_color,
                                                            (init_f_nodes[i][0], init_f_nodes[i][1]-line_width/2-arrow_size),
                                                            (init_f_nodes[i][0],init_f_nodes[i][1]-line_width/2-arrow_size-arrow_len),
                                                            width=arrow_th)
                                        force_text.append((f'{abs(init_force[i])} kN',(init_f_nodes[i][0]+line_width/2+arrow_size,
                                                                                        init_f_nodes[i][1]-line_width/2-arrow_size*2)))

                                truss_nodes.append(init_f_nodes[i])
                            init_node_flag = 1
                        
                        screen.blit(next_button, next_button_box)
                        next_button.blit(next_button_text,next_button_text_box)

                        screen.blit(clear_button, clear_button_box)
                        clear_button.blit(clear_button_text,clear_button_text_box)
        
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
                            if draw_flag:
                                if prev_drawn_line == 0:        # The first point of the stroke is made a circle to start
                                    pygame.draw.circle(drawn_lines, line_color, start_point,line_width/2)
                                prev_drawn_line = cursor_pos.center     # Set current position as previous position to draw connecting line
                            else:    
                                for center in truss_nodes:
                                    if pygame.Vector2(cursor_pos.center).distance_squared_to(center) <= (line_width/2)**2:
                                        start_point = pygame.Vector2(center)
                                        draw_flag = True
                                        break
                            
                        elif prev_drawn_line != 0:
                            for center in truss_nodes:
                                if pygame.Vector2(cursor_pos.center).distance_squared_to(center) <= (line_width/2)**2:
                                    
                                    end_point = pygame.Vector2(center)

                                    pygame.draw.circle(drawn_lines, line_color, start_point, line_width/2)
                                    pygame.draw.line(drawn_lines, line_color, start_point, end_point, width=line_width)
                                    pygame.draw.circle(drawn_lines, line_color, end_point, line_width/2)
                                    pygame.draw.circle(drawn_lines, screw_color, start_point, line_width/4)
                                    pygame.draw.circle(drawn_lines, screw_color, end_point, line_width/4)

                                    truss_members.append((tuple(start_point), tuple(end_point)))
                                    
                                    prev_drawn_line = 0
                                    draw_flag = False

                                    break
                            else:
                                # If the end point is not an already existing node
                                end_point = pygame.Vector2(cursor_pos.center)

                                pygame.draw.circle(drawn_lines, line_color, start_point, line_width/2)
                                pygame.draw.line(drawn_lines, line_color, start_point, end_point, width=line_width)
                                pygame.draw.circle(drawn_lines, line_color, end_point, line_width/2)
                                pygame.draw.circle(drawn_lines, screw_color, start_point, line_width/4)
                                pygame.draw.circle(drawn_lines, screw_color, end_point, line_width/4)

                                truss_nodes.append(tuple(end_point))
                                truss_members.append((tuple(start_point), tuple(end_point)))
                                
                                prev_drawn_line = 0
                                draw_flag = False

                        # Checks to see if timer countdowns have reached zero
                        if report_countdown < 0:    # Send user to self report screen
                            node.curr_screen_flag = "report"
                            report_started_at = now
                        elif trial_countdown == 0:   # Send user to resting phase
                            rest_started_at = now
                            node.curr_screen_flag = "rest"


                        # Logic for buttons being pushed
                        if next_button_box.collidepoint(cursor_pos.center):     # Next page button clicked
                            # Change color of button
                            next_button.fill((155,0,0))
                            next_button.blit(next_button_text,next_button_text_box) 
                            if node.button == True:
                                # update to the next screen
                                node.curr_screen_flag = "eval"
                        else:
                            next_button.fill((255,0,0))
                            next_button.blit(next_button_text,next_button_text_box)

                        if clear_button_box.collidepoint(cursor_pos.center):    # Clear workspace button clicked
                            # Change color of button
                            clear_button.fill((75,75,75))
                            clear_button.blit(clear_button_text,clear_button_text_box)
                            if node.button == True:
                                # clear the drawn truss members and nodes (reset the initialization flag)
                                truss_members = []
                                truss_nodes = []
                                drawn_lines.fill((0,0,0,0))
                                init_node_flag = 0
                        else:
                            clear_button.fill((100,100,100))
                            clear_button.blit(clear_button_text,clear_button_text_box)

        

                    case 'eval':        # Waiting screen while eval takes place
                        # Displays all current surfaces to update graphics
                        screen.fill(screen_color)

                        stress, stress_norm = SolveTruss(user_members=truss_members,support_nodes=init_support_nodes,force_nodes=init_f_nodes,force_dir=init_force_dir,forces=init_force)

                        if stress is None or stress_norm is None:
                            node.curr_screen_flag = 'design'
                        else:
                            node.curr_screen_flag = 'disp_eval'



                    case 'disp_eval':
                        screen.fill(screen_color)

                        screen.blit(return_button, return_button_box)
                        return_button.blit(return_button_text,return_button_text_box)

                        if eval_disp_flag == 0:
                            stress_text = []
                            for i in range(len(stress)):
                                ratio = max(0.0, min(1.0, (abs(float(stress_norm[i])) - 0.25) / 0.75))

                                green = (0, 180, 0)
                                dark_red = (150, 0, 0)

                                color = tuple(
                                    round(low + ratio * (high - low))
                                    for low, high in zip(green, dark_red)
                                )

                                eval_truss = pygame.draw.line(truss_eval,
                                                    color,
                                                    truss_members[i][0],truss_members[i][1],width=line_width)
                                pygame.draw.circle(truss_eval,
                                                    color,
                                                    truss_members[i][0],line_width/2)
                                pygame.draw.circle(truss_eval,(0,0,0),truss_members[i][0],line_width/4)
                                pygame.draw.circle(truss_eval,
                                                    color,
                                                    truss_members[i][1],line_width/2)
                                pygame.draw.circle(truss_eval,(0,0,0),truss_members[i][1],line_width/4)

                                stress_text.append((f"{stress[i]/pow(10,6):.2f} MPa", eval_truss.center))
                            eval_disp_flag = 1

                        screen.blit(truss_eval, (0,0))
                        for text, location in stress_text:
                            truss_eval.blit(stress_font.render(text, True, (0,0,0)), location)

                        rclpy.spin_once(node, timeout_sec=0.0)
                        haptic_position = node.position
                        cursor_pos = pygame.Rect(haptic_position[0],haptic_position[2],cursor_len,cursor_len)
                        screen.blit(cursor, cursor_pos)



                        if return_button_box.collidepoint(cursor_pos.center):
                            # Change color of button
                            return_button.fill((155,0,0))
                            return_button.blit(return_button_text,return_button_text_box) 
                            if node.button == True:
                                # update to the next screen
                                node.curr_screen_flag = "design"
                                truss_eval.fill((0,0,0,0))
                                eval_disp_flag = 0
                        else:
                            return_button.fill((255,0,0))
                            return_button.blit(return_button_text,return_button_text_box)

                    case 'report':
                        rclpy.spin_once(node, timeout_sec=0.0)
                        haptic_position = node.position
                        cursor_pos = pygame.Rect(
                            haptic_position[0], haptic_position[2], cursor_len, cursor_len
                        )
                        
                        for slider, _ in report_scales:
                            slider.update_position(cursor_pos.center)
                        can_submit = all(slider.value is not None for slider, _ in report_scales)

                        button_pressed = node.button and not report_button_was_down
                        report_button_was_down = node.button
                        hovering_submit = report_button_box.collidepoint(cursor_pos.center)

                        screen.fill(screen_color)

                        screen.blit(report_hint, report_hint.get_rect(center=(SCREEN_WIDTH // 2, 185)))

                        for slider, prompt in report_scales:
                            screen.blit(prompt,prompt.get_rect(center=(SCREEN_WIDTH // 2, slider.y - 65)))
                            slider.draw(screen)
                            screen.blit(low_label,low_label.get_rect(midtop=(slider.left, slider.y + 50)))
                            screen.blit(high_label,high_label.get_rect(midtop=(slider.right, slider.y + 50)))

                        submit_color = (110, 110, 110)
                        if can_submit:
                            submit_color = (43, 74, 140)
                            if hovering_submit:
                                submit_color = (29, 53, 105)
                        pygame.draw.rect(screen, submit_color, report_button_box, border_radius=8)
                        screen.blit(
                            submit_label, submit_label.get_rect(center=report_button_box.center)
                        )
                        screen.blit(cursor, cursor_pos)

                        if can_submit and hovering_submit and button_pressed:
                            now = node.time
                            paused_total_ms += now - report_started_at
                            report_started_at = None
                            last_report_time = now

                            response = {
                                "workload": workload_slider.value,
                                "confidence": confidence_slider.value,
                                "trial_elapsed_ms": now - start_time - paused_total_ms,
                                "submitted_at_ms": now,
                            }
                            self_reports.append(response)
                            # node.get_logger().info(f"Self-report submitted: {response}")
                            node.curr_screen_flag = "design"

                    case 'rest':
                        screen.fill(screen_color)

                        rclpy.spin_once(node, timeout_sec=0.0)
                        now = node.time

                        rest_remaining_s = max(0.0, rest_length - (now - rest_started_at))
                        if rest_remaining_s == 0.0:
                            truss_members = []
                            truss_nodes = []
                            force_text = []
                            stress_text = []
                            truss_supports.fill((0,0,0,0))
                            truss_forces.fill((0,0,0,0))
                            drawn_lines.fill((0,0,0,0))
                            init_node_flag = 0
                            
                            rest_started_at = None
                            report_started_at = None
                            trial_start_flag = False                            
                            node.curr_screen_flag = "design"
                            trial_num = trial_num + 1

                        screen.fill(screen_color)
                        rest_text = timer_font.render("Take 30 seconds to rest.",True,(0, 0, 0))
                        screen.blit(rest_text, rest_text.get_rect(center=(SCREEN_WIDTH//2,2*SCREEN_HEIGHT//5)))
                        rest_text = timer_font.render("Please return the stylus to the device home.", True,(0,0,0))
                        screen.blit(rest_text, rest_text.get_rect(center=(SCREEN_WIDTH//2,3*SCREEN_HEIGHT//5)))

                
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