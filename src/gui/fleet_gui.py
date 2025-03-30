import pygame
import sys
import math
import numpy as np
import os
from src.models.robot import Robot

class FleetGUI:
    """GUI for displaying and interacting with the fleet management system."""
    
    # Constants
    BACKGROUND_COLOR = (240, 240, 240)
    VERTEX_COLOR = (100, 100, 100)
    VERTEX_RADIUS = 10
    CHARGER_COLOR = (50, 200, 50)
    LANE_COLOR = (150, 150, 150)
    LANE_WIDTH = 2
    ROBOT_RADIUS = 12
    SELECTED_COLOR = (255, 255, 0)
    FONT_COLOR = (50, 50, 50)
              
    # Robot state colors
    STATE_COLORS = {
        'idle': (100, 100, 200),
        'moving': (0, 200, 0),
        'waiting': (255, 165, 0),
        'charging': (0, 200, 200),
        'completed': (200, 200, 0)
    }
    
    def __init__(self, nav_graph, fleet_manager, traffic_manager, width=1024, height=768):
        """Initialize GUI.
        
        Args:
            nav_graph (NavGraph): Navigation graph
            fleet_manager (FleetManager): Fleet manager
            traffic_manager (TrafficManager): Traffic manager
            width (int): Window width
            height (int): Window height
        """
        print("intializing git")
        pygame.init()
        pygame.font.init()
        
        self.width = width
        self.height = height
        self.nav_graph = nav_graph
        self.fleet_manager = fleet_manager
        self.traffic_manager = traffic_manager
        
        self.awaiting_destination_for = None  # Track robot needing destination
        self.speed_multiplier = 2.0  # 2x speed boost
        self.destination_button = None
        
        self.robot_img = self._load_robot_image()

        
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Robot Fleet Management System")
        
        self.font = pygame.font.SysFont('Arial', 14)
        self.large_font = pygame.font.SysFont('Arial', 18, bold=True)
        
        self.clock = pygame.time.Clock()
        self.running = True
        self.last_update_time = pygame.time.get_ticks()
        
        # Interaction state
        self.selected_robot = None
        self.selected_vertex = None
        
        # Setup view transformation
        self.offset_x = 0
        self.offset_y = 0
        self.scale = 1.0
        self.compute_transform()
        
        # Notification system
        self.notifications = []
        self.notification_duration = 3000  # ms
        
    def _load_robot_image(self):
        """Load and scale robot image."""
        try:
            # Replace with your actual robot image path
            img_path = os.path.join('assets', 'robo.jpg')
            img = pygame.image.load(img_path)
            return pygame.transform.scale(img, (self.ROBOT_RADIUS*2, self.ROBOT_RADIUS*2))
        except:
            # Fallback to circle if image missing
            return None
        
    def compute_transform(self):
        """Compute view transformation to fit the graph in the window."""
        # Get min and max coordinates
        positions = [data['pos'] for _, data in self.nav_graph.G.nodes(data=True)]
        if not positions:
            return
            
        x_coords, y_coords = zip(*positions)
        min_x, max_x = min(x_coords), max(x_coords)
        min_y, max_y = min(y_coords), max(y_coords)
        
        # Add margin
        margin = 50
        graph_width = max_x - min_x
        graph_height = max_y - min_y
        
        # Calculate scale to fit
        scale_x = (self.width - 2 * margin) / graph_width if graph_width > 0 else 1.0
        scale_y = (self.height - 2 * margin) / graph_height if graph_height > 0 else 1.0
        self.scale = min(scale_x, scale_y)
        
        # Center the graph
        self.offset_x = self.width / 2 - (min_x + max_x) / 2 * self.scale
        self.offset_y = self.height / 2 - (min_y + max_y) / 2 * self.scale
        
    def transform_point(self, graph_point):
        """Convert graph coordinates to screen coordinates"""
        try:
            x, y = graph_point
            screen_x = x * self.scale + self.offset_x
            screen_y = y * self.scale + self.offset_y
            return (screen_x, screen_y)
        except Exception as e:
            print(f"Transform error: {e}")
            return (0, 0)
        
    def inverse_transform_point(self, screen_point):
        """Convert screen coordinates to graph coordinates"""
        try:
            screen_x, screen_y = screen_point
            x = (screen_x - self.offset_x) / self.scale
            y = (screen_y - self.offset_y) / self.scale
            return (x, y)
        except Exception as e:
            print(f"Inverse transform error: {e}")
            return (0, 0)
    
    def add_notification(self, message, color=(255, 0, 0)):
        """Add a notification message to display."""
        self.notifications.append({
            'message': message,
            'color': color,
            'timestamp': pygame.time.get_ticks()
        })
        
    def update_notifications(self):
        """Update notifications and remove expired ones."""
        current_time = pygame.time.get_ticks()
        self.notifications = [n for n in self.notifications 
                             if current_time - n['timestamp'] < self.notification_duration]
    
    def draw_notifications(self):
        """Draw notifications on screen."""
        y_offset = 10
        for notification in self.notifications:
            text_surface = self.large_font.render(notification['message'], True, notification['color'])
            text_rect = text_surface.get_rect(center=(self.width / 2, y_offset + text_surface.get_height() / 2))
            self.screen.blit(text_surface, text_rect)  # Fix here
            y_offset += text_surface.get_height() + 5

    
    def draw_graph(self):
        """Draw the navigation graph."""
        # Draw lanes first (so they're behind vertices)
        for u, v, data in self.nav_graph.get_lanes():
            start_pos = self.nav_graph.get_vertex_position(u)
            end_pos = self.nav_graph.get_vertex_position(v)
            
            screen_start = self.transform_point(start_pos)
            screen_end = self.transform_point(end_pos)
            
            # Draw lane line
            pygame.draw.line(self.screen, self.LANE_COLOR, screen_start, screen_end, self.LANE_WIDTH)
            
            # Draw direction indicator (arrow)
            dx = screen_end[0] - screen_start[0]
            dy = screen_end[1] - screen_start[1]
            length = math.sqrt(dx*dx + dy*dy)
            
            if length > 0:
                # Calculate position for arrow (70% along the lane)
                arrow_pos_x = screen_start[0] + dx * 0.7
                arrow_pos_y = screen_start[1] + dy * 0.7
                
                # Calculate normalized direction vector
                dir_x, dir_y = dx/length, dy/length
                
                # Calculate perpendicular vector
                perp_x, perp_y = -dir_y, dir_x
                
                # Calculate arrow points
                arrow_size = 5
                
                # Arrow tip
                tip_x = arrow_pos_x + dir_x * arrow_size
                tip_y = arrow_pos_y + dir_y * arrow_size
                
                # Arrow base left
                left_x = arrow_pos_x - dir_x * arrow_size + perp_x * arrow_size
                left_y = arrow_pos_y - dir_y * arrow_size + perp_y * arrow_size
                
                # Arrow base right
                right_x = arrow_pos_x - dir_x * arrow_size - perp_x * arrow_size
                right_y = arrow_pos_y - dir_y * arrow_size - perp_y * arrow_size
                
                # Draw arrow
                pygame.draw.polygon(self.screen, self.LANE_COLOR, 
                                   [(tip_x, tip_y), (left_x, left_y), (right_x, right_y)])
        
        # Draw vertices
        for vertex_id, data in self.nav_graph.get_vertices():
            pos = data['pos']
            screen_pos = self.transform_point(pos)
            
            is_charger = data.get('is_charger', False)
            color = self.CHARGER_COLOR if is_charger else self.VERTEX_COLOR
            
            # Highlight selected vertex
            if vertex_id == self.selected_vertex:
                pygame.draw.circle(self.screen, self.SELECTED_COLOR, screen_pos, 
                                  self.VERTEX_RADIUS + 3)
            
            # Draw vertex
            pygame.draw.circle(self.screen, color, screen_pos, self.VERTEX_RADIUS)
            
            # Draw vertex name
            name = data.get('name', f'v{vertex_id}')
            if name:
                text_surface = self.font.render(name, True, self.FONT_COLOR)
                self.screen.blit(text_surface, (screen_pos[0] - text_surface.get_width() / 2, 
                                               screen_pos[1] - 30))
                
                
    def _compute_transform(self):
        """Compute view transformation."""
        positions = [data['pos'] for _, data in self.nav_graph.G.nodes(data=True)]
        if not positions:
            return
            
        x_coords, y_coords = zip(*positions)
        margin = 50
        graph_width = max(x_coords) - min(x_coords)
        graph_height = max(y_coords) - min(y_coords)
        
        scale_x = (self.width - 2*margin) / graph_width if graph_width > 0 else 1.0
        scale_y = (self.height - 2*margin) / graph_height if graph_height > 0 else 1.0
        self.scale = min(scale_x, scale_y)
        
        self.offset_x = self.width/2 - (min(x_coords) + max(x_coords))/2 * self.scale
        self.offset_y = self.height/2 - (min(y_coords) + max(y_coords))/2 * self.scale
        
    
    def _draw_battery(self, position, level):
        """Draw battery level indicator."""
        width, height = 20, 4
        border = 1
        x, y = position[0] - width/2, position[1] + self.ROBOT_RADIUS + 5
        
        # Background
        pygame.draw.rect(self.screen, (50, 50, 50), (x, y, width, height))
        
        # Level
        color = (0, 200, 0) if level > 20 else (200, 0, 0)
        fill_width = max(0, min(width * level / 100, width))
        pygame.draw.rect(self.screen, color, (x, y, fill_width, height))
        
        
        
    def draw_robots(self):
        """Draw robots with images and numbered names."""
        for robot_id, robot in self.fleet_manager.robots.items():
            screen_pos = self.transform_point(robot.position)
            
            # Draw selection highlight
            if robot_id == self.selected_robot:
                pygame.draw.circle(self.screen, self.SELECTED_COLOR, screen_pos, self.ROBOT_RADIUS+3)
                self._draw_destination_button(screen_pos)
                
            
            # Draw robot (image or fallback circle)
            if self.robot_img:
                img_rect = self.robot_img.get_rect(center=screen_pos)
                self.screen.blit(self.robot_img, img_rect)
            else:
                pygame.draw.circle(self.screen, robot.color, screen_pos, self.ROBOT_RADIUS)
            
            # Draw robot name (R1, R2, etc.)
            name_text = self.font.render(robot_id, True, (255, 255, 255))
            name_rect = name_text.get_rect(center=screen_pos)
            self.screen.blit(name_text, name_rect)
            
            # Draw battery indicator
            self._draw_battery(screen_pos, robot.battery)

    def _draw_battery(self, position, level):
        """Draw battery level indicator."""
        width, height = 20, 4
        border = 1
        x, y = position[0] - width/2, position[1] + self.ROBOT_RADIUS + 5
        
        # Background
        pygame.draw.rect(self.screen, (50, 50, 50), (x, y, width, height))
        
        # Level
        color = (0, 200, 0) if level > 20 else (200, 0, 0)
        fill_width = max(0, min(width * level / 100, width))
        pygame.draw.rect(self.screen, color, (x, y, fill_width, height))
        
    
    def draw_status_panel(self):
        """Draw status panel with robot information."""
        panel_width = 250
        panel_height = self.height
        panel_x = self.width - panel_width
        
        # Draw panel background
        pygame.draw.rect(self.screen, (220, 220, 220), 
                        (panel_x, 0, panel_width, panel_height))
        
        # Draw title
        title = self.large_font.render("Fleet Status", True, (0, 0, 0))
        self.screen.blit(title, (panel_x + 10, 10))
        
        # Draw selected robot info
        if self.selected_robot:
            y_offset = 50
            robot = self.fleet_manager.robots.get(self.selected_robot)
            
            if robot:
                info = robot.get_info()
                
                # Draw robot ID
                text = self.large_font.render(f"Robot: {info['id']}", True, (0, 0, 0))
                self.screen.blit(text, (panel_x + 10, y_offset))
                y_offset += 30
                
                # Draw state
                text = self.font.render(f"State: {info['state'].upper()}", True, (0, 0, 0))
                self.screen.blit(text, (panel_x + 10, y_offset))
                y_offset += 20
                
                # Draw battery
                text = self.font.render(f"Battery: {info['battery']:.1f}%", True, (0, 0, 0))
                self.screen.blit(text, (panel_x + 10, y_offset))
                y_offset += 20
                
                # Draw current position
                text = self.font.render(f"Position: {info['current_vertex']}", True, (0, 0, 0))
                self.screen.blit(text, (panel_x + 10, y_offset))
                y_offset += 20
                
                # Draw destination
                if info['destination'] is not None:
                    text = self.font.render(f"Destination: {info['destination']}", True, (0, 0, 0))
                    self.screen.blit(text, (panel_x + 10, y_offset))
                    y_offset += 20
        
        # Draw fleet summary
        y_offset = 200
        text = self.large_font.render("Fleet Summary", True, (0, 0, 0))
        self.screen.blit(text, (panel_x + 10, y_offset))
        y_offset += 30
        
        # Count robots by state
        states = {}
        for robot_id, robot in self.fleet_manager.robots.items():
            state = robot.state
            states[state] = states.get(state, 0) + 1
        
        # Display counts
        for state, count in states.items():
            text = self.font.render(f"{state.upper()}: {count}", True, (0, 0, 0))
            self.screen.blit(text, (panel_x + 10, y_offset))
            y_offset += 20
        
        # Draw instructions
        y_offset = self.height - 150
        text = self.large_font.render("Instructions", True, (0, 0, 0))
        self.screen.blit(text, (panel_x + 10, y_offset))
        y_offset += 30
        
        instructions = [
            "Click vertex: Spawn robot or select destination",
            "Click robot: Select robot",
            "ESC: Cancel selection",
            "Delete: Remove selected robot"
        ]
        
        for instruction in instructions:
            text = self.font.render(instruction, True, (0, 0, 0))
            self.screen.blit(text, (panel_x + 10, y_offset))
            y_offset += 20
    
    ''' def handle_events(self):
        """Handle pygame events with fixed vertex clicking."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left mouse button
                    mouse_pos = pygame.mouse.get_pos()
                    
                    # Check if click is in the main view (excluding status panel)
                    if mouse_pos[0] < self.width - 250:
                        graph_pos = self.inverse_transform_point(mouse_pos)
                        
                        # 1. Check if clicked on an existing robot
                        robot_id = self.fleet_manager.get_robot_by_position(graph_pos)
                        if robot_id:
                            self.selected_robot = robot_id
                            continue
                        
                        # 2. Check for vertex click (INCREASED THRESHOLD to 2.0)
                        vertex_id = self.nav_graph.get_vertex_id_by_position(graph_pos, threshold=2.0)
                        if vertex_id is not None:
                            self._handle_vertex_click(vertex_id) 
    '''
    
    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mouse_pos = pygame.mouse.get_pos()
                if mouse_pos[0] < self.width - 250:  # Exclude panel
                    # Convert to graph coordinates
                    graph_pos = self.inverse_transform_point(mouse_pos)
                    print(f"\nClicked at screen: {mouse_pos} -> graph: {graph_pos}")
                    
                    # Find nearest vertex
                    vertex_id = self.nav_graph.get_vertex_id_by_position(graph_pos, threshold=2.0)
                    print(f"Found vertex_id: {vertex_id}")
                    
                    if vertex_id is not None:
                        self._handle_vertex_click(vertex_id)
                    else:
                        print("No vertex found near click position")
                        self.add_notification("No vertex selected", (255, 0, 0))
    
    def _spawn_or_assign_task(self, vertex_id):
        """Either spawn new robot or assign task to selected robot"""
        if self.selected_robot:
            # Assign task to selected robot
            if self.fleet_manager.assign_task(self.selected_robot, vertex_id):
                self.add_notification(f"Task assigned to {self.selected_robot}", (0, 255, 0))
        else:
            # Spawn new robot with numbered name
            robot_name = f"R{len(self.fleet_manager.robots) + 1}"
            vertex_pos = self.nav_graph.get_vertex_position(vertex_id)
            
            try:
                new_robot = Robot(
                    robot_id=robot_name,
                    position=vertex_pos,
                    current_vertex=vertex_id
                )
                self.fleet_manager.add_robot(new_robot)
                self.add_notification(f"{robot_name} spawned at Vertex {vertex_id}", (0, 255, 0))
            except Exception as e:
                self.add_notification(f"Spawn failed: {str(e)}", (255, 0, 0))                        

    
    
    def _handle_vertex_click(self, mouse_pos):
        """Handle vertex clicks with destination selection mode"""
        graph_pos = self.inverse_transform_point(mouse_pos)
        vertex_id = self.nav_graph.get_vertex_id_by_position(graph_pos, threshold=2.0)

        # Check if clicking destination button
        if self.destination_button and self.destination_button.collidepoint(mouse_pos):
            self.awaiting_destination_for = self.selected_robot
            self.add_notification("Select destination vertex", (100, 255, 100))
            return

        # Validate vertex selection
        if vertex_id is None:
            return

        # If in destination selection mode
        if self.awaiting_destination_for:
            robot = self.fleet_manager.robots.get(self.awaiting_destination_for)
            if robot and robot.current_vertex != vertex_id:
                path = self.nav_graph.get_path(robot.current_vertex, vertex_id)
                if path:
                    self._animate_path(robot, path)
                    self.add_notification(f"{robot.id} moving to vertex {vertex_id}", (100, 255, 100))
            self.awaiting_destination_for = None
            return

        # Spawn new robot
        if not any(robot.current_vertex == vertex_id for robot in self.fleet_manager.robots.values()):
            robot_name = f"R{len(self.fleet_manager.robots) + 1}"
            new_robot = Robot(
                robot_id=robot_name,
                position=self.nav_graph.get_vertex_position(vertex_id),
                current_vertex=vertex_id
            )
            self.fleet_manager.add_robot(new_robot)
            self.selected_robot = new_robot.id
            self._create_destination_button(new_robot.position) 
                
    def _create_destination_button(self, robot_pos):
        """Create clickable destination button near robot"""
        button_size = 30
        screen_pos = self.transform_point(robot_pos)
        self.destination_button = pygame.Rect(
            screen_pos[0] + 40,  # Position to right of robot
            screen_pos[1] - 15,
            button_size,
            button_size
        )
        
    def _animate_path(self, robot, path):
        """2x faster path animation"""
        if not path or len(path) < 2:
            return

        # Get all positions along path
        positions = [self.nav_graph.get_vertex_position(vid) for vid in path]
        
        # Create animation points with fewer steps for faster movement
        animation_points = []
        for i in range(len(positions)-1):
            start = positions[i]
            end = positions[i+1]
            steps = int(math.dist(start, end) / 0.2)  # Reduced steps for speed
            for t in np.linspace(0, 1, max(3, steps)):
                x = start[0] + t * (end[0] - start[0])
                y = start[1] + t * (end[1] - start[1])
                animation_points.append((x, y))

        # Store animation data with increased speed
        robot.animation = {
            'points': animation_points,
            'current_index': 0,
            'speed': 1.0 * self.speed_multiplier  # 2x speed applied
        }
                
    def _draw_destination_button(self, screen_pos):
        """Draw the destination selection button"""
        if not self.destination_button:
            return
            
        pygame.draw.rect(self.screen, (0, 200, 0), self.destination_button, 0, 5)
        text = self.font.render("→", True, (255, 255, 255))
        text_rect = text.get_rect(center=self.destination_button.center)
        self.screen.blit(text, text_rect)
            
    def update(self):
        """Update robot animations"""
        current_time = pygame.time.get_ticks()
        dt = (current_time - self.last_update_time) / 1000.0
        self.last_update_time = current_time

        for robot in self.fleet_manager.robots.values():
            if hasattr(robot, 'animation'):
                anim = robot.animation
                anim['current_index'] = min(
                    anim['current_index'] + anim['speed'] * dt * 60,
                    len(anim['points']) - 1
                )
                robot.position = anim['points'][int(anim['current_index'])]
                
                # Check if animation complete
                if anim['current_index'] >= len(anim['points']) - 1:
                    robot.current_vertex = robot.destination
                    del robot.animation
    
    def render(self):
        """Render the GUI."""
        # Clear screen
        self.screen.fill(self.BACKGROUND_COLOR)
        
        # Draw navigation graph
        self.draw_graph()
        
        # Draw robots
        self.draw_robots()
        
        # Draw status panel
        self.draw_status_panel()
        
        # Draw notifications
        self.draw_notifications()
        
        # Update display
        pygame.display.flip()
    
    def run(self):
        """Main GUI loop with complete functionality."""
        clock = pygame.time.Clock()
        self.running = True
        
        while self.running:
            # 1. Handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:  # Left click
                        mouse_pos = pygame.mouse.get_pos()
                        # Only handle clicks in main view area (not status panel)
                        if mouse_pos[0] < self.width - 250:
                            self._handle_vertex_click(mouse_pos)
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        self.selected_robot = None
                    elif event.key == pygame.K_DELETE and self.selected_robot:
                        self.fleet_manager.remove_robot(self.selected_robot)
                        self.selected_robot = None
            
            # 2. Update game state
            current_time = pygame.time.get_ticks()
            dt = (current_time - self.last_update_time) / 1000.0  # Delta time in seconds
            self.last_update_time = current_time
            self.fleet_manager.update_robots(dt, self.traffic_manager)
            
            # 3. Update notifications
            self.notifications = [n for n in self.notifications 
                            if current_time - n['timestamp'] < self.notification_duration]
            
            # 4. Render everything
            self.screen.fill(self.BACKGROUND_COLOR)
            self.draw_graph()
            self.draw_robots()
            self.draw_status_panel()
            self.draw_notifications()
            pygame.display.flip()
            
            # 5. Maintain 60 FPS
            clock.tick(60)
        
        pygame.quit()
            
        
