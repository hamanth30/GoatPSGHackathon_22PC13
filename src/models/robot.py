import time
import uuid
import numpy as np

class Robot:
    """Class representing a robot in the fleet management system."""
    
    STATES = {
        'IDLE': 'idle',
        'MOVING': 'moving',
        'WAITING': 'waiting',
        'CHARGING': 'charging',
        'COMPLETED': 'completed'
    }
    
    COLORS = [
        (255, 0, 0),     # Red
        (0, 255, 0),     # Green
        (0, 0, 255),     # Blue
        (255, 255, 0),   # Yellow
        (255, 0, 255),   # Magenta
        (0, 255, 255),   # Cyan
        (255, 165, 0),   # Orange
        (128, 0, 128),   # Purple
        (0, 128, 0),     # Dark Green
        (139, 69, 19)    # Brown
    ]
    
    def __init__(self, robot_id=None, position=(0, 0), current_vertex=None):
        """Initialize a robot.
        
        Args:
            robot_id (str, optional): Unique identifier for the robot
            position (tuple): (x, y) coordinates
            current_vertex (int): Vertex ID where the robot is located
        """
        self.id = robot_id if robot_id else str(uuid.uuid4())[:8]
        self.position = position
        self.current_vertex = current_vertex
        self.destination = None
        self.path = []
        self.next_vertex = None
        self.state = self.STATES['IDLE']
        self.color = self._generate_color()
        self.speed = 0.05  # Units per second
        self.battery = 100  # Battery percentage
        self.last_update_time = time.time()
        self.animation = None  # Must exist
        self.destination = None  # Must exist
        
    def _generate_color(self):
        """Generate a unique color for the robot based on its ID."""
        color_index = hash(self.id) % len(self.COLORS)
        return self.COLORS[color_index]
    
    def set_destination(self, destination, path):
        """Set a new destination and path for the robot.
        
        Args:
            destination (int): Target vertex ID
            path (list): List of vertex IDs forming the path
        """
        self.destination = destination
        self.path = path
        if len(path) > 1:
            self.next_vertex = path[1]  # First element is current vertex
            self.state = self.STATES['MOVING']
        elif len(path) == 1:
            self.state = self.STATES['COMPLETED']
        else:
            self.state = self.STATES['IDLE']
    
    def update(self, dt, vertex_positions, current_lane_occupancy):
        """Update robot state and position.
        
        Args:
            dt (float): Time delta in seconds
            vertex_positions (dict): Dict mapping vertex IDs to positions
            current_lane_occupancy (dict): Dict of occupied lanes
        
        Returns:
            dict: Updated state information
        """
        self.last_update_time = time.time()
        
        # Decrease battery slightly
        self.battery = max(0, self.battery - 0.01 * dt)
        
        if self.state == self.STATES['IDLE'] or self.state == self.STATES['COMPLETED']:
            return {"state": self.state}
        
        if self.state == self.STATES['CHARGING']:
            # Increase battery when charging
            self.battery = min(100, self.battery + 0.1 * dt)
            if self.battery >= 100:
                self.state = self.STATES['IDLE']
            return {"state": self.state, "battery": self.battery}
        
        if self.state == self.STATES['WAITING']:
            # Check if lane is free now
            lane = (self.current_vertex, self.next_vertex)
            if lane not in current_lane_occupancy or current_lane_occupancy[lane] == self.id:
                self.state = self.STATES['MOVING']
            return {"state": self.state}
        
        if self.state == self.STATES['MOVING']:
            if not self.next_vertex:
                if len(self.path) > 0:
                    self.path.pop(0)  # Remove current vertex
                    if self.path:
                        self.next_vertex = self.path[0]
                    else:
                        self.state = self.STATES['COMPLETED']
                        return {"state": self.state}
                else:
                    self.state = self.STATES['COMPLETED']
                    return {"state": self.state}
            
            # Check if lane is occupied by another robot
            lane = (self.current_vertex, self.next_vertex)
            if lane in current_lane_occupancy and current_lane_occupancy[lane] != self.id:
                self.state = self.STATES['WAITING']
                return {"state": self.state}
            
            # Move toward next vertex
            current_pos = np.array(self.position)
            next_pos = np.array(vertex_positions[self.next_vertex])
            
            direction = next_pos - current_pos
            distance = np.linalg.norm(direction)
            
            # Check if we've reached the next vertex
            if distance < self.speed * dt:
                self.position = tuple(next_pos)
                self.current_vertex = self.next_vertex
                
                # Update path and next vertex
                self.path.pop(0)
                if self.path:
                    self.next_vertex = self.path[0]
                else:
                    self.next_vertex = None
                    self.state = self.STATES['COMPLETED']
                
                return {"state": self.state, "position": self.position, 
                       "current_vertex": self.current_vertex}
            
            # Move along path
            if distance > 0:
                normalized_direction = direction / distance
                movement = normalized_direction * self.speed * dt
                self.position = tuple(current_pos + movement)
            
            return {"state": self.state, "position": self.position}
        
        return {"state": self.state}
    
    def start_charging(self):
        """Start charging the robot."""
        self.state = self.STATES['CHARGING']
    
    def cancel_task(self):
        """Cancel the current task and set robot to idle."""
        self.destination = None
        self.path = []
        self.next_vertex = None
        self.state = self.STATES['IDLE']

    def get_info(self):
        """Get a dictionary with robot information."""
        return {
            "id": self.id,
            "position": self.position,
            "current_vertex": self.current_vertex,
            "destination": self.destination,
            "state": self.state,
            "battery": self.battery
        }