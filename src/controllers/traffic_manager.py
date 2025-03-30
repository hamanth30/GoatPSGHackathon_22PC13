import logging

class TrafficManager:
    """Manages traffic negotiation and collision avoidance between robots."""
    
    def __init__(self, nav_graph, log_file="logs/fleet_logs.txt"):
        """Initialize traffic manager.
        
        Args:
            nav_graph (NavGraph): Navigation graph object
            log_file (str): Path to log file
        """
        self.nav_graph = nav_graph
        self.lane_occupancy = {}  # Maps (start, end) to robot_id
        self.vertex_occupancy = {}  # Maps vertex_id to robot_id
        self.setup_logging(log_file)
        
    def setup_logging(self, log_file):
        """Set up logging to file and console."""
        self.logger = logging.getLogger('TrafficManager')
        self.logger.setLevel(logging.INFO)
        
        # Create file handler
        fh = logging.FileHandler(log_file)
        fh.setLevel(logging.INFO)
        
        # Create console handler
        ch = logging.StreamHandler()
        ch.setLevel(logging.INFO)
        
        # Create formatter and add to handlers
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        fh.setFormatter(formatter)
        ch.setFormatter(formatter)
        
        # Add handlers to logger
        self.logger.addHandler(fh)
        self.logger.addHandler(ch)
    
    def register_lane_usage(self, robot_id, start_vertex, end_vertex):
        """Register a robot as using a lane.
        
        Args:
            robot_id (str): ID of robot
            start_vertex (int): Source vertex ID
            end_vertex (int): Target vertex ID
        
        Returns:
            bool: True if lane is free and registration successful, False if lane is occupied
        """
        lane = (start_vertex, end_vertex)
        
        # Check if lane is already occupied by another robot
        if lane in self.lane_occupancy and self.lane_occupancy[lane] != robot_id:
            self.logger.warning(f"Lane {lane} already occupied by robot {self.lane_occupancy[lane]}")
            return False
        
        # Register lane usage
        self.lane_occupancy[lane] = robot_id
        self.vertex_occupancy[start_vertex] = robot_id
        
        self.logger.info(f"Robot {robot_id} registered for lane {lane}")
        return True
    
    def release_lane(self, robot_id, start_vertex, end_vertex):
        """Release a lane that was being used by a robot.
        
        Args:
            robot_id (str): ID of robot
            start_vertex (int): Source vertex ID
            end_vertex (int): Target vertex ID
        
        Returns:
            bool: True if release successful, False otherwise
        """
        lane = (start_vertex, end_vertex)
        
        # Check if lane is occupied by this robot
        if lane in self.lane_occupancy and self.lane_occupancy[lane] == robot_id:
            del self.lane_occupancy[lane]
            
            # Only release vertex if it's occupied by this robot
            if start_vertex in self.vertex_occupancy and self.vertex_occupancy[start_vertex] == robot_id:
                del self.vertex_occupancy[start_vertex]
                
            self.logger.info(f"Robot {robot_id} released lane {lane}")
            return True
            
        return False
    
    def is_lane_free(self, start_vertex, end_vertex, robot_id=None):
        """Check if a lane is free for a robot to use.
        
        Args:
            start_vertex (int): Source vertex ID
            end_vertex (int): Target vertex ID
            robot_id (str, optional): ID of robot making the request
        
        Returns:
            bool: True if lane is free, False otherwise
        """
        lane = (start_vertex, end_vertex)
        
        # Lane is free if it's not in the occupancy dict
        if lane not in self.lane_occupancy:
            return True
            
        # Lane is free if it's occupied by the requesting robot
        if robot_id and self.lane_occupancy[lane] == robot_id:
            return True
            
        return False
    
    def is_vertex_free(self, vertex_id, robot_id=None):
        """Check if a vertex is free for a robot to use.
        
        Args:
            vertex_id (int): Vertex ID
            robot_id (str, optional): ID of robot making the request
        
        Returns:
            bool: True if vertex is free, False otherwise
        """
        # Vertex is free if it's not in the occupancy dict
        if vertex_id not in self.vertex_occupancy:
            return True
            
        # Vertex is free if it's occupied by the requesting robot
        if robot_id and self.vertex_occupancy[vertex_id] == robot_id:
            return True
            
        return False
    
    def get_lane_occupancy(self):
        """Get the current lane occupancy.
        
        Returns:
            dict: Dict mapping lanes to robot IDs
        """
        return self.lane_occupancy
    
    def get_vertex_occupancy(self):
        """Get the current vertex occupancy.
        
        Returns:
            dict: Dict mapping vertices to robot IDs
        """
        return self.vertex_occupancy
    
    def reset(self):
        """Reset all traffic data."""
        self.lane_occupancy = {}
        self.vertex_occupancy = {}
        self.logger.info("Traffic manager reset")
    
    def update_from_robot_positions(self, robots):
        """Update traffic data based on robot positions.
        
        Args:
            robots (dict): Dict mapping robot IDs to Robot objects
        """
        # Clear existing data
        self.lane_occupancy = {}
        self.vertex_occupancy = {}
        
        # Update based on current robot positions
        for robot_id, robot in robots.items():
            if robot.current_vertex is not None:
                self.vertex_occupancy[robot.current_vertex] = robot_id
                
                if robot.next_vertex is not None:
                    self.lane_occupancy[(robot.current_vertex, robot.next_vertex)] = robot_id