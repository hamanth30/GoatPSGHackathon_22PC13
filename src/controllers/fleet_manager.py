import time
import logging
from collections import defaultdict

class FleetManager:
    """Manager for coordinating multiple robots in the system."""
    
    def __init__(self, nav_graph, log_file="logs/fleet_logs.txt"):
        """Initialize fleet manager.
        
        Args:
            nav_graph (NavGraph): Navigation graph object
            log_file (str): Path to log file
        """
        self.nav_graph = nav_graph
        self.robots = {}  # Maps robot IDs to Robot objects
        self.tasks = {}   # Maps robot IDs to their current tasks
        self.setup_logging(log_file)
        
    def setup_logging(self, log_file):
        """Set up logging to file and console."""
        self.logger = logging.getLogger('FleetManager')
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
        
    def add_robot(self, robot):
        """Add a robot to the fleet.
        
        Args:
            robot (Robot): Robot object to add
        """
        self.robots[robot.id] = robot
        self.logger.info(f"Robot {robot.id} added at vertex {robot.current_vertex}")
        
    def remove_robot(self, robot_id):
        """Remove a robot from the fleet.
        
        Args:
            robot_id (str): ID of robot to remove
        
        Returns:
            bool: True if robot was removed, False otherwise
        """
        if robot_id in self.robots:
            del self.robots[robot_id]
            if robot_id in self.tasks:
                del self.tasks[robot_id]
            self.logger.info(f"Robot {robot_id} removed from fleet")
            return True
        return False
    
    def assign_task(self, robot_id, destination_vertex):
        """Assign a navigation task to a robot.
        
        Args:
            robot_id (str): ID of robot to assign task to
            destination_vertex (int): Vertex ID for the destination
        
        Returns:
            bool: True if task was assigned, False otherwise
        """
        if robot_id not in self.robots:
            self.logger.warning(f"Cannot assign task: Robot {robot_id} not found")
            return False
        
        robot = self.robots[robot_id]
        
        if robot.current_vertex == destination_vertex:
            self.logger.info(f"Robot {robot_id} already at destination {destination_vertex}")
            return True
        
        # Calculate path
        path = self.nav_graph.get_path(robot.current_vertex, destination_vertex)
        
        if not path:
            self.logger.warning(f"No path found from {robot.current_vertex} to {destination_vertex}")
            return False
        
        # Assign the task
        robot.set_destination(destination_vertex, path)
        self.tasks[robot_id] = {
            "destination": destination_vertex,
            "path": path,
            "start_time": time.time()
        }
        
        self.logger.info(f"Task assigned to robot {robot_id}: Navigate from {robot.current_vertex} to {destination_vertex}")
        return True
    
    def cancel_task(self, robot_id):
        """Cancel a robot's current task.
        
        Args:
            robot_id (str): ID of robot
        
        Returns:
            bool: True if task was cancelled, False otherwise
        """
        if robot_id not in self.robots:
            return False
        
        robot = self.robots[robot_id]
        robot.cancel_task()
        
        if robot_id in self.tasks:
            del self.tasks[robot_id]
            
        self.logger.info(f"Task cancelled for robot {robot_id}")
        return True
    
    def update_robots(self, dt, traffic_manager):
        """Update all robots in the fleet.
        
        Args:
            dt (float): Time delta in seconds
            traffic_manager (TrafficManager): Traffic manager object
        
        Returns:
            dict: Updated robot states
        """
        vertex_positions = {v_id: data['pos'] for v_id, data in self.nav_graph.G.nodes(data=True)}
        lane_occupancy = traffic_manager.get_lane_occupancy()
        
        updates = {}
        for robot_id, robot in self.robots.items():
            pre_state = robot.state
            pre_vertex = robot.current_vertex
            
            # Update robot
            update_info = robot.update(dt, vertex_positions, lane_occupancy)
            updates[robot_id] = update_info
            
            # Log state changes
            if update_info.get('state') != pre_state:
                self.logger.info(f"Robot {robot_id} state changed: {pre_state} -> {update_info.get('state')}")
            
            # Log vertex changes
            if 'current_vertex' in update_info and update_info['current_vertex'] != pre_vertex:
                self.logger.info(f"Robot {robot_id} moved from vertex {pre_vertex} to {update_info['current_vertex']}")
                
                # Check if destination reached
                if robot.destination == update_info['current_vertex']:
                    self.logger.info(f"Robot {robot_id} reached destination {robot.destination}")
                    
                    # Check if it's a charging station
                    if update_info['current_vertex'] in self.nav_graph.get_chargers() and robot.battery < 50:
                        robot.start_charging()
                        self.logger.info(f"Robot {robot_id} started charging at vertex {update_info['current_vertex']}")
            
            # Update traffic manager with robot's current position
            if robot.current_vertex is not None and robot.next_vertex is not None:
                traffic_manager.register_lane_usage(robot_id, robot.current_vertex, robot.next_vertex)
            
        return updates
    
    def get_robot_statuses(self):
        """Get status information for all robots.
        
        Returns:
            dict: Dict mapping robot IDs to status dicts
        """
        return {robot_id: robot.get_info() for robot_id, robot in self.robots.items()}
    
    def get_robot_by_position(self, position, threshold=20.0):
        """Find robot at a given position.
        
        Args:
            position (tuple): (x, y) coordinates
            threshold (float): Maximum distance to consider
        
        Returns:
            str: Robot ID or None if no robot found
        """
        for robot_id, robot in self.robots.items():
            dx = robot.position[0] - position[0]
            dy = robot.position[1] - position[1]
            distance = (dx**2 + dy**2)**0.5
            if distance <= threshold:
                return robot_id
        return None