import time
import os
import logging

def ensure_directory_exists(directory):
    """Ensure that a directory exists, creating it if necessary.
    
    Args:
        directory (str): Directory path
    """
    if not os.path.exists(directory):
        os.makedirs(directory)

def setup_logging(log_file="logs/fleet_logs.txt"):
    """Set up logging configuration.
    
    Args:
        log_file (str): Path to log file
        
    Returns:
        logging.Logger: Configured logger
    """
    # Ensure logs directory exists
    log_dir = os.path.dirname(log_file)
    ensure_directory_exists(log_dir)
    
    # Create logger
    logger = logging.getLogger('FleetManagement')
    logger.setLevel(logging.INFO)
    
    # Create file handler
    fh = logging.FileHandler(log_file)
    fh.setLevel(logging.INFO)
    
    # Create console handler
    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)
    
    # Create formatter
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    fh.setFormatter(formatter)
    ch.setFormatter(formatter)
    
    # Add handlers to logger
    logger.addHandler(fh)
    logger.addHandler(ch)
    
    return logger

def calculate_distance(point1, point2):
    """Calculate Euclidean distance between two points.
    
    Args:
        point1 (tuple): (x, y) coordinates of first point
        point2 (tuple): (x, y) coordinates of second point
        
    Returns:
        float: Distance between points
    """
    return ((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)**0.5

def generate_unique_id(prefix=""):
    """Generate a unique ID based on timestamp.
    
    Args:
        prefix (str): Optional prefix for ID
        
    Returns:
        str: Unique ID
    """
    timestamp = int(time.time() * 1000)
    return f"{prefix}{timestamp}"