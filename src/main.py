import os
import sys
import pygame
from src.models.nav_graph import NavGraph
from src.models.robot import Robot
from src.controllers.fleet_manager import FleetManager
from src.controllers.traffic_manager import TrafficManager
from src.gui.fleet_gui import FleetGUI
from src.utils.helpers import setup_logging, ensure_directory_exists

def main():
    print("main loop")
    """Main entry point for the Fleet Management System."""
    # Setup directories
    ensure_directory_exists('logs')
    ensure_directory_exists('data')
    
    # Setup logging
    logger = setup_logging()
    logger.info("Starting Fleet Management System")
    
    # Load navigation graph
    nav_graph_path = os.path.join('data', 'nav_graph_1.json')
    try:
        nav_graph = NavGraph(nav_graph_path)
        logger.info(f"Navigation graph loaded from {nav_graph_path}")
    except Exception as e:
        logger.error(f"Failed to load navigation graph: {e}")
        print(f"Error: Failed to load navigation graph: {e}")
        return
    
    
    print("\n=== ACTUAL VERTEX POSITIONS ===")
    for vertex_id, data in nav_graph.G.nodes(data=True):
        print(f"Vertex {vertex_id}: {data['pos']} (Name: {data.get('name', '')})")
    
    # Initialize managers
    traffic_manager = TrafficManager(nav_graph)
    fleet_manager = FleetManager(nav_graph)
    print("calling fleet_gui")
    # Initialize GUI
    gui = FleetGUI(nav_graph, fleet_manager, traffic_manager)
    
    # Run GUI main loop
    logger.info("Starting GUI")
    gui.run()
    
    logger.info("Fleet Management System shutting down")

if __name__ == "__main__":
    main()