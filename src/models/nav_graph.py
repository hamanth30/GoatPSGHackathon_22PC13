import json
import networkx as nx
import numpy as np
import math

class NavGraph:
    """Class to handle the navigation graph representation and operations."""
    
    def __init__(self, graph_file):
        """Initialize NavGraph with a json file containing graph definition.
        
        Args:
            graph_file (str): Path to the json file containing nav_graph
        """
        self.graph_file = graph_file
        self.G = nx.DiGraph()  # Directed graph for one-way lanes
        self.vertices = []
        self.lanes = []
        self.chargers = []
        self.vertex_names = {}
        self.load_graph()
        
    def load_graph(self):
        """Load graph data from json file."""
        try:
            with open(self.graph_file, 'r') as file:
                data = json.load(file)
                
            # Get the first level in the graph
            level_key = list(data['levels'].keys())[0]
            level_data = data['levels'][level_key]
            
            # Process vertices
            self.vertices = level_data['vertices']
            for i, vertex in enumerate(self.vertices):
                x, y, attrs = vertex
                name = attrs.get('name', f'v{i}')
                is_charger = attrs.get('is_charger', False)
                
                # Add vertex to graph
                self.G.add_node(i, pos=(x, y), name=name, is_charger=is_charger)
                self.vertex_names[i] = name
                
                if is_charger:
                    self.chargers.append(i)
            
            # Process lanes
            self.lanes = level_data['lanes']
            for lane in self.lanes:
                start, end, attrs = lane
                speed_limit = attrs.get('speed_limit', 0)
                
                # Calculate distance between vertices
                start_pos = np.array(self.vertices[start][:2])
                end_pos = np.array(self.vertices[end][:2])
                distance = np.linalg.norm(end_pos - start_pos)
                
                # Add edge to graph
                self.G.add_edge(start, end, speed_limit=speed_limit, distance=distance)
                
        except Exception as e:
            print(f"Error loading nav_graph: {e}")
            raise
    
        
    def print_vertex_info(self):
        """Debug method to verify vertex data"""
        print("\n=== Vertex Information ===")
        for node_id, data in self.G.nodes(data=True):
            print(f"Vertex ID: {node_id}, Position: {data['pos']}, Name: {data.get('name', '')}")
        print("=========================\n")
    
    def get_vertices(self):
        """Return all vertices with their positions and attributes."""
        return [(i, data) for i, data in self.G.nodes(data=True)]
    
    def get_lanes(self):
        """Return all lanes with their attributes."""
        return [(u, v, data) for u, v, data in self.G.edges(data=True)]
    
    def get_vertex_position(self, vertex_id):
        """Get the position of a vertex by its ID."""
        return self.G.nodes[vertex_id]['pos']
    
    def get_path(self, start, end):
        """Find the shortest path between start and end vertices."""
        try:
            path = nx.shortest_path(self.G, source=start, target=end, weight='distance')
            return path
        except nx.NetworkXNoPath:
            return None
    
    def get_vertex_id_by_position(self, position, threshold=1.0):
        """Find the closest vertex to a given position with proper validation"""
        min_dist = float('inf')
        closest_vertex = None
        
        for vertex_id, data in self.G.nodes(data=True):
            try:
                vertex_pos = data['pos']
                dist = math.sqrt((vertex_pos[0]-position[0])**2 + (vertex_pos[1]-position[1])**2)
                
                if dist < min_dist and dist < threshold:
                    min_dist = dist
                    closest_vertex = vertex_id
                    
            except KeyError:
                continue
            
        return closest_vertex
    
    def get_chargers(self):
        """Return all charging station vertices."""
        return self.chargers