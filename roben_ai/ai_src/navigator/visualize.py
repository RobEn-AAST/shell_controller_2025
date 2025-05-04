import matplotlib.pyplot as plt
from threading import Thread
import numpy as np
import cv2


def visualize(carla_map, target_locations, full_route, ego_location):
    plt.figure(figsize=(12, 12))
    
    # 1. Get all road boundaries
    min_x, max_x = float('inf'), -float('inf')
    min_y, max_y = float('inf'), -float('inf')
    road_lines = []
    
    # First pass: collect boundaries and road lines
    for wp_start, wp_end in carla_map.get_topology():
        start_loc = wp_start.transform.location
        end_loc = wp_end.transform.location
        
        # Update boundaries
        min_x = min(min_x, start_loc.x, end_loc.x)
        max_x = max(max_x, start_loc.x, end_loc.x)
        min_y = min(min_y, start_loc.y, end_loc.y)
        max_y = max(max_y, start_loc.y, end_loc.y)
        
        road_lines.append(((start_loc.x, start_loc.y), (end_loc.x, end_loc.y)))

    # 2. Set map boundaries with 10% padding
    plt.xlim(min_x - 0.1*(max_x-min_x), max_x + 0.1*(max_x-min_x))
    plt.ylim(min_y - 0.1*(max_y-min_y), max_y + 0.1*(max_y-min_y))

    # 3. Draw roads with proper lane awareness
    for (x1, y1), (x2, y2) in road_lines:
        plt.plot([x1, x2], [y1, y2], 'black', linewidth=1, alpha=0.4, zorder=1)

    # 4. Draw route with lane-centering
    route_x = [wp[0].transform.location.x for wp in full_route]
    route_y = [wp[0].transform.location.y for wp in full_route]
    plt.plot(route_x, route_y, 'deepskyblue', linewidth=2, zorder=3, label='Planned Path')

    # 5. Plot projected targets
    target_x = [loc.x for loc in target_locations]
    target_y = [loc.y for loc in target_locations]
    plt.scatter(target_x, target_y, s=50, c='lime', marker='*', 
                edgecolors='darkgreen', zorder=4, label='Targets')

    # 6. Plot vehicle position
    plt.scatter(ego_location.x, ego_location.y, s=80, c='red', 
                edgecolors='darkred', zorder=5, label='Ego Vehicle')

    plt.gca().set_aspect('equal', adjustable='box')
    plt.title(f"{carla_map.name} Navigation Plan")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.legend(loc='upper right')
    plt.grid(alpha=0.3)
    plt.show()
    

class CarlaVisualizer(Thread):
    def __init__(self, agent_route, targets, carla_map, map_size=(1000, 1000)):
        super().__init__()
        self.running = True
        self.agent_route = agent_route
        self.targets = targets  # List of carla.Location
        self.carla_map = carla_map
        self.map_scale = 0.2  # 1px = 0.2 meters
        self.map_size = map_size
        self.current_pos = (0, 0)
        
        # Precompute road network lines
        self.road_lines = []
        for wp_start, wp_end in carla_map.get_topology():
            start = self._world_to_pixel(wp_start.transform.location)
            end = self._world_to_pixel(wp_end.transform.location)
            self.road_lines.append((start, end))
            
        # Precompute target positions
        self.target_points = [self._world_to_pixel(t) for t in targets]
        
    def _world_to_pixel(self, location):
        return (
            int(location.x * self.map_scale + self.map_size[0]/2),
            int(-location.y * self.map_scale + self.map_size[1]/2)
        )
        
    def run(self):
        canvas = np.zeros((self.map_size[1], self.map_size[0], 3), dtype=np.uint8)
        
        # Draw road network (yellow)
        for start, end in self.road_lines:
            cv2.line(canvas, start, end, (0, 255, 255), 1, lineType=cv2.LINE_AA)
            
        # Draw planned path (blue)
        path_points = [self._world_to_pixel(wp[0].transform.location) for wp in self.agent_route]
        for i in range(1, len(path_points)):
            cv2.line(canvas, path_points[i-1], path_points[i], (255, 0, 0), 1, lineType=cv2.LINE_AA)
            
        # Draw targets (green circles)
        for point in self.target_points:
            cv2.circle(canvas, point, 4, (0, 255, 0), -1, lineType=cv2.LINE_AA)
            
        while self.running:
            frame = canvas.copy()
            
            # Draw vehicle (small red dot)
            cv2.circle(frame, self.current_pos, 2, (0, 0, 255), -1, lineType=cv2.LINE_AA)
            
            cv2.imshow("CARLA Navigation", frame)
            if cv2.waitKey(20) == ord('q'):
                self.running = False
                
    def update_position(self, location):
        self.current_pos = self._world_to_pixel(location)

