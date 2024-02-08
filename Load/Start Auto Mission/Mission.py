"""
Mission Plan Graph for Guided UAV Navigation
Author: Dmitri Lyalikov

Uses Djikstra Algorithm to find shortest path between
two waypoints of mission plan represented as Weighted Graph

"""
from dataclasses import dataclass
from functools import lru_cache
from heapq import heappush, heappop
from dronekit import LocationGlobal
import math
import numpy as np


@dataclass
class Waypoint:
    u: int # index of starting state
    v: int # index of ending state
    point: LocationGlobal # GPS coordinates of the waypoint
    weight: int = 1 # Unit time to travel from u to v in seconds 

    def reversed(self):
        return Waypoint(self.v, self.u, self.weight)

    def __lt__(self, other) -> bool:
        return self.weight < other.weight

    def __str__(self) -> str:
        return f"{self.u} -> {self.v}"


class Mission:
    def __init__(self, states=STATES):
        self._states = states
        self._waypoints = [[] for _ in states]
        # Create default survey mission at 30 meters covering 160 meters
        self.build_mission()

    @property
    def state_count(self):
        return len(self._states)

    @property
    def waypoint_count(self):
        return sum(map(len, self._waypoints))

    def index_of(self, state) -> int:
        return self._states.index(state)

    def add_waypoint(self, waypoint):
        self._waypoints[waypoint.u].append(waypoint)

    def state_at(self, index: int):
        return self._states[index]

    def add_state(self, state):
        self._states.append(state)
        self._waypoint.append([])
        return self.state_count - 1

    def add_waypoint_by_indices(self, u, v, tms):
        waypoint = Waypoint(u, v, tms)
        self.add_waypoint(waypoint)

    def add_waypoint_by_vertices(self, first, second, tms):
        u = self._states.index(first)
        v = self._states.index(second)
        self.add_waypoint_by_indices(u, v, tms)

    def waypoints_for_index(self, index: int):
        return self._waypoints[index]

    def neighbors_for_index_with_weights(self, index):
        distance_tuples = []
        for waypoint in self.waypoints_for_index(index):
            distance_tuples.append((self.state_at(waypoint.v), waypoint.TMS))
        return distance_tuples

    def __str__(self):
        desc: str = ''
        for i in range(self.state_count):
            desc += f"{self.state_at(i)} -> {self.neighbors_for_index_with_weights(i)} \n"
        return desc

    @lru_cache(maxsize=None)
    def get_min_path(self, start, end):
        assert start in self._states and end in self._states
        distances, path_dict = self.dijkstra(start)
        path = path_dict_to_path(self.index_of(start),
                                 self.index_of(end), path_dict)
        return get_weighted_path(self, path)

    def dijkstra(self, root):
        # Find starting index
        first: int = self.index_of(root)
        # Distances are unknown first
        distances = [None] * self.state_count
        # Root is 0 away from root
        distances[first] = 0
        path_dict = {}
        pq = PriorityQueue()
        pq.push(DijkstraNode(first, 0))
        while not pq.empty:
            u: int = pq.pop().state
            dist_u = distances[u]
            for we in self.wayypoints_for_index(u):
                dist_v = distances[we.v]
                if dist_v is None or dist_v > we.weight + dist_u:
                    distances[we.v] = we.weight + dist_u
                    path_dict[we.v] = we
                    pq.push(DijkstraNode(we.v, we.weight + dist_u))
        return distances, path_dict

    def build_mission(self, home_lat: int = 120.1, home_long: int = 35.5, TargetAltitude: int =20, Area: int =16000, Cam_FOV: int =160, MAX_RANGE=250):
        # We need to build a Mission with least waypoints holding GPS coordinates for each waypoint. 
        # Starting at home location, build LocationGlobal waypoints for each state in the mission plan
        # Allow for all area to be covered by the camera at a given altitude. 
        # We will use a 160 degree camera FOV and 30 meters altitude to cover 160 square meters of area


        # calculate area covered by camera at given altitude, assuming camera is facing directly down

        # Calculate area of 0 altitude image captured by camera at given altitude in square meters using camera FOV in degrees and altitude in meters
        camera_area = ((math.tan(math.radians(Cam_FOV/2)) * TargetAltitude)*2)**2

        MAX_RANGE = MAX_RANGE / math.sqrt(camera_area)

        # width of the grid area to be covered by the camera in meters
        L = math.sqrt(Area)
        # How many waypoints to cover this width 
        waypoint_width = int(L / math.sqrt(camera_area)) + 1
        # Make sure waypoint_width is odd, so center home location is in the middle of the grid
        if waypoint_width % 2 == 0:
            waypoint_width += 1
        
        # Furthest distance from the origin
        reach = L // 2 
        if reach > MAX_RANGE: # Change max range units of camera area
            reach = MAX_RANGE
        reach = int(reach)

        start = LocationGlobal(home_lat, home_long, TargetAltitude)
        # Create np Grid matrix of waypoints
        # Create waypoint_witdth x waypoint_width grid of waypoints. Home coordinates will be in the middle of the grid. (0, 0)
        # Expand grid from center in all four quadrants. 
        # If Reach = 1: Matrix looks like
        #    (-1, 1), (0, 1), (1, 1)
        #    (-1, 0), (0, 0), (1, 0)
        #    (-1, -1), (0, -1), (1, -1)

        # If Reach = 2: Matrix looks like:
        #   (-2, 2), (-1, 2), (0, 2), (1, 2), (2, 2)
        #   (-2, 1), (-1, 1), (0, 1), (1, 1), (2, 1)
        #   (-2, 0), (-1, 0), (0, 0), (1, 0), (2, 0)
        #   (-2, -1), (-1, -1), (0, -1), (1, -1), (2, -1)
        #   (-2, -2), (-1, -2), (0, -2), (1, -2), (2, -2)
        #         
        # Create matrix based on reach
        # Create 2D matrix 
        transform = np.zeros((reach * 2 + 1, reach * 2 + 1), dtype=object)
        for y in range(-reach, reach + 1):
            # Iterate in reverse direction as done for x
            for x in range(-reach, reach + 1):
                transform[x + reach, y + reach] = (x, -y)

        # Generate Vertices for the graph
        path_matrix = create_spiral_matrix(waypoint_width)

        # Iterate through transfrom and create waypoints where:
        # Longitude = home_long + x * waypoint_width
        # Latitude = home_lat + y * waypoint_width
        # Iterate through numpy matrix 
        for elem in transform.flatten():
            # Get x and y coordinates
            waypoint = transform_coordinate(home_lat, home_long, TargetAltitude, math.sqrt(waypoint_width), elem)
            # Add waypoint to mission plan, and connect to next waypoint as defined in path_matrix
            self.add_waypoint(waypoint)
            
            
            
            
            
            # self.add_waypoint(Waypoint(0, 0, waypoint))
        
        

# Function that returns new GPS coordinates when adding some meters to longitude and latitude
def new_gps_coords(lat, lon, dNorth, dEast):
    earth_radius = 6378137.0
    # Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*lat/180))

    # New coordinates in degrees
    newlat = lat + dLat * 180/math.pi
    newlon = lon + dLon * 180/math.pi
    return LocationGlobal(newlat, newlon, 0)


@dataclass
class DijkstraNode:
    state: int
    distance: float

    def __lt__(self, other) -> bool:
        return self.distance < other.distance

    def __eq__(self, other) -> bool:
        return self.distance == other.distance


class PriorityQueue:
    def __init__(self):
        self._container = []

    @property
    def empty(self) -> bool:
        return not self._container

    def push(self, item) -> None:
        heappush(self._container, item)

    def pop(self):
        return heappop(self._container)

    def __repr__(self):
        return repr(self._container)


def path_dict_to_path(start, end, path_dict):
    if len(path_dict) == 0:
        return []
    waypoint_path = []
    wp = path_dict[end]
    waypoint_path.append(wp)
    while wp.u != start:
        wp = path_dict[wp.u]
        waypoint_path.append(wp)
    return list(reversed(waypoint_path))


def get_weighted_path(wg, wp):
    path = []
    for waypoint in wp:
        print(f"{wg.state_at(waypoint.u)} {waypoint.TMS} > {wg.state_at(waypoint.v)}")
        path.append(waypoint.point)
    print(path)
    return path

def haversine(long1, lat1, long2, lat2):
    """
    Calculate the great circle distance between two points
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians
    long1, lat1, long2, lat2 = map(math.radians, [long1, lat1, long2, lat2])

    # haversine formula
    dlon = long2 - long1
    dlat = lat2 - lat1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    r = 6371 # Radius of earth in kilometers. Use 3956 for miles
    # Return distance in meters
    return c * r 

def transform_coordinate(home_lat, home_long, targetAlt, unit, direction: (int, int)):
    """
    Transform GPS coordinates by adding some meters to longitude and latitude
    """
    x_scale, y_scale = direction

    new_long = home_long + (x_scale * unit) / (haversine(home_long, home_lat, home_long + 1, home_lat) * 1000)
    new_lat = home_lat + (y_scale * unit) / (haversine(home_long, home_lat, home_long, home_lat + 1) * 1000)
    
    return LocationGlobal(new_lat, new_long, TargetAlt)


def create_spiral_matrix(n):
    """
    Create a 2D numpy array with a spiral pattern of incremental numbers.

    Parameters:
    - n: Size of the square matrix (n x n). n must be odd.

    Returns:
    - A 2D numpy array with a spiral pattern.
    """
    if n % 2 == 0:
        raise ValueError("n must be odd")

    matrix = np.zeros((n, n), dtype=int)

    mid = n // 2
    current_value = 0

    for layer in range(mid + 1):
        # Top row
        for i in range(mid - layer, mid + layer + 1):
            matrix[mid - layer, i] = current_value
            current_value += 1

        # Right column
        for i in range(mid - layer + 1, mid + layer + 1):
            matrix[i, mid + layer] = current_value
            current_value += 1

        # Bottom row
        for i in range(mid + layer - 1, mid - layer, -1):
            matrix[mid + layer, i] = current_value
            current_value += 1

        # Left column
        for i in range(mid + layer , mid - layer, -1):
            matrix[i, mid - layer] = current_value
            current_value += 1

    return matrix


if __name__ == "__main__":
    TAP_FSM = TAPfsm()
    TAP_FSM.get_min_path("Test-Logic-Reset", "Update-IR")
