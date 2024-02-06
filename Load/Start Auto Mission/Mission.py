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


@dataclass
class Waypoint:
    u: int
    v: int
    point: LocationGlobal
    weight: int = 1

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

    def build_mission(self, home_lat: int = 120.1, home_long: int = 35.5, TargetAltitude: int =20, Area: int =160000, Cam_FOV: int =160):
        # We need to build a Mission with least waypoints holding GPS coordinates for each waypoint. 
        # Starting at home location, build LocationGlobal waypoints for each state in the mission plan
        # Allow for all area to be covered by the camera at a given altitude. 
        # We will use a 160 degree camera FOV and 30 meters altitude to cover 160 square meters of area

        # calculate area covered by camera at given altitude, assuming camera is facing directly down

        # Calculate area of 0 altitude image captured by camera at given altitude in square meters using camera FOV in degrees and altitude in meters
        camera_area = (math.tan(math.radians(Cam_FOV/2)) * TargetAltitude)**2
        
        
        num_waypoints = Area // camera_area
        # calculate distance between waypoints
        distance = math.sqrt(camera_area)
        # calculate number of rows needed to cover the area
        num_rows = math.sqrt(num_waypoints)
        # calculate number of columns needed to cover the area
        num_columns = num_waypoints // num_rows
        # calculate distance between rows
        row_distance = distance
        # calculate distance between columns
        column_distance = distance
        # calculate starting point for the mission
        start = LocationGlobal(home_lat, home_long, TargetAltitude)
        # calculate ending point for the mission
        end = LocationGlobal(home_lat, home_long, TargetAltitude)

        # create GPS coordinates for each waypoint in the mission plan in a grid, 
        # Start at home location and move to the right, then move up, then move to the left, then move back dwon, and so on.
        # Assuming camera is facing down and home location is starting point directly in middle of grid
        # until the entire area is covered
        """
        Grid will look like this: where H is home location and firt waypoint is the top left corner of the grid
        * * * * * * * * * 
        * * * * * * * * * 
        * * * * * * * * * 
        * * * * * * * * *
        * * * * h * * * *
        * * * * * * * * *
        * * * * * * * * *
        * * * * * * * * *
        * * * * * * * * *
        """
        # create first waypoint at top left corner of the grid
        first = LocationGlobal(home_lat - (num_rows/2 * row_distance), home_long - (num_columns/2 * column_distance), TargetAltitude)
        # create last waypoint at bottom right corner of the grid
        last = LocationGlobal(home_lat + (num_rows/2 * row_distance), home_long + (num_columns/2 * column_distance), TargetAltitude)
        # create Waypoint objects for each waypoint in the mission plan
        # Make four quadrants of the grid, starting from the top left corner and moving to the right, then down, then left, then up
        # until the entire area is covered
        # Quadrant 1
        for i in range(num_rows//2):
            for j in range(num_columns//2):
                self.add_waypoint(Waypoint(first, LocationGlobal(first.lat + i * row_distance, first.lon + j * column_distance, TargetAltitude)))
        # Quadrant 2
        for i in range(num_rows//2):
            for j in range(num_columns//2):
                self.add_waypoint(Waypoint(first, LocationGlobal(first.lat + i * row_distance, first.lon - j * column_distance, TargetAltitude)))
        # Quadrant 3
        for i in range(num_rows//2):
            for j in range(num_columns//2):
                self.add_waypoint(Waypoint(first, LocationGlobal(first.lat - i * row_distance, first.lon - j * column_distance, TargetAltitude)))
        # Quadrant 4
        for i in range(num_rows//2):
            for j in range(num_columns//2):
                self.add_waypoint(Waypoint(first, LocationGlobal(first.lat - i * row_distance, first.lon + j * column_distance, TargetAltitude)))


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


if __name__ == "__main__":
    TAP_FSM = TAPfsm()
    TAP_FSM.get_min_path("Test-Logic-Reset", "Update-IR")
