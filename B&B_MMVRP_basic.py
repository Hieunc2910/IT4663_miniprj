import sys
import heapq


def read_data(file_path=None):
    """Read input data from file or stdin"""
    if file_path:
        with open(file_path, 'r') as f:
            lines = f.readlines()
    else:
        lines = sys.stdin.readlines()

    n, k = map(int, lines[0].split())
    distance_matrix = []

    for i in range(1, n + 2):
        row = list(map(int, lines[i].split()))
        distance_matrix.append(row)

    return n, k, distance_matrix


class State:
    def __init__(self, n, k):
        self.n = n
        self.k = k
        self.routes = [[] for _ in range(k)]
        self.distances = [0] * k
        self.unvisited = set(range(1, n + 1))
        self.lower_bound = 0

    def __lt__(self, other):
        return self.lower_bound < other.lower_bound

    def clone(self):
        new_state = State(self.n, self.k)
        new_state.routes = [route[:] for route in self.routes]
        new_state.distances = self.distances.copy()
        new_state.unvisited = self.unvisited.copy()
        new_state.lower_bound = self.lower_bound
        return new_state


def compute_lower_bound(state, distances):
    """Compute a lower bound for a partial solution"""
    # Current max distance
    current_max = max(state.distances) if state.distances else 0

    # If all nodes are visited, return the current max distance
    if not state.unvisited:
        return current_max

    # Add minimum distance from each unvisited node to any route
    min_remaining = 0
    if state.unvisited:
        # For each unvisited node, find its minimum distance to any route endpoint
        min_distances = []
        for node in state.unvisited:
            min_dist = float('inf')
            for vehicle_id in range(state.k):
                last_node = state.routes[vehicle_id][-1] if state.routes[vehicle_id] else 0
                dist = distances[last_node][node]
                min_dist = min(min_dist, dist)
            min_distances.append(min_dist)

        # Simple estimate: add at least the minimum distance to the current max
        min_remaining = min(min_distances) if min_distances else 0

    return max(current_max, min_remaining)


def min_max_vrp(n, k, distances):
    best_solution = None
    best_max_distance = float('inf')

    # Initialize priority queue with root node
    root = State(n, k)
    for i in range(k):
        root.routes[i].append(0)  # Add depot to each route

    queue = [root]
    nodes_explored = 0

    while queue and nodes_explored < 1000000:
        nodes_explored += 1
        current = heapq.heappop(queue)

        # If all nodes have been visited
        if not current.unvisited:
            # Complete routes by returning to depot
            max_distance = 0
            for i in range(k):
                if current.routes[i][-1] != 0:
                    current.distances[i] += distances[current.routes[i][-1]][0]
                    current.routes[i].append(0)
                max_distance = max(max_distance, current.distances[i])

            if max_distance < best_max_distance:
                best_max_distance = max_distance
                best_solution = current

            continue

        if current.lower_bound >= best_max_distance:
            continue

        # Try to assign each unvisited node to each vehicle
        for node_id in list(current.unvisited):
            for vehicle_id in range(k):
                new_state = current.clone()
                last_node = new_state.routes[vehicle_id][-1]

                # Add the node to the route
                new_state.unvisited.remove(node_id)
                new_state.routes[vehicle_id].append(node_id)
                new_state.distances[vehicle_id] += distances[last_node][node_id]

                # Compute new lower bound
                new_state.lower_bound = compute_lower_bound(new_state, distances)

                # Prune if the new bound exceeds the best solution
                if new_state.lower_bound < best_max_distance:
                    heapq.heappush(queue, new_state)

    return best_solution


def format_solution(solution):
    """Format the solution according to the output requirements"""
    output = [str(solution.k)]
    for route in solution.routes:
        # Remove the final depot visit for output format if present
        if route[-1] == 0:
            route = route[:-1]
        output.append(str(len(route)))
        output.append(' '.join(map(str, route)))
    return '\n'.join(output)


def main():
    # Hardcode the input file name
    file_path = "10 2.txt"
    n, k, distances = read_data(file_path)

    solution = min_max_vrp(n, k, distances)

    if solution:
        print(format_solution(solution))
    else:
        print("No solution found")
if __name__ == "__main__":
    main()
