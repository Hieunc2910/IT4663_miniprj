import sys
import heapq

def read_data(file_path=None):
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
    current_max = max(state.distances) if state.distances else 0

    if not state.unvisited:
        return current_max

    min_remaining = 0
    if state.unvisited:
        min_distances = []
        for node in state.unvisited:
            min_dist = float('inf')
            for vehicle_id in range(state.k):
                last_node = state.routes[vehicle_id][-1] if state.routes[vehicle_id] else 0
                dist = distances[last_node][node]
                min_dist = min(min_dist, dist)
            min_distances.append(min_dist)

        min_remaining = min(min_distances) if min_distances else 0

    return max(current_max, min_remaining)


def min_max_vrp(n, k, distances):
    best_solution = None
    best_max_distance = float('inf')

    root = State(n, k)
    for i in range(k):
        root.routes[i].append(0)

    queue = [root]
    nodes_explored = 0

    while queue and nodes_explored < 1000000:
        nodes_explored += 1
        current = heapq.heappop(queue)

        if not current.unvisited:
            max_distance = max(current.distances)

            if max_distance < best_max_distance:
                best_max_distance = max_distance
                best_solution = current

            continue

        if current.lower_bound >= best_max_distance:
            continue

        for node_id in list(current.unvisited):
            for vehicle_id in range(k):
                new_state = current.clone()
                last_node = new_state.routes[vehicle_id][-1]

                new_state.unvisited.remove(node_id)
                new_state.routes[vehicle_id].append(node_id)
                new_state.distances[vehicle_id] += distances[last_node][node_id]

                new_state.lower_bound = compute_lower_bound(new_state, distances)

                if new_state.lower_bound < best_max_distance:
                    heapq.heappush(queue, new_state)

    return best_solution


def format_solution(solution):
    output = [str(solution.k)]
    for route in solution.routes:
        output.append(str(len(route)))
        output.append(' '.join(map(str, route)))
    return '\n'.join(output)


def main():
    file_path = "10 2.txt"
    n, k, distances = read_data(file_path)
    solution = min_max_vrp(n, k, distances)
    if solution:
        print(format_solution(solution))
    else:
        print("No solution found")
if __name__ == "__main__":
    main()