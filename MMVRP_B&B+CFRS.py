import sys
import heapq
import random
from time import time


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
        self.depth = 0

    def __lt__(self, other):
        if abs(self.lower_bound - other.lower_bound) < 1e-6:
            return self.depth > other.depth
        return self.lower_bound < other.lower_bound

    def clone(self):
        new_state = State(self.n, self.k)
        new_state.routes = [route[:] for route in self.routes]
        new_state.distances = self.distances.copy()
        new_state.unvisited = self.unvisited.copy()
        new_state.lower_bound = self.lower_bound
        new_state.depth = self.depth + 1
        return new_state


def preprocess_distances(distances, n):
    """Calculate minimum outgoing distance from each node"""
    min_outgoing = [float('inf')] * (n + 1)

    for i in range(n + 1):
        for j in range(n + 1):
            if i != j:
                min_outgoing[i] = min(min_outgoing[i], distances[i][j])

    return min_outgoing


def create_geographical_clusters(n, k, distances):
    """Divide nodes into geographical clusters based on distances"""
    nodes = list(range(1, n + 1))
    clusters = [[] for _ in range(k)]
    centroids = []

    # K-means++ initialization for centroids
    first_centroid = random.choice(nodes)
    centroids.append(first_centroid)

    # Choose remaining centroids with probability proportional to distance
    while len(centroids) < k:
        distances_to_centroids = []
        for node in nodes:
            if node in centroids:
                distances_to_centroids.append(0)
            else:
                min_dist = min(distances[node][c] for c in centroids)
                distances_to_centroids.append(min_dist ** 2)

        # Choose next centroid
        sum_distances = sum(distances_to_centroids)
        if sum_distances == 0:
            next_centroid = random.choice([n for n in nodes if n not in centroids])
        else:
            probabilities = [d / sum_distances for d in distances_to_centroids]
            next_centroid = random.choices(nodes, weights=probabilities, k=1)[0]

        centroids.append(next_centroid)

    # Assign nodes to clusters
    for node in nodes:
        min_dist = float('inf')
        best_cluster = 0

        for i, centroid in enumerate(centroids):
            dist = distances[node][centroid]
            if dist < min_dist:
                min_dist = dist
                best_cluster = i

        clusters[best_cluster].append(node)

    # Balance clusters
    balance_clusters(clusters, distances, k)

    return clusters


def balance_clusters(clusters, distances, k):
    """Balance clusters to have similar sizes"""
    target_size = sum(len(cluster) for cluster in clusters) // k

    # Sort clusters by size (descending)
    cluster_sizes = [(len(cluster), i) for i, cluster in enumerate(clusters)]
    cluster_sizes.sort(reverse=True)

    # Transfer nodes from largest to smallest clusters
    for _ in range(3):  # Multiple passes
        cluster_sizes = [(len(cluster), i) for i, cluster in enumerate(clusters)]
        cluster_sizes.sort(reverse=True)

        for large_size, large_idx in cluster_sizes:
            if large_size <= target_size + 1:
                continue

            small_size, small_idx = cluster_sizes[-1]

            # Find node that's closest to small cluster's nodes
            best_node = None
            best_avg_dist = float('inf')

            for node in clusters[large_idx]:
                avg_dist = sum(distances[node][other] for other in clusters[small_idx]) / (small_size or 1)
                if avg_dist < best_avg_dist:
                    best_avg_dist = avg_dist
                    best_node = node

            if best_node:
                clusters[large_idx].remove(best_node)
                clusters[small_idx].append(best_node)


def optimize_cluster_route(cluster, distances):
    """Optimize route within a cluster starting from depot without returning"""
    if not cluster:
        return [], 0  # Empty route

    # Start from depot
    current = 0
    route = [0]
    unvisited = cluster.copy()

    # Build initial route with nearest neighbor
    while unvisited:
        nearest = min(unvisited, key=lambda node: distances[current][node])
        route.append(nearest)
        unvisited.remove(nearest)
        current = nearest

    # 2-opt improvement
    if len(route) > 2:
        improved = True
        while improved:
            improved = False
            for i in range(0, len(route) - 1):
                for j in range(i + 1, len(route)):

                    if i == 0:
                        from_i = 0  # Depot
                    else:
                        from_i = route[i - 1]

                    to_i = route[i]
                    to_j = route[j]

                    if j < len(route) - 1:
                        from_j = route[j + 1]
                    else:
                        continue

                    current_dist = distances[from_i][to_i] + distances[to_j][from_j]
                    new_dist = distances[from_i][to_j] + distances[to_i][from_j]

                    if new_dist < current_dist:
                        route[i:j + 1] = reversed(route[i:j + 1])
                        improved = True
                        break
                if improved:
                    break

    # Calculate total distance (from depot through all nodes)
    total_dist = 0
    prev_node = 0
    for node in route:
        total_dist += distances[prev_node][node]
        prev_node = node

    return route, total_dist


def compute_lower_bound(state, distances, min_outgoing):
    """Enhanced lower bound using cluster-based reasoning"""

    current_max = max(state.distances) if state.distances else 0

    # If all nodes are visited, return the current max distance
    if not state.unvisited:
        return current_max

    # 1. Minimum insertion cost estimation
    min_insertion_cost = float('inf')
    for node in state.unvisited:
        best_insertion = float('inf')
        for vehicle_id in range(state.k):
            last_node = state.routes[vehicle_id][-1] if state.routes[vehicle_id] else 0
            insertion_cost = distances[last_node][node]
            best_insertion = min(best_insertion, insertion_cost)
        min_insertion_cost = min(min_insertion_cost, best_insertion)

    # 2. MST-based bound for remaining nodes
    mst_cost = 0
    remaining = list(state.unvisited)

    if len(remaining) > 1:
        # Prim's algorithm for MST approximation
        visited = {remaining[0]}
        while len(visited) < len(remaining):
            best_edge = float('inf')
            for u in visited:
                for v in remaining:
                    if v not in visited and distances[u][v] < best_edge:
                        best_edge = distances[u][v]
            mst_cost += best_edge
            visited.add(remaining[len(visited)])

    # 3. Even distribution bound
    remaining_nodes = len(state.unvisited)
    min_vehicles_needed = remaining_nodes // state.k + (1 if remaining_nodes % state.k else 0)

    # Calculate final lower bound
    return max(current_max,
               current_max + min_insertion_cost,
               min(state.distances) + mst_cost / state.k)


def cluster_first_route_second(n, k, distances, time_limit=25):
    start_time = time()

    min_outgoing = preprocess_distances(distances, n)

    # Step 1: Create geographical clusters
    clusters = create_geographical_clusters(n, k, distances)

    # Step 2: Optimize routes within each cluster
    optimized_routes = []
    cluster_distances = []

    for i, cluster in enumerate(clusters):
        route, distance = optimize_cluster_route(cluster, distances)
        optimized_routes.append(route)
        cluster_distances.append(distance)

    # Create initial solution from clusters
    initial_solution = State(n, k)
    for i in range(k):
        initial_solution.routes[i] = optimized_routes[i]  # Routes now start from depot (implied)
        initial_solution.distances[i] = cluster_distances[i]
        for node in optimized_routes[i]:
            if node > 0:  # Skip depot nodes
                initial_solution.unvisited.remove(node)

    # Get best max distance
    best_max_distance = max(initial_solution.distances)
    best_solution = initial_solution

    # Step 3: Apply Branch and Bound to improve the solution
    # Initialize priority queue with improved initial state
    root = initial_solution
    root.lower_bound = compute_lower_bound(root, distances, min_outgoing)

    queue = [root]
    nodes_explored = 0
    best_states = {}

    while queue and nodes_explored <= 100000 and time() - start_time < time_limit:
        nodes_explored += 1
        current = heapq.heappop(queue)

        # Skip dominated states
        unvisited_key = frozenset(current.unvisited)
        if unvisited_key in best_states:
            max_dist = max(current.distances)
            if max_dist >= best_states[unvisited_key]:
                continue
            best_states[unvisited_key] = max_dist
        else:
            best_states[unvisited_key] = max(current.distances)

        # If all nodes have been visited
        if not current.unvisited:
            max_distance = max(current.distances)
            if max_distance < best_max_distance:
                best_max_distance = max_distance
                best_solution = current
            continue

        # Prune if lower bound exceeds best solution
        if current.lower_bound >= best_max_distance:
            continue

        # Dynamic variable ordering based on regret
        regret_values = []
        for node in current.unvisited:
            # Calculate insertion costs for all vehicles
            insertion_costs = []
            for vehicle_id in range(k):
                last_node = current.routes[vehicle_id][-1] if current.routes[vehicle_id] else 0
                insertion_costs.append((distances[last_node][node], vehicle_id))

            # Sort by cost to find best and second best
            insertion_costs.sort()
            if len(insertion_costs) >= 2:
                regret = insertion_costs[1][0] - insertion_costs[0][0]
                regret_values.append((regret, node, insertion_costs[0][1]))
            else:
                regret_values.append((0, node, insertion_costs[0][1]))

        # Sort by regret (descending)
        regret_values.sort(reverse=True)

        # Limit branching factor
        max_branching = min(3, len(current.unvisited))

        for _, node_id, best_vehicle in regret_values[:max_branching]:
            vehicles_to_try = [best_vehicle]

            # Add a few more vehicles that are promising
            other_vehicles = sorted(range(k),
                                    key=lambda v: distances[current.routes[v][-1] if current.routes[v] else 0][node_id])

            for v in other_vehicles:
                if v != best_vehicle and len(vehicles_to_try) < 3:
                    vehicles_to_try.append(v)

            for vehicle_id in vehicles_to_try:
                new_state = current.clone()
                last_node = new_state.routes[vehicle_id][-1] if new_state.routes[vehicle_id] else 0

                new_state.unvisited.remove(node_id)
                new_state.routes[vehicle_id].append(node_id)
                new_state.distances[vehicle_id] += distances[last_node][node_id]

                if new_state.distances[vehicle_id] >= best_max_distance:
                    continue

                new_state.lower_bound = compute_lower_bound(new_state, distances, min_outgoing)

                if new_state.lower_bound < best_max_distance:
                    heapq.heappush(queue, new_state)

    return best_solution


def format_solution(solution):
    output = [str(solution.k)]
    for route in solution.routes:
        route = [0] + route
        output.append(str(len(route)))
        output.append(' '.join(map(str, route)))
    return '\n'.join(output)


def main():
    file_path = "1000 20.txt"
    n, k, distances = read_data(file_path)
    best_solution = None
    best_max_distance = float('inf')
    num_iterations = 100

    start_time = time()

    for i in range(num_iterations):
        solution = cluster_first_route_second(n, k, distances, time_limit=25 / num_iterations)

        if solution:
            max_distance = max(solution.distances)
            if max_distance < best_max_distance:
                best_max_distance = max_distance
                best_solution = solution
        if time() - start_time > 24:
            break


    if best_solution:
        print(format_solution(best_solution))
    else:
        print("No solution found.", file=sys.stderr)


if __name__ == "__main__":
    main()
