#PYTHON 
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

def read_input():
    N, K = map(int, input().split())
    distance_matrix = []
    for _ in range(N + 1):
        distance_matrix.append(list(map(int, input().split())))
    return {
        "distance_matrix": distance_matrix,
        "num_vehicles": K,
        "depot": 0
    }

def print_solution(data, manager, routing, solution):
    print(data["num_vehicles"])
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        route = []
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        route.append(manager.IndexToNode(index))

        # Đảm bảo luôn bắt đầu từ depot
        if route[0] != data["depot"]:
            route.insert(0, data["depot"])

        # In đúng đề: không in depot cuối, không đếm depot
        pickup_points = [x for x in route if x != data["depot"]]
        print(len(pickup_points)+1)
        print(" ".join(map(str, route[:-1])))  # bỏ depot cuối

def solve_vrp(data):
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
    )
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_index)

    routing.AddDimension(
        transit_index,
        0, 1000000, True,
        "Distance"
    )
    distance_dimension = routing.GetDimensionOrDie("Distance")
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.time_limit.seconds = 10

    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        print_solution(data, manager, routing, solution)
    else:
        print(0)

def main():
    data = read_input()
    solve_vrp(data)

if __name__ == "__main__":
    main()
