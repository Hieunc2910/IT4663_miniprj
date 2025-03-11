from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import sys


def read_input():
    # Đọc giá trị N (số điểm thu gom), K (số bưu tá)
    N, K = map(int, sys.stdin.readline().split())

    # Đọc ma trận khoảng cách
    distance_matrix = [list(map(int, sys.stdin.readline().split())) for _ in range(N + 1)]

    return N, K, distance_matrix


def solve_mmvrp(N, K, distance_matrix):
    # 1. Khởi tạo Routing Model
    manager = pywrapcp.RoutingIndexManager(N + 1, K, 0)  # Điểm xuất phát là 0
    model = pywrapcp.RoutingModel(manager)

    # 2. Định nghĩa callback tính chi phí di chuyển
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = model.RegisterTransitCallback(distance_callback)
    model.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # 3. Thêm dimension để theo dõi khoảng cách
    dimension_name = 'Distance'
    model.AddDimension(
        transit_callback_index,
        0,  # no slack
        3000,  # large upper bound for route distance
        True,  # start cumul to zero
        dimension_name
    )

    distance_dimension = model.GetDimensionOrDie(dimension_name)

    # 4. Thiết lập mục tiêu Min-Max
    # Sử dụng cách khác để thiết lập mục tiêu Min-Max
    # Minmize max of route costs
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # 5. Thiết lập thông số tìm kiếm
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.seconds = 10  # Giới hạn thời gian 10 giây

    # 6. Giải bài toán
    solution = model.SolveWithParameters(search_parameters)

    # 7. Xuất kết quả
    if solution:
        print(K)  # In số bưu tá
        for vehicle_id in range(K):
            index = model.Start(vehicle_id)
            route = []
            while not model.IsEnd(index):
                route.append(manager.IndexToNode(index))
                index = solution.Value(model.NextVar(index))
            print(len(route))  # In số điểm trong tuyến
            print(" ".join(map(str, route)))  # In danh sách điểm theo tuyến
    else:
        print("Không tìm thấy lời giải!")

if __name__ == "__main__":
    N, K, distance_matrix = read_input()
    solve_mmvrp(N, K, distance_matrix)