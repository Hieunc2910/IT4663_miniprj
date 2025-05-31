import sys
import math
import copy

def read_input():
    N, K = map(int, sys.stdin.readline().split())
    d = []
    for _ in range(N+1):
        d.append(list(map(int, sys.stdin.readline().split())))
    return N, K, d

def route_length(route, d):
    total = 0
    for i in range(len(route)-1):
        total += d[route[i]][route[i+1]]
    return total

def insert_best(route, point, d):
    best_cost = math.inf
    best_pos = len(route)  # Mặc định chèn cuối
    
    # Thử chèn vào cuối (không cần quay về depot)
    new_route = route + [point]
    cost = route_length(new_route, d)
    if cost < best_cost:
        best_cost = cost
        best_pos = len(route)
    
    # Thử chèn vào các vị trí giữa
    for i in range(1, len(route)):
        new_route = route[:i] + [point] + route[i:]
        cost = route_length(new_route, d)
        if cost < best_cost:
            best_cost = cost
            best_pos = i
    
    return route[:best_pos] + [point] + route[best_pos:]

def two_opt_open(route, d):
    """Tối ưu hóa tuyến mở bằng 2-opt"""
    if len(route) < 4:  # Cần ít nhất 4 điểm để làm 2-opt
        return route
        
    improved = True
    best = route[:]
    best_length = route_length(route, d)
    
    while improved:
        improved = False
        for i in range(1, len(best) - 1):
            for j in range(i + 1, len(best)):
                if j - i == 1: 
                    continue
                    
                # Đảo ngược đoạn từ i đến j
                new_route = best[:i] + best[i:j+1][::-1] + best[j+1:]
                new_length = route_length(new_route, d)
                
                if new_length < best_length:
                    best = new_route
                    best_length = new_length
                    improved = True
                    break
            if improved:
                break
    return best

def solve(N, K, d):
    # Sắp xếp điểm theo khoảng cách từ depot giảm dần
    points = sorted(range(1, N+1), key=lambda x: d[0][x], reverse=True)
    
    # Khởi tạo K tuyến với K điểm xa nhất (chỉ bắt đầu từ depot)
    routes = [[0, p] for p in points[:K]]
    assigned = set(points[:K])
    
    # Chèn các điểm còn lại
    for p in points[K:]:
        # Chọn tuyến mà sau khi chèn, quãng đường dài nhất là nhỏ nhất
        best_route_idx = -1
        best_max_length = math.inf
        best_new_route = None
        
        for idx, route in enumerate(routes):
            new_route = insert_best(route, p, d)
            temp_routes = routes[:idx] + [new_route] + routes[idx+1:]
            max_length = max(route_length(r, d) for r in temp_routes)
            
            if max_length < best_max_length:
                best_max_length = max_length
                best_route_idx = idx
                best_new_route = new_route
        
        routes[best_route_idx] = best_new_route
        assigned.add(p)
    
    # Tối ưu hóa từng tuyến với 2-opt cho tuyến mở
    for i in range(K):
        routes[i] = two_opt_open(routes[i], d)
    
    return routes

def main():
    N, K, d = read_input()
    routes = solve(N, K, d)
    
    print(K)
    for route in routes:
        print(len(route))
        print(' '.join(map(str, route)))

if __name__ == '__main__':
    main()