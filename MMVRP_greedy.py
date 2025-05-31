string = input().split()
n = int(string[0])
k = int(string[1])

t = []
for i in range(n+1):
    row = list(map(int, input().split()))
    t.append(row)

path = [[0] for x in range(k)]

def calculate_route_time(route):
    total_time = 0
    for i in range(len(route)-1):
        total_time += t[route[i]][route[i+1]]
    return total_time

for i in range(1, n+1):
    best_cost = 1e9
    best_route = -1
    best_position = -1
    
    for j in range(k):
        for pos in range(1, len(path[j])+1):  
            new_route = path[j][:pos] + [i] + path[j][pos:]
            new_time = calculate_route_time(new_route)
            
            if new_time < best_cost:
                best_cost = new_time
                best_route = j
                best_position = pos
    
    path[best_route] = path[best_route][:best_position] + [i] + path[best_route][best_position:]

print(k)
for i in range(k):
    print(len(path[i]))
    for j in range(len(path[i])):
        print(path[i][j], end=" ")
    print()