from ortools.linear_solver import pywraplp

model = pywraplp.Solver.CreateSolver("SAT")
n,k = map(int, input().split())

d = []
for i in range(n+1):
    row = list(map(int, input().split()))
    d.append(row)

x = []
for i in range(n+1):
    layer = []
    for j in range(n+1):
        row = []
        for p in range(k):
            row.append(model.IntVar(0,1, f"x[{i},{j},{k}]"))
        layer.append(row) 
    x.append(layer)

z = model.IntVar(0,model.infinity(),"z")

u = []
for i in range(n+1):
    row = []
    for p in range(k):
        row.append(model.IntVar(0, n, f"u[{i},{p}]"))
    u.append(row)

for i in range(n+1):
    for p in range(k):
        model.Add(x[i][i][p]==0)


for p in range(k):
    model.Add(sum(x[0][i][p] for i in range(1, n+1))==1)

for i in range(1, n+1):
    model.Add(sum(x[j][i][p] for j in range(n+1) for p in range(k))==1)

for i in range(1, n+1):
    for p in range(k):
        model.Add(sum(x[i][j][p] for j in range(n+1))-sum(x[j][i][p] for j in range(n+1))==0)

for i in range(1, n+1):
    for j in range(1, n+1):
        if i!=j:    
            for p in range(k):
                model.Add(u[i][p]-u[j][p]+n*x[i][j][p]<=n-1)

for p in range(k):
    model.Add(z - sum(x[i][j][p]*d[i][j] for i in range(0,n+1) for j in range(1, n+1))>=0)
model.Minimize(z)

status = model.Solve()
if status == pywraplp.Solver.OPTIMAL or status == pywraplp.Solver.FEASIBLE:
    print(k)
    for p in range(k):
        path = [0]
        m=-1
        while(m!=0):
            tmp = path[-1]
            for i in range(n+1):
                if i!=tmp and x[tmp][i][p].solution_value()>0.5:
                    path.append(i)
                    m=i
                    break
        print(len(path)-1)
        for i in range(len(path)-1):
            print(path[i], end=" ")
        print()