import sys
import random
import math
import time


def read_input():
    try:
        data = sys.stdin.read().strip().split()
        it = iter(data)
        N, K = int(next(it)), int(next(it))
        d = [[0.0] * (N + 1) for _ in range(N + 1)]
        for i in range(N + 1):
            for j in range(N + 1):
                d[i][j] = float(next(it))
        return N, K, d
    except:
        return 0, 0, []


def route_length(route, d):
    return sum(d[route[i]][route[i + 1]] for i in range(len(route) - 1))


def objective(routes, d):
    return max(route_length(r, d) for r in routes)


def initial_solution(N, K, d):
    visited = [False] * (N + 1)
    routes = [[] for _ in range(K)]
    unassigned = set(range(1, N + 1))

    for k in range(K):
        current = 0
        route = [0]
        while len(route) < 1 + N // K + 1 and unassigned:
            nearest = min(unassigned, key=lambda j: d[current][j])
            route.append(nearest)
            unassigned.remove(nearest)
            current = nearest
        routes[k] = route
    # Gán phần còn lại
    leftover = list(unassigned)
    for i, node in enumerate(leftover):
        routes[i % K].append(node)
    return routes


def random_removal(routes, remove_count):
    all_nodes = [(k, node) for k, rt in enumerate(routes) for node in rt if node != 0]
    to_remove = random.sample(all_nodes, min(remove_count, len(all_nodes)))
    removed = []
    for k, node in to_remove:
        if node in routes[k]:
            routes[k].remove(node)
            removed.append(node)
    return removed


def greedy_repair(routes, removed, d):
    K = len(routes)
    lens = [route_length(r, d) for r in routes]
    for node in removed:
        best_obj = float('inf')
        best_k, best_pos = None, None
        for k in range(K):
            rt, L = routes[k], lens[k]
            for pos in range(1, len(rt) + 1):
                prev = rt[pos - 1]
                nxt = rt[pos] if pos < len(rt) else None
                gain = d[prev][node] + (d[node][nxt] if nxt else 0) - (d[prev][nxt] if nxt else 0)
                new_len = L + gain
                max_len = max(new_len if i == k else lens[i] for i in range(K))
                if max_len < best_obj:
                    best_obj, best_k, best_pos = max_len, k, pos
        routes[best_k].insert(best_pos, node)
        lens[best_k] = route_length(routes[best_k], d)
    return routes


def two_opt(route, d):
    improved = True
    while improved:
        improved = False
        for i in range(1, len(route) - 2):
            for j in range(i + 1, len(route) - 1):
                a, b = route[i - 1], route[i]
                c, d2 = route[j], route[j + 1]
                delta = d[a][c] + d[b][d2] - d[a][b] - d[c][d2]
                if delta < -1e-6:
                    route[i:j + 1] = reversed(route[i:j + 1])
                    improved = True
                    break
            if improved:
                break
    return route


def cheap_relocate(routes, d, sample_size=3):
    K = len(routes)
    lens = [route_length(r, d) for r in routes]
    i_long = max(range(K), key=lambda k: lens[k])
    i_short = min(range(K), key=lambda k: lens[k])
    long_rt = routes[i_long]
    short_rt = routes[i_short]
    best_gain = 0
    best_move = None

    if len(long_rt) <= 2:
        return routes

    candidates = random.sample(long_rt[1:], min(sample_size, len(long_rt) - 1))
    for node in candidates:
        idx = long_rt.index(node)
        prev = long_rt[idx - 1]
        nxt = long_rt[idx + 1] if idx + 1 < len(long_rt) else None
        remove_cost = d[prev][node] + (d[node][nxt] if nxt else 0) - (d[prev][nxt] if nxt else 0)

        for pos in range(1, len(short_rt) + 1):
            p = short_rt[pos - 1]
            n = short_rt[pos] if pos < len(short_rt) else None
            insert_cost = d[p][node] + (d[node][n] if n else 0) - (d[p][n] if n else 0)
            new_long = lens[i_long] - remove_cost
            new_short = lens[i_short] + insert_cost
            old_max = max(lens[i_long], lens[i_short])
            new_max = max(new_long, new_short)
            gain = old_max - new_max
            if gain > best_gain:
                best_gain = gain
                best_move = (node, idx, i_long, pos, i_short)

    if best_move:
        node, idx, il, pos, isr = best_move
        routes[il].pop(idx)
        routes[isr].insert(pos, node)
    return routes


def alns_minmax_vrp(N, K, d, time_limit=2.5):
    T0 = 1.0
    alpha = 0.995
    p_remove = 0.1
    no_improve = 0
    best = initial_solution(N, K, d)
    best_obj = objective(best, d)
    current = [r[:] for r in best]
    current_obj = best_obj
    T = T0
    end_time = time.time() + time_limit

    while time.time() < end_time:
        candidate = [r[:] for r in current]
        remove_count = max(1, int(p_remove * N))
        removed = random_removal(candidate, remove_count)
        candidate = greedy_repair(candidate, removed, d)

        route_lens = [(route_length(candidate[k], d), k) for k in range(K)]
        route_lens.sort(reverse=True)
        for _, k in route_lens[:max(1, K // 2)]:
            candidate[k] = two_opt(candidate[k], d)

        candidate = cheap_relocate(candidate, d)

        cand_obj = objective(candidate, d)

        if cand_obj < current_obj or random.random() < math.exp((current_obj - cand_obj) / T):
            current, current_obj = candidate, cand_obj
            if cand_obj < best_obj:
                best, best_obj = [r[:] for r in candidate], cand_obj
                no_improve = 0
            else:
                no_improve += 1
        else:
            no_improve += 1

        if no_improve >= 3:
            p_remove = min(0.3, p_remove + 0.05)
        else:
            p_remove = max(0.1, p_remove - 0.01)

        T *= alpha

    return best, best_obj


def output_solution(sol):
    print(len(sol))
    for r in sol:
        print(len(r))
        print(" ".join(str(x) for x in r))


def main():
    N, K, d = read_input()
    if N == 0 or K == 0:
        print(0)
        return

    start_time = time.time()
    total_time = 24.5
    run_time = 2.5
    best_sol = None
    best_obj = float('inf')

    while time.time() - start_time + run_time < total_time:
        sol, obj = alns_minmax_vrp(N, K, d, time_limit=run_time)
        if obj < best_obj:
            best_sol = sol
            best_obj = obj

    output_solution(best_sol)

if __name__ == "__main__":
    main()
