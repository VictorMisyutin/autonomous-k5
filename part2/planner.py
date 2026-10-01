"""Step 8: A* on the grid with 4 moves (forward, right, back, left), with a penalty for turning."""
import heapq

# index == heading // 90  ->  0: forward(+y), 1: right(+x), 2: back(-y), 3: left(-x)
MOVES = [(0, 1), (1, 0), (0, -1), (-1, 0)]

TURN_COST = 3  # a 90 degree turn "costs" as much as 3 straight cells (a U-turn costs 2x this)


def astar(grid, start, goal, start_dir=0):
    """grid[y, x] == 1 is blocked. start/goal are (x, y) cells, start_dir = heading // 90.
    Returns a list of cells from start to goal, or None.

    Each search state is (cell, direction), so A* knows which way the car faces
    and can charge extra for turns. This makes it prefer straight routes.
    """
    h_max, w_max = grid.shape

    def h(c):
        return abs(c[0] - goal[0]) + abs(c[1] - goal[1])  # Manhattan distance

    start_state = (start, start_dir)
    open_heap = [(h(start), 0, start_state)]
    came_from = {start_state: None}
    g = {start_state: 0}
    end_state = None

    while open_heap:
        _, cost, state = heapq.heappop(open_heap)
        cell, d = state
        if cell == goal:
            end_state = state
            break
        if cost > g[state]:
            continue  # stale heap entry
        for nd, (dx, dy) in enumerate(MOVES):
            nxt = (cell[0] + dx, cell[1] + dy)
            if not (0 <= nxt[0] < w_max and 0 <= nxt[1] < h_max):
                continue
            if grid[nxt[1], nxt[0]]:
                continue
            turns = min((nd - d) % 4, (d - nd) % 4)  # 0 = straight, 1 = 90 deg, 2 = U-turn
            new_cost = cost + 1 + TURN_COST * turns
            new_state = (nxt, nd)
            if new_cost < g.get(new_state, float("inf")):
                g[new_state] = new_cost
                came_from[new_state] = state
                heapq.heappush(open_heap, (new_cost + h(nxt), new_cost, new_state))

    if end_state is None:
        return None  # ran out of cells without reaching goal

    path = []
    state = end_state
    while state is not None:
        path.append(state[0])
        state = came_from[state]
    return path[::-1]
