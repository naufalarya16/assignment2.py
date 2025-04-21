import heapq
import time

def greedy_best_first_search(grid, start, goal):
    start_time = time.time()
    rows, cols = len(grid), len(grid[0])
    
    def heuristic(pos):
        x1, y1 = pos
        x2, y2 = goal
        return abs(x2 - x1) + abs(y2 - y1)
    
    directions = [(-1, 0), (0, 1), (1, 0), (0, -1)]
    open_set = []
    closed_set = set()
    visited_count = 0
    parent = {}

    heapq.heappush(open_set, (heuristic(start), start))

    while open_set:
        _, current = heapq.heappop(open_set)
        visited_count += 1

        if current == goal:
            path = []
            while current != start:
                path.append(current)
                current = parent[current]
            path.append(start)
            path.reverse()
            time_taken = (time.time() - start_time) * 1000
            return path, visited_count, time_taken

        if current in closed_set:
            continue
        closed_set.add(current)

        for dx, dy in directions:
            nx, ny = current[0] + dx, current[1] + dy
            neighbor = (nx, ny)

            if 0 <= nx < rows and 0 <= ny < cols and grid[nx][ny] != 'T' and neighbor not in closed_set:
                parent[neighbor] = current
                heapq.heappush(open_set, (heuristic(neighbor), neighbor))

    return None, visited_count, (time.time() - start_time) * 1000

def a_star_search(grid, start, goal):
    start_time = time.time()
    rows, cols = len(grid), len(grid[0])

    def heuristic(pos):
        x1, y1 = pos
        x2, y2 = goal
        return abs(x2 - x1) + abs(y2 - y1)

    directions = [(-1, 0), (0, 1), (1, 0), (0, -1)]
    open_set = []
    closed_set = set()
    g_score = {start: 0}
    visited_count = 0
    parent = {}

    heapq.heappush(open_set, (heuristic(start), start))

    while open_set:
        _, current = heapq.heappop(open_set)
        visited_count += 1

        if current == goal:
            path = []
            while current != start:
                path.append(current)
                current = parent[current]
            path.append(start)
            path.reverse()
            return path, visited_count, (time.time() - start_time) * 1000

        if current in closed_set:
            continue
        closed_set.add(current)

        for dx, dy in directions:
            nx, ny = current[0] + dx, current[1] + dy
            neighbor = (nx, ny)

            if 0 <= nx < rows and 0 <= ny < cols and grid[nx][ny] != 'T':
                tentative_g = g_score[current] + 1
                if tentative_g < g_score.get(neighbor, float('inf')):
                    parent[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor)
                    heapq.heappush(open_set, (f_score, neighbor))

    return None, visited_count, (time.time() - start_time) * 1000

def visualize_map(city_map, gbfs_path=None, a_star_path=None):
    visual_map = [row[:] for row in city_map]

    # Tandai jalur GBFS dengan '+'
    if gbfs_path:
        for x, y in gbfs_path:
            if visual_map[x][y] not in ['S', 'H']:
                visual_map[x][y] = '+'

    # Tandai jalur A* dengan '*'
    if a_star_path:
        for x, y in a_star_path:
            if visual_map[x][y] not in ['S', 'H', '+']:  # Jangan timpa S/H atau +
                visual_map[x][y] = '*'

    print()
    for row in visual_map:
        print(" ".join(row))

if __name__ == "__main__":
    city_map = [
        [".", ".", ".", ".", ".", ".", ".", ".", "."],
        [".", "S", ".", ".", "T", "T", ".", ".", "."],
        [".", ".", ".", ".", "T", ".", ".", ".", "."],
        [".", "T", "T", "T", "T", ".", "T", "T", "."],
        [".", ".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", "T", "T", ".", "T", "T", ".", "."],
        [".", ".", ".", ".", ".", ".", "T", ".", "."],
        [".", "T", ".", "T", "T", ".", ".", "H", "."],
        [".", ".", ".", ".", ".", ".", ".", ".", "."]
    ]

    start = goal = None
    for i in range(len(city_map)):
        for j in range(len(city_map[0])):
            if city_map[i][j] == 'S':
                start = (i, j)
            elif city_map[i][j] == 'H':
                goal = (i, j)

    if start is None or goal is None:
        print("Posisi ambulan atau rumah sakit tidak ditemukan!")
    else:
        print(f"Ambulan di: {start}")
        print(f"Rumah sakit di: {goal}")

        print("\nGreedy Best-First Search:")
        gbfs_path, gbfs_visited, gbfs_time = greedy_best_first_search(city_map, start, goal)
        if gbfs_path:
            print(f"  Jalur ditemukan ({len(gbfs_path)} langkah)")
            print(f"  Node dikunjungi: {gbfs_visited}")
            print(f"  Waktu eksekusi: {gbfs_time:.2f} ms")
        else:
            print("  Jalur tidak ditemukan.")

        print("\nA* Search:")
        a_star_path, a_star_visited, a_star_time = a_star_search(city_map, start, goal)
        if a_star_path:
            print(f"  Jalur ditemukan ({len(a_star_path)} langkah)")
            print(f"  Node dikunjungi: {a_star_visited}")
            print(f"  Waktu eksekusi: {a_star_time:.2f} ms")
        else:
            print("  Jalur tidak ditemukan.")

        print("\nVisualisasi Jalur (GBFS = '+', A* = '*'):")
        visualize_map(city_map, gbfs_path, a_star_path)
