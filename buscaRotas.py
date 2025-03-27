import heapq
from collections import deque

# Representação do grafo com distâncias entre os nós
graph = {
    "Arad": {"Zerind": 75, "Timisoara": 118, "Sibiu": 140},
    "Zerind": {"Oradea": 71, "Arad": 75},
    "Oradea": {"Zerind": 71, "Sibiu": 151},
    "Timisoara": {"Arad": 118, "Lugoj": 111},
    "Lugoj": {"Timisoara": 111, "Mehadia": 70},
    "Mehadia": {"Lugoj": 70, "Drobeta": 75},
    "Drobeta": {"Mehadia": 75, "Craiova": 120},
    "Craiova": {"Drobeta": 120, "Rimnicu Vilcea": 146, "Pitesti": 138},
    "Sibiu": {"Arad": 140, "Oradea": 151, "Rimnicu Vilcea": 80, "Fagaras": 99},
    "Rimnicu Vilcea": {"Sibiu": 80, "Craiova": 146, "Pitesti": 97},
    "Fagaras": {"Sibiu": 99, "Bucharest": 211},
    "Pitesti": {"Rimnicu Vilcea": 97, "Craiova": 138, "Bucharest": 101},
    "Bucharest": {"Fagaras": 211, "Pitesti": 101, "Giurgiu": 90, "Urziceni": 85},
    "Giurgiu": {"Bucharest": 90},
    "Urziceni": {"Bucharest": 85, "Vaslui": 142, "Hirsova": 98},
    "Vaslui": {"Iasi": 92, "Urziceni": 142},
    "Iasi": {"Vaslui": 92, "Neamt": 87},
    "Neamt": {"Iasi": 87},
    "Hirsova": {"Urziceni": 98, "Eforie": 86},
    "Eforie": {"Hirsova": 86}
}

# Função para busca em largura (amplitude)
def bfs(start, goal):
    queue = deque([(start, [start], 0)])
    visited = set()
    while queue:
        node, path, cost = queue.popleft()
        if node == goal:
            return path, cost
        if node not in visited:
            visited.add(node)
            for neighbor, distance in graph[node].items():
                queue.append((neighbor, path + [neighbor], cost + distance))
    return None

# Função para busca de custo uniforme
def uniform_cost_search(start, goal):
    pq = [(0, start, [start])]
    visited = set()
    while pq:
        cost, node, path = heapq.heappop(pq)
        if node == goal:
            return path, cost
        if node not in visited:
            visited.add(node)
            for neighbor, distance in graph[node].items():
                heapq.heappush(pq, (cost + distance, neighbor, path + [neighbor]))
    return None

# Função para busca em profundidade
def dfs(start, goal, depth_limit=None):
    stack = [(start, [start], 0)]
    visited = set()
    while stack:
        node, path, cost = stack.pop()
        if node == goal:
            return path, cost
        if node not in visited:
            visited.add(node)
            if depth_limit is None or len(path) - 1 < depth_limit:
                for neighbor, distance in graph[node].items():
                    stack.append((neighbor, path + [neighbor], cost + distance))
    return None

# Função para busca de aprofundamento iterativo
def iterative_deepening(start, goal):
    depth = 0
    while True:
        result = dfs(start, goal, depth)
        if result:
            return result
        depth += 1

# Função para busca gulosa
def greedy_search(start, goal, heuristic):
    pq = [(heuristic[start], start, [start], 0)]
    visited = set()
    while pq:
        _, node, path, cost = heapq.heappop(pq)
        if node == goal:
            return path, cost
        if node not in visited:
            visited.add(node)
            for neighbor, distance in graph[node].items():
                heapq.heappush(pq, (heuristic[neighbor], neighbor, path + [neighbor], cost + distance))
    return None

# Função para busca A*
def a_star_search(start, goal, heuristic):
    pq = [(heuristic[start], 0, start, [start])]
    visited = set()
    while pq:
        _, cost, node, path = heapq.heappop(pq)
        if node == goal:
            return path, cost
        if node not in visited:
            visited.add(node)
            for neighbor, distance in graph[node].items():
                new_cost = cost + distance
                heapq.heappush(pq, (new_cost + heuristic[neighbor], new_cost, neighbor, path + [neighbor]))
    return None

# Heurística inicial (substituir por valores reais, se necessário)
heuristic = {city: 0 for city in graph}

# Função para validar a entrada do usuário
def get_valid_city(prompt):
    while True:
        city = input(prompt).strip()
        if city in graph:
            return city
        print("Cidade inválida. Por favor, digite uma cidade válida.")

# Solicita ao usuário os pontos de partida e destino
start = get_valid_city("Digite a cidade de partida: ")
goal = get_valid_city("Digite a cidade de destino: ")

# Exibe os resultados das buscas
print("\nResultados das buscas:")
print("1. Sem informação (buscas cegas):")
print("1.1. Busca em extensão (BFS):", bfs(start, goal))
print("1.2. Busca de custo uniforme:", uniform_cost_search(start, goal))
print("1.3. Busca em profundidade (DFS):", dfs(start, goal))
print("1.4. Busca em profundidade limitada (limite = 5):", dfs(start, goal, 5))
print("1.5. Busca de aprofundamento iterativo:", iterative_deepening(start, goal))

print("\n2. Com informação (buscas heurísticas):")
print("2.1. Busca gulosa:", greedy_search(start, goal, heuristic))
print("2.2. Algoritmo A*:", a_star_search(start, goal, heuristic))
