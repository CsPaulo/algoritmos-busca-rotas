# Importações
import heapq
from collections import deque

# Grafo da Romênia
graph = {
    "Arad": {"Zerind": 75, "Timisoara": 118, "Sibiu": 140},
    "Zerind": {"Arad": 75, "Oradea": 71},
    "Oradea": {"Zerind": 71, "Sibiu": 151},
    "Timisoara": {"Arad": 118, "Lugoj": 111},
    "Lugoj": {"Timisoara": 111, "Mehadia": 70},
    "Mehadia": {"Lugoj": 70, "Drobeta": 75},
    "Drobeta": {"Mehadia": 75, "Craiova": 120},
    "Craiova": {"Drobeta": 120, "Rimnicu Vilcea": 146, "Pitesti": 138},
    "Rimnicu Vilcea": {"Sibiu": 80, "Craiova": 146, "Pitesti": 97},
    "Sibiu": {"Arad": 140, "Oradea": 151, "Fagaras": 99, "Rimnicu Vilcea": 80},
    "Fagaras": {"Sibiu": 99, "Bucharest": 211},
    "Pitesti": {"Rimnicu Vilcea": 97, "Craiova": 138, "Bucharest": 101},
    "Bucharest": {"Fagaras": 211, "Pitesti": 101, "Giurgiu": 90, "Urziceni": 85},
    "Giurgiu": {"Bucharest": 90},
    "Urziceni": {"Bucharest": 85, "Hirsova": 98, "Vaslui": 142},
    "Hirsova": {"Urziceni": 98, "Eforie": 86},
    "Eforie": {"Hirsova": 86},
    "Vaslui": {"Urziceni": 142, "Iasi": 92},
    "Iasi": {"Vaslui": 92, "Neamt": 87},
    "Neamt": {"Iasi": 87}
}

# Heurística (distância em linha reta até Bucareste)
heuristic = {
    "Arad": 366, "Bucharest": 0, "Craiova": 160, "Drobeta": 242,
    "Eforie": 161, "Fagaras": 176, "Giurgiu": 77, "Hirsova": 151,
    "Iasi": 226, "Lugoj": 244, "Mehadia": 241, "Neamt": 234,
    "Oradea": 380, "Pitesti": 100, "Rimnicu Vilcea": 193, "Sibiu": 253,
    "Timisoara": 329, "Urziceni": 80, "Vaslui": 199, "Zerind": 374
}

# Funções de busca
def bfs(start, goal):
    queue = deque([(start, [start])])
    visited = set()

    while queue:
        current, path = queue.popleft()
        if current == goal:
            return path, len(path) - 1  # Distância como número de passos

        visited.add(current)

        for neighbor in graph[current]:
            if neighbor not in visited:
                queue.append((neighbor, path + [neighbor]))
                visited.add(neighbor)
    return None, None

def uniform_cost_search(start, goal):
    queue = [(0, start, [start])]
    visited = set()

    while queue:
        cost, current, path = heapq.heappop(queue)

        if current == goal:
            return path, cost

        if current in visited:
            continue
        visited.add(current)

        for neighbor in graph[current]:
            if neighbor not in visited:
                total_cost = cost + graph[current][neighbor]
                heapq.heappush(queue, (total_cost, neighbor, path + [neighbor]))
    return None, None

def dfs(start, goal):
    stack = [(start, [start])]
    visited = set()

    while stack:
        current, path = stack.pop()
        if current == goal:
            return path, len(path) - 1  # Distância como número de passos

        visited.add(current)

        for neighbor in graph[current]:
            if neighbor not in visited:
                stack.append((neighbor, path + [neighbor]))
    return None, None

def a_star(start, goal):
    queue = [(heuristic[start], 0, start, [start])]
    visited = set()

    while queue:
        est_total, cost_so_far, current, path = heapq.heappop(queue)

        if current == goal:
            return path, cost_so_far

        if current in visited:
            continue
        visited.add(current)

        for neighbor in graph[current]:
            if neighbor not in visited:
                new_cost = cost_so_far + graph[current][neighbor]
                est = new_cost + heuristic[neighbor]
                heapq.heappush(queue, (est, new_cost, neighbor, path + [neighbor]))
    return None, None

# Menu para o usuário
def main():
    print("Bem-vindo ao sistema de busca de rotas!")
    print("Cidades disponíveis:", ", ".join(graph.keys()))
    start = input("Digite a cidade de partida: ")
    goal = input("Digite a cidade de destino: ")

    if start not in graph or goal not in graph:
        print("Erro: Cidade inválida!")
        return

    print("\nEscolha o algoritmo de busca:")
    print("1 - Busca em Largura (BFS)")
    print("2 - Busca de Custo Uniforme")
    print("3 - Busca em Profundidade (DFS)")
    print("4 - Algoritmo A*")
    print("5 - Executar todas as buscas")
    choice = input("Digite o número do algoritmo: ")

    if choice == "1":
        path, cost = bfs(start, goal)
        print_result("Busca em Largura (BFS)", path, cost)
    elif choice == "2":
        path, cost = uniform_cost_search(start, goal)
        print_result("Busca de Custo Uniforme", path, cost)
    elif choice == "3":
        path, cost = dfs(start, goal)
        print_result("Busca em Profundidade (DFS)", path, cost)
    elif choice == "4":
        path, cost = a_star(start, goal)
        print_result("Algoritmo A*", path, cost)
    elif choice == "5":
        execute_all_searches(start, goal)
    else:
        print("Erro: Escolha inválida!")

def print_result(algorithm, path, cost):
    if path:
        print(f"\n{algorithm}:")
        print(f"  Caminho: {' -> '.join(path)}")
        print(f"  Número de cidades: {len(path)}")
        print(f"  Distância total: {cost}")
    else:
        print(f"\n{algorithm}: Nenhum caminho encontrado.")

def execute_all_searches(start, goal):
    print("\nExecutando todas as buscas:")
    algorithms = [
        ("Busca em Largura (BFS)", bfs),
        ("Busca de Custo Uniforme", uniform_cost_search),
        ("Busca em Profundidade (DFS)", dfs),
        ("Algoritmo A*", a_star),
    ]
    for name, func in algorithms:
        path, cost = func(start, goal)
        print_result(name, path, cost)

if __name__ == "__main__":
    main()
