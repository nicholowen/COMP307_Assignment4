import a4.utility as utility
import a4.loader as loader
import numpy as np


def main():
    # Paths to the data and solution files.
    vrp_file = "data/n32-k5.vrp"  # "data/n80-k10.vrp"
    sol_file = "data/n32-k5.sol"  # "data/n80-k10.sol"\
    # vrp_file = "data/n80-k10.vrp"
    # sol_file = "data/n80-k10.sol"

    # Loading the VRP data file.
    px, py, demand, capacity, depot = loader.load_data(vrp_file)

    # Displaying to console the distance and visualizing the optimal VRP solution.
    vrp_best_sol = loader.load_solution(sol_file)
    best_distance = utility.calculate_total_distance(vrp_best_sol, px, py, depot)
    print("Best VRP Distance:", best_distance)
    utility.visualise_solution(vrp_best_sol, px, py, depot, "Optimal Solution")

    # Executing and visualizing the nearest neighbour VRP heuristic.
    # Uncomment it to do your assignment!

    nnh_solution = nearest_neighbour_heuristic(px, py, demand, capacity, depot)
    # nnh_distance = utility.calculate_total_distance(nnh_solution, px, py, depot)
    # print("Nearest Neighbour VRP Heuristic Distance:", nnh_distance)
    # utility.visualise_solution(nnh_solution, px, py, depot, "Nearest Neighbour Heuristic")

    # Executing and visualizing the saving VRP heuristic.
    # Uncomment it to do your assignment!

    sh_solution = savings_heuristic(px, py, demand, capacity, depot)
    sh_distance = utility.calculate_total_distance(sh_solution, px, py, depot)
    print("Saving VRP Heuristic Distance:", sh_distance)
    utility.visualise_solution(sh_solution, px, py, depot, "Savings Heuristic")


def nearest_neighbour_heuristic(px, py, demand, capacity, depot):
    """
    Algorithm for the nearest neighbour heuristic to generate VRP solutions.

    :param px: List of X coordinates for each node.
    :param py: List of Y coordinates for each node.
    :param demand: List of each nodes demand.
    :param capacity: Vehicle carrying capacity.
    :param depot: Depot.
    :return: List of vehicle routes (tours).
    """

    # TODO - Implement the Nearest Neighbour Heuristic to generate VRP solutions.

    # NN VRP should be 1146.40

    nodes, visited, solution, route = [], [], [], []
    demand_total = 0
    start = depot
    for i in range(1, len(px)):
        nodes.append(i)

    # print(nodes)

    while len(nodes) > 0:
        n = feasible_nodes(px, py, demand, demand_total, capacity, start, nodes)
        # n = utility.find_closest_node(px, py, start, nodes)
        if n != 0:
            if demand_total + demand[n] <= capacity:
                start = n
                nodes.remove(n)
                visited.append(n)
                route.append(n)
                demand_total += demand[n]

                if len(nodes) == 0:
                    route.append(n)
                    solution.append(route)
        else:
            solution.append(route)
            route = []
            start = depot
            demand_total = 0

    return solution


def feasible_nodes(px, py, demand, total_demand, capacity, node, nodes):
    min_dist = 9999999
    index = 0
    for n in nodes:
        distance = utility.calculate_euclidean_distance(px, py, node, n)
        if distance > 0:
            if distance <= min_dist:
                if (total_demand + demand[n]) <= capacity:
                    min_dist = distance
                    index = n

    return index


def savings_heuristic(px, py, demand, capacity, depot):
    """
    Algorithm for Implementing the savings heuristic to generate VRP solutions.

    :param px: List of X coordinates for each node.
    :param py: List of Y coordinates for each node.
    :param demand: List of each nodes demand.
    :param capacity: Vehicle carrying capacity.
    :param depot: Depot.
    :return: List of vehicle routes (tours).
    """

    # TODO - Implement the Saving Heuristic to generate VRP solutions.

    # saving VRP should be 843.69

    routes, route, = [], []
    for i in range(1, len(px)):
        route.append(i)
        routes.append(route)
        route = []

    can_merge = True
    while can_merge:

        savings = np.zeros((len(routes), len(routes)))

        route_distance = []
        for r in routes:
            if len(r) > 0:
                route_distance.append(utility.calculate_total_distance([r], px, py, depot))

        for i in range(len(routes)):
            for j in range(len(routes)):
                a = len(routes[i]) - 1
                if i == j:
                    savings[i][j] = 0
                else:
                    savings[i][j] = utility.calculate_euclidean_distance(px, py, routes[i][a], depot) + \
                                    utility.calculate_euclidean_distance(px, py, depot, routes[j][0]) - \
                                    utility.calculate_euclidean_distance(px, py, routes[i][a], routes[j][0])

        best_savings = 0
        best_i = 0
        best_j = 0
        for i in range(len(routes)):
            for j in range(len(routes)):
                demand1 = get_total_demand(demand, routes[i])
                demand2 = get_total_demand(demand, routes[j])
                if (savings[i][j] > best_savings or savings[j][i] > best_savings) and demand1 + demand2 <= capacity:
                    best_savings = savings[i][j]
                    best_i = i
                    best_j = j

        if best_i != best_j:
            routes[best_i].extend(routes[best_j])
            routes.remove(routes[best_j])

        can_merge = False
        for i in range(len(routes)):
            for j in range(len(routes)):
                if i != j:
                    if get_total_demand(demand, routes[i]) + get_total_demand(demand, routes[j]) <= capacity:
                        can_merge = True

    return routes


def get_total_demand(demand, route):
    d = 0
    for i in route:
        d += demand[i]

    return d


if __name__ == '__main__':
    main()
