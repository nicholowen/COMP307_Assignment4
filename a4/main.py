import a4.utility as utility
import a4.loader as loader
import numpy as np


def main():
    # Paths to the data and solution files.
    vrp_file = "data/n32-k5.vrp"
    sol_file = "data/n32-k5.sol"
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
    #
    nnh_solution = nearest_neighbour_heuristic(px, py, demand, capacity, depot)
    nnh_distance = utility.calculate_total_distance(nnh_solution, px, py, depot)
    print("Nearest Neighbour VRP Heuristic Distance:", nnh_distance)
    utility.visualise_solution(nnh_solution, px, py, depot, "Nearest Neighbour Heuristic")

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

    nodes, visited, solution, route = [], [], [], []
    demand_total = 0
    start = depot
    for i in range(1, len(px)):
        nodes.append(i)

    # print(nodes)

    while len(nodes) > 0:
        # checks the closest node that does not exceed capacity
        n = feasible_node(px, py, demand, demand_total, capacity, start, nodes)

        if n != 0:
            start = n
            nodes.remove(n)
            visited.append(n)
            route.append(n)
            demand_total += demand[n]

            # no more nodes left, append the current (and final) route to the solution
            if len(nodes) == 0:
                route.append(n)
                solution.append(route)

        # a route has been found - add it to the solution
        else:
            solution.append(route)
            route = []
            start = depot
            demand_total = 0

    print(solution)

    return solution

# searches all nodes and returns the closest node that does not exceed capacity
def feasible_node(px, py, demand, total_demand, capacity, node, nodes):
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
    # generate a list containing a list for each node
    routes, route, = [], []
    for i in range(1, len(px)):
        route.append(i)
        routes.append(route)
        route = []

    can_merge = True
    # while there are still routes that can be merged and not exceed capacity)
    while can_merge:
        # create a square matrix of zeroes
        savings = np.zeros((len(routes), len(routes)))

        # traverse the matrix and record the best saving for each set of routes - i,j
        for i in range(len(routes)):
            for j in range(len(routes)):
                end = len(routes[i]) - 1
                if i == j:
                    # the same node
                    savings[i][j] = 0
                else:
                    # using equation: savings = L(v1, 1) + L(1, v2) / L(v1 - v2)
                    savings[i][j] = utility.calculate_euclidean_distance(px, py, routes[i][end], depot) + \
                                    utility.calculate_euclidean_distance(px, py, depot, routes[j][0]) - \
                                    utility.calculate_euclidean_distance(px, py, routes[i][end], routes[j][0])

        # initialise values to find the best combination of i and j nodes
        best_savings = 0
        best_i = 0
        best_j = 0
        for i in range(len(routes)):
            for j in range(len(routes)):
                # gets the total demand (used capacity) of each route being checked
                demand1 = get_total_demand(demand, routes[i])
                demand2 = get_total_demand(demand, routes[j])
                # ensures the best combination of i and j
                if savings[i][j] > best_savings and demand1 + demand2 <= capacity:
                    best_savings = savings[i][j]
                    best_i = i
                    best_j = j

        # merges the routes in the i, j order
        routes[best_i].extend(routes[best_j])
        routes.remove(routes[best_j])

        # this check is the loop exit condition
        can_merge = False
        for i in range(len(routes)):
            for j in range(len(routes)):
                if i != j:
                    # check every route and see if there are any combination of routes that won't exceed capacity
                    if get_total_demand(demand, routes[i]) + get_total_demand(demand, routes[j]) <= capacity:
                        can_merge = True

    print(routes)
    return routes

# returns the total demand of a given route
def get_total_demand(demand, route):
    d = 0
    for i in route:
        d += demand[i]

    return d


if __name__ == '__main__':
    main()
