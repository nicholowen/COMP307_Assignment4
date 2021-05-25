import a4.utility as utility
import a4.loader as loader
import numpy as np


def main():
    # Paths to the data and solution files.
    vrp_file = "data/n32-k5.vrp"  # "data/n80-k10.vrp"
    sol_file = "data/n32-k5.sol"  # "data/n80-k10.sol"

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

    # sh_solution = savings_heuristic(px, py, demand, capacity, depot)
    # sh_distance = utility.calculate_total_distance(sh_solution, px, py, depot)
    # print("Saving VRP Heuristic Distance:", sh_distance)
    # utility.visualise_solution(sh_solution, px, py, depot, "Savings Heuristic")


class node:
    def __init__(self, x, y, visited, demand):
        self.x = x
        self.y = y
        self.visited = visited
        self.demand = demand

    def print(self):
        print("X: ", self.x, " Y: ", self.y)


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
    # initialise node objects
    root_node = node(px[0], px[0], False, demand[0])
    nodes = list()
    for i in range(1, len(px)):
        nodes.append(node(px[i], py[i], False, demand[i]))

    # TODO - Implement the Nearest Neighbour Heuristic to generate VRP solutions.

    # NN VRP should be 1146.40
    visited = list()
    solution = list()
    total = 0
    demand_total = 0
    start = root_node
    counter = 0
    while len(nodes) != 0:
        for n in nodes:
            if n not in visited:
                a, b = utility.find_closest_node(start, nodes)
                start = n
                if demand_total + a.demand <= 100:
                    demand_total += a.demand
                    visited.append(a)
                    nodes.remove(a)
                    solution.append(a)
                    total += b
                else:
                    start = root_node
                    demand_total = 0
                    counter += 1

    print(total)
    print(counter)

    return None


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

    return None


if __name__ == '__main__':
    main()
