# algorithim
# import that allows priority queue and calculations
import heapq
import math

# interface import
from campus_nav import heuristic, get_neighbors, edge_cost, all_nodes

def astar(start, goal):
    """
    Locates the cheapest cost route to take from the starting point to the 
    desired goal point with A*
    
    Usage:
    path             # list of node ids including start and goal, or [] (exmpty) 
                     # if no route (from interface)
    nodes_expanded   # is how many nodes we popped off the queue - 
                     # that's the number we plot (straight from interface)
    """
    # PQ is responsible for choosing with nodes to move on to and expand.
    # estimated goal cost, actual cost, node is included.
    # f(n) = g(n) + h(n) (f(n): ?? g(n): ?? h(n): ??)
    priority_queue = []

    # actual cost at the start is 0
    starting_cost = 0

    # estimate total cost of travels from the start point to the desired goal
    # point.
    # no movement means: f(start) = 0 + heuristic(start,goal)
    starting_priority = starting_cost + heuristic(start, goal)

    # add the strting node to PQ
    # heapq should place tuple with the smallest value first at the front
    heapq.heappush(
        priority_queue,
        (starting_priority, starting_cost, start)
    )

    # sofarcost: stores the lowest cost to date from the starting point
    # (A* found so far)
    # start node to iself is 0
    sofarcost = {
        start: 0
    }

    # came: remembers the prior node that was used to come to this node
    # starting node has no prior node
    came = {
        start: None
    }

    # counts nodes explored by A*
    nodes_expanded = 0

    # keeps track on how many nodes A* removes from PQ
    while priority_queue:

        # removes smallest estimated total cost
        # three values from tuple go into current_priority, current_cost,
        # and current
        # heappop: remove & return node with lowest cost to explore
        current_priority, current_cost, current = heapq.heappop(
            priority_queue
        )

        # if same node appears more than once within the heap, (e.g. could find 
        # it within a costly path the first time and then a not so costly path
        # another time so the more expensive entry remains in heeap so we can
        # skip and move on when reached)
        if current_cost != sofarcost[current]:
            # move on
            continue
            
        # node is explored now
        nodes_expanded += 1

        # if current state is equaled to goal point, lowest cost path is found
        if current == goal:

            # list will be held in reverse order (goal to start)
            path = []
            
            # starts at goal point and goes backwards alongside came 
            while current is not None:
                # add current node to path
                # append: add to back of list
                path.append(current)
                # move backwards to prior node
                current = came[current]

            # reverse so it goes start to goal
            path.reverse()

            # return path and expansion
            return path, nodes_expanded
        
        # go through neighbor nodes from current
        for neighbor in get_neighbors(current):

            # retrieve number of seconds to move through edge
            move_cost = edge_cost(current, neighbor)

            # blocked edge = infinity cost
            # skipped
            # math.isinf: check if it does not exist or have numerous cost values
            if math.isinf(move_cost):
                # move on
                continue

            # calculate cost of reaching neighbor node through the current
            new_cost = current_cost + move_cost

            # update neighbor if it was never found earlier or the route is cheaper
            # than a prior route
            if (
                neighbor not in sofarcost or new_cost < sofarcost[neighbor]
            ):
                # store the new discovered cheaper cost to neighbor
                sofarcost[neighbor] = new_cost

                # reached neighbor node from current node
                came[neighbor] = current

                # calculate A* priority with neighbor node
                # f(n) = g(n) + h(n)
                estimated_total_cost = (
                    new_cost + heuristic(neighbor, goal)
                )

                # add neighbor to PQ and heapq will arrange accordignly, 
                # which should allow the node with smallest estimated total cost
                # to be removed
                # heappush: add to PQ
                heapq.heappush(
                    priority_queue,
                    (estimated_total_cost, new_cost, neighbor)
                )

    # if PQ is empty, A* could not find goal despite checking all 
    # route options
    return [], nodes_expanded


# class which replans cheaply when the map changes while you're 
# already walking
class DStarLite:
    """
    D* Lite class:
    ...
    """
    def __init__(self, start, goal):
        """
        Implements g and rhs value tracking per node.
        """

        # stores starting node
        self.start = start
        # stores goal node
        self.goal = goal
        # stores rhs (one step lookahead cost for each node)
        # rhs: what D* thinks the cost should be after exploring neighbor nodes
        self.rhs = {}
        # stores g (best known cost to goal for each node)
        # g: what the D* belives the cost is
        self.g = {}
        # 0 because the starting node has not shifted or traveled yet
        self.km = 0

        # node starts off with unknown cost so starts
        # inifinity since D* hasn't searched yet
        for node in all_nodes():
            self.g[node] = float("inf")
            self.rhs[node] = float("inf")

        # goal is at destination so rhs is 0
        self.rhs[self.goal] = 0

        # PQ for reverse search which begins at goal
        self.priority_queue = []
        # calculate two-part key (for the goal)
        goal_key = self.calculate_key(self.goal)
        # add goal to the PQ
        # heapq compares the first key first, and if 2 nodes have the same
        # first key, then it compares second key (first key determines priority, 
        # so it reoves the node with the smallest key first in order to explore)
        # D* Lite can begin its search from the goal node
        heapq.heappush(
            self.priority_queue,
            (goal_key[0], goal_key[1], self.goal)
        )

    def is_consistent(self, node):
        """
        Determines a node's consistency.
    
        Returns:
        True if g and rhs values are equal to one another.
        False, otherwise.
        """
        return self.g[node] == self.rhs[node]
    
    def calculate_key(self, node):
        """
        Calculates two-part key for a node.
        """
        # pick the lowest value between g and rhs
        smaller_cost = min(self.g[node], self.rhs[node])
        # first part selects cost, heuristic from current start to node, km
        firstkey = smaller_cost + heuristic(self.start, node) + self.km
        # second part is the lowest of g and rhs
        secondkey = smaller_cost
        # return two-part key as tuple
        return (firstkey, secondkey)

    def updatingkm(self, newstart):
        """
        Updates km when starting node travels and moves.
        """
        # save the previous starting node before changing
        previousstart = self.start
        # increase km (via heuristic distance from old to new starting node)
        # (heuristic(a, b) from interface)
        # += adds on current value
        # km stores total distance the start has moved since start of search
        self.km += heuristic(previousstart, newstart)
        # update starting point
        # D* remembers how far the start has shifted instead of restarting the
        # search every time the user/agent moves to next node (A*)
        self.start = newstart

    # helper for recalculation and updates node
    def updaterhs(self, node):
        """
        Updates rhs for node and adds it to the PQ if it is
        not consistent with g
        Additionally, makes sure the node info is accurate and if 
        it is not then makes sure it is in PQ so D* can fix it.
        """

        # rhs 0 at goal
        if node != self.goal:

            # assume no known path
            lowestrhs = float("inf")

            # check neighboring nodes
            for neighbor in get_neighbors(node):

                # cost from current node to neighbor
                movecost = edge_cost(node, neighbor)

                # possible cost if travel through neighbor
                possiblerhs = movecost + self.g[neighbor]

                # keep lowest possible rhs
                if possiblerhs < lowestrhs:
                    lowestrhs = possiblerhs

            # save the best one-step lookahead value
            self.rhs[node] = lowestrhs

        # if g and rhs are not the same, update node
        if not self.is_consistent(node):

            # calculate 2-part key for node
            nodekey = self.calculate_key(node)

            # put node in PQ
            heapq.heappush(
                self.priority_queue,
                (nodekey[0], nodekey[1], node)
            )

    def computeshortestpath(self):
        """
        Main loop for D* which repairs the most shortest path
        """

        # counts the amount of nodes in D* explores
        nodes_expanded = 0

        # continues search as nodes go to PQ
        while self.priority_queue:

            # get smallest key in PQ
            # 0 is ?? and 1 is ??
            # top key: ??
            topkey = (
                self.priority_queue[0][0],
                self.priority_queue[0][1]
            )

            # calculate current key (for start)
            # start key: ??
            startkey = self.calculate_key(self.start)

            # if smallest key is not better then start key, then stop
            # if start node is consistent, stop
            if topkey >= startkey and self.is_consistent(self.start):
                break

            # remove node with smallest key from PQ
            # heappop: ??
            oldfirstkey, oldsecondkey, current = heapq.heappop(
                self.priority_queue
            )

            # store old key (tuple)
            oldkey = (oldfirstkey, oldsecondkey)

            # calculate node's current key
            newkey = self.calculate_key(current)

            # if old queue key is outdated add it back with the new key
            if oldkey < newkey:
                heapq.heappush(
                    self.priority_queue,
                    (newkey[0], newkey[1], current)
                )
                continue

            # node is being explored
            nodes_expanded += 1

            # rhs is better than g, update g
            # g should then match rhs
            if self.g[current] > self.rhs[current]:

                # ??
                self.g[current] = self.rhs[current]

                # update neighbor nodes (rhs values could be dependent on current)
                for neighbor in get_neighbors(current):
                    self.updaterhs(neighbor)

            else:

                # old g value isn't reliable to depend on
                self.g[current] = float ("inf")

                # update current
                self.updaterhs(current)

                # update neighbor nodes
                for neighbor in get_neighbors(current):
                    self.updaterhs(neighbor)

        # return nodes explored (during this round)
        return nodes_expanded


    def step_to(self, node):
        """
        Moves agent/user one node forward and updates both the start and km
        Keeps the steps updated and accurate
        """

        # checks to make sure node is connected to current start
        # current start: ??
        if node not in get_neighbors(self.start):
            raise ValueError("only can move forward to a node that is neighboring")

        # update km and current start node to new node
        self.updatingkm(node)

    def notify_edge_change(self, a, b, oldcost):
        """
        Keeps track of nodes on a change edge that need to be updated
        """

        # updates very first node because the edge may affect rhs (change edge)
        # changed rhs: ??
        # a: ??
        self.updaterhs(a)

        # update the second node for same reason
        # b: ??
        self.updaterhs(b)

    def planroute(self):
        """
        Computes D* route and returns path and nodes expanded.
        """

        # repair shortest path info
        nodes_expanded = self.computeshortestpath()

        # if no path exists, return empty
        if math.isinf(self.g[self.start]):
            return [], nodes_expanded

        # begin route at current start
        path = [self.start]
        current = self.start

        # goes on until goal met
        while current != self.goal:

            # locate the neighbor with cheapest cost to goal
            current = min(
                get_neighbors(current),
                # lambda: ??
                key=lambda neighbor: edge_cost(current, neighbor) + self.g[neighbor]
            )

            # add chosen node to path
            path.append(current)

        # return route and nodes explored
        return path, nodes_expanded


# temp test case
if __name__  == "__main__":
    print("successful run")

    path, nodes = astar("snell_library", "curry_student_center")
    print("path:", path)
    print("nodes:", nodes)

# temp test case with D* day 8
if __name__ == "__main__":
    dstar = DStarLite("snell_library", "curry_student_center")
    dstar_path, dstar_nodes = dstar.planroute()

    print("D* path:", dstar_path)
    print("D* nodes:", dstar_nodes)

   # test A and D match day 8
if path == dstar_path:
    print("PASS: A* and D* Lite paths match")
else:
    print("FAIL: do not match")