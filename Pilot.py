from queue import PriorityQueue
from Config import Environment
from State import State
import math
import sys


def main():
    env = Environment(sys.argv[1])
    traversal_path = []
    goal_state = State(env.goal_state)
    initial_state = State(env.initial_state, GOAL_STATE=goal_state)
    traversed = {}

    def hill_climbing(state):
        pq = PriorityQueue()
        successor_list = state.successor(env.get_apprx_visible_vertices(state.position))
        for successor in successor_list:
            if successor.position in traversed:
                successor = traversed[successor.position]
            pq.put(successor)
        next_state = pq.get()
        if next_state.is_goal():
            return next_state
        if next_state.position in traversed:
            next_state.h += math.sqrt((state.position[0] - next_state.position[0])**2 +
                                      (state.position[1] - next_state.position[1]) ** 2)
        traversed[next_state.position] = next_state
        traversal_path.append(next_state)
        return hill_climbing(next_state)
    traversal_path.append(initial_state)
    goal = hill_climbing(initial_state)
    if goal is not None:
        traversal_path.append(goal)
        print("Goal found, total stops are %d" % (len(traversal_path)))

    print(traversal_path)
    env.animate_path(traversal_path, lambda x: x.position)

if __name__ == '__main__':
    main()