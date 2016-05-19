import math


class State:
    # This is an object class for state definition.
    def __init__(self, position, GOAL_STATE=None):
        self.position = tuple(position)
        self.GOAL_STATE = GOAL_STATE
        self.h = 0
        self.successor_list = []
        if GOAL_STATE is not None:
            self.heuristics()

    def heuristics(self):
        # h is the direct distance between goal point and vertex.
        self.h += math.sqrt((self.position[0]-self.GOAL_STATE.position[0])**2 +
                            (self.position[1]-self.GOAL_STATE.position[1])**2)

    def is_goal(self):
        return self.position == self.GOAL_STATE.position

    def successor(self, visible_vertices):
        for vertex in visible_vertices:
            self.successor_list.append(State(vertex, self.GOAL_STATE))
        return self.successor_list

    def __lt__(self, other):
        return self.h < other.h

    def __repr__(self):
        return 'State : %s, h : %d\n' % (self.position, self.h)