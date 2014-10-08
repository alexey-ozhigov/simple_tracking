import rospy

class Enum(set):
    def __getattr__(self, name):
        if name in self:
            return name
        raise AttributeError

class State:
    def __init__(self, state_set, init_state):
        assert init_state in state_set
        self.states = Enum(state_set)
        self.set(init_state)

    def set(self, new_state):
        assert new_state in self.states
        self.cur_state = new_state
        rospy.loginfo('State: %s' % new_state)

    def __eq__(self, state):
        assert state in self.states
        return self.cur_state == state
