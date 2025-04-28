from collections import namedtuple


Transition = namedtuple('Transition', ['state', 'action_log_prob', 'reward', 'next_state', 'done', 'entropy'])