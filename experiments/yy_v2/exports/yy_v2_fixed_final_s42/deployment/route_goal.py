"""Reference local-goal selector for the deployment route manager.

Import this from the model bundle (the policy node already adds the bundle to
``sys.path``). The same ``navrl.goal.select_local_goal`` function is used by
the training environment, so fixed/adaptive lookahead rules stay aligned.
"""
from navrl.goal import select_local_goal

__all__ = ['select_local_goal']
