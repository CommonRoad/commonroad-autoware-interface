import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from rclpy.impl.rcutils_logger import RcutilsLogger
from std_msgs.msg import Bool

class FailSafe(Behaviour):
    """
    FailSafe BehaviorTree Node.

    :var logger: ROS2 node logger
    :var blackboard: Blackboard for behavior tree
    """
    def __init__(self, name, logger: RcutilsLogger):
        super().__init__(name)
        self._logger = logger
        self.init_blackboard(name)
    
    def init_blackboard(self, name):
        """
        Initialize the blackboard clients for the behavior tree.

        :param name: Name of the behavior tree node
        """
        self.blackboard = py_trees.blackboard.Client(name=(name + "Blackboard"))
        self.blackboard.register_key(key="/failsafe/bool", access=py_trees.common.Access.WRITE)
    
    def setup(self):
        pass

    def initialise(self):
        pass

    def update(self):
        """
        FailSafe behavior tree node. Save boolean to blackboard.
        """
        self.blackboard.failsafe.bool = True

        return Status.SUCCESS
        
    def terminate(self, new_status):
        self._logger.debug("[SVEN]Terminating FailSafe to " + str(new_status))
