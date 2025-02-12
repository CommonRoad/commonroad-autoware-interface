from abc import ABC, abstractmethod
import py_trees
from rclpy.impl.rcutils_logger import RcutilsLogger


class BaseTree(ABC):
    def __init__(self, logger: RcutilsLogger, verbose: bool, config=None):
        # self.config = config or self.default_config()
        self.logger = logger
        self.verbose = verbose
        self.root = None

    # @abstractmethod
    # def default_config(self):
    #     pass

    @abstractmethod
    def create_behavior_tree(self):
        # root = self.create_node(self.config["root"])
        # return root
        pass

    def create_node(self, node_config):
        node_type = node_config["type"]
        node_name = node_config.get("name", node_type)
        
        if node_type == "Sequence":
            node = py_trees.composites.Sequence(node_name)
        elif node_type == "Selector":
            node = py_trees.composites.Selector(node_name)
        else:
            # Dynamically import the sub-tree class based on the node type
            module = __import__(f'behavior_tree.{node_type.lower()}', fromlist=[node_type])
            node_class = getattr(module, node_type)
            node = node_class().root

        for child_config in node_config.get("children", []):
            child_node = self.create_node(child_config)
            node.add_child(child_node)

        return node

    def tick_once(self):
        self.root.tick_once()
