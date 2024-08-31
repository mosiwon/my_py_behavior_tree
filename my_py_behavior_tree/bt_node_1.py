import rclpy
from rclpy.node import Node
import py_trees
import py_trees.display

class MyBTNode(Node):
    def __init__(self):
        super().__init__('my_bt_node')
        self.get_logger().info("Behavior Tree 초기화")

    def create_tree(self):
        root = py_trees.composites.Sequence(name="루트 시퀀스", memory=True)
        py_trees.display.render_dot_tree(root, with_blackboard_variables=True)
        return root

    def run(self):
        tree = py_trees.trees.BehaviourTree(self.create_tree())
        rclpy.spin_once(self, timeout_sec=1)

def main(args=None):
    rclpy.init(args=args)
    node = MyBTNode()
    node.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()