import rclpy
from rclpy.node import Node
import py_trees
import py_trees.display
import random

class MyBTNode(Node):
    def __init__(self):
        super().__init__('my_bt_node')
        self.get_logger().info("Behavior Tree 초기화")

    def create_tree(self):
        root = py_trees.composites.Sequence(name="루트 시퀀스", memory=True)
        detect_obstacle = DetectObstacle(name="장애물 감지")
        root.add_child(detect_obstacle)
      
        py_trees.display.render_dot_tree(root, with_blackboard_variables=True)
        return root

    def run(self):
        tree = py_trees.trees.BehaviourTree(self.create_tree())
        rclpy.spin_once(self, timeout_sec=1)

class DetectObstacle(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)

    def update(self):
        obstacle_detected = random.choice([True, False])
        if obstacle_detected:
            self.logger.info("장애물 발견!")
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.info("장애물 없음")
            return py_trees.common.Status.FAILURE

def main(args=None):
    rclpy.init(args=args)
    node = MyBTNode()
    node.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()