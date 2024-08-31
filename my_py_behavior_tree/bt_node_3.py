import rclpy
from rclpy.node import Node
import py_trees
import py_trees.display
import random
import time

class DetectObstacle(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node

    def update(self):
        obstacle_detected = random.choice([True, False])
        if obstacle_detected:
            self.node.get_logger().info("장애물 발견!")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().info("장애물 없음")
            return py_trees.common.Status.FAILURE

class MoveBackward(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node

    def update(self):
        self.node.get_logger().info("후진 중...")
        time.sleep(1)
        return py_trees.common.Status.SUCCESS

class MyBTNode(Node):
    def __init__(self):
        super().__init__('my_bt_node')
        self.get_logger().info("Behavior Tree 초기화")

    def create_tree(self):
        root = py_trees.composites.Sequence(name="루트 시퀀스", memory=True)

        obstacle_sequence = py_trees.composites.Sequence(name="장애물 시퀀스", memory=False)
        detect_obstacle = DetectObstacle(name="장애물 감지", node=self)
        move_backward = MoveBackward(name="후진", node=self)
        obstacle_sequence.add_children([detect_obstacle, move_backward])

        root.add_child(obstacle_sequence)
      
        py_trees.display.render_dot_tree(root, with_blackboard_variables=True)
        return root

    def run(self):
        tree = py_trees.trees.BehaviourTree(self.create_tree())
        while rclpy.ok():
            tree.tick()
            rclpy.spin_once(self, timeout_sec=1)

def main(args=None):
    rclpy.init(args=args)
    node = MyBTNode()
    node.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()