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

class Battery:
    def __init__(self, level=80):
        self.level = level

    def drain(self, amount):
        self.level = max(0, self.level - amount)
        return self.level

    def charge(self):
        while self.level < 100:
            time.sleep(0.5)
            self.level += 5
            print(f"[배터리 충전] 충전 중... 현재 배터리 잔량: {self.level}%")

class CheckBattery(py_trees.behaviour.Behaviour):
    def __init__(self, name, battery):
        super().__init__(name)
        self.battery = battery

    def update(self):
        self.battery.drain(10) 
        print(f"[배터리 확인] 현재 배터리 잔량: {self.battery.level}%")
        if self.battery.level < 20:
            print("[배터리 확인] 배터리가 부족합니다, 충전이 필요합니다!")
            return py_trees.common.Status.FAILURE
        else:
            print("[배터리 확인] 배터리 잔량이 충분합니다.")
            return py_trees.common.Status.SUCCESS

class ChargeBattery(py_trees.behaviour.Behaviour):
    def __init__(self, name, battery):
        super().__init__(name)
        self.battery = battery

    def update(self):
        print("[배터리 충전] 충전 시작...")
        self.battery.charge()
        print("[배터리 충전] 배터리가 완전히 충전되었습니다!")
        return py_trees.common.Status.SUCCESS

class MyBTNode(Node):
    def __init__(self):
        super().__init__('my_bt_node')
        self.get_logger().info("Behavior Tree 초기화")
        self.battery = Battery()

    def create_tree(self):
        root = py_trees.composites.Sequence(name="루트 시퀀스", memory=True)

        obstacle_sequence = py_trees.composites.Sequence(name="장애물 시퀀스", memory=False)
        detect_obstacle = DetectObstacle(name="장애물 감지", node=self)
        move_backward = MoveBackward(name="후진", node=self)
        obstacle_sequence.add_children([detect_obstacle, move_backward])

        battery_check = CheckBattery(name="배터리 확인", battery=self.battery)
        charge_battery = ChargeBattery(name="배터리 충전", battery=self.battery)
        battery_management = py_trees.composites.Selector(name="배터리 체크 및 충전 셀렉터", memory=False)
        battery_management.add_children([battery_check, charge_battery])

        root.add_children([obstacle_sequence, battery_management])
      
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