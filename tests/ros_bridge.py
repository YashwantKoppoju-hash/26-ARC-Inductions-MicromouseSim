"""Real DDS transport using a simulated scan timestamp; physics is unmodified."""

import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import LoggingSeverity
from rclpy.time import Time

from student_agent.solver import StudentSolver


class RosBridge:
    def __init__(self, engine, mouse, domain_id):
        rclpy.init(domain_id=domain_id)

        class ObservedSimNode(engine.SimNode):
            def __init__(self, physical_mouse):
                self.commands_received = 0
                super().__init__(physical_mouse)

            def cmd_callback(self, msg):
                super().cmd_callback(msg)
                self.commands_received += 1

        self.sim_time = 0.0
        self.solver = StudentSolver()
        self.solver.get_logger().set_level(LoggingSeverity.ERROR)
        self.sim = ObservedSimNode(mouse)
        self.sim.timer.cancel()  # Harness drives the same publisher at 20 Hz.
        self.sim.get_clock = lambda: self
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.sim)
        self.executor.add_node(self.solver)
        deadline = time.monotonic() + 15
        while not (self.sim.scan_pub.get_subscription_count()
                   and self.sim.vel_pub.get_subscription_count()
                   and self.solver.cmd_pub.get_subscription_count()):
            if time.monotonic() > deadline:
                raise RuntimeError('DDS discovery timed out')
            self.executor.spin_once(timeout_sec=0.01)

    def now(self):
        return Time(seconds=self.sim_time)

    def step(self, dt):
        self.sim_time += dt
        previous = self.sim.commands_received
        self.sim.publish_sensor_data()
        deadline = time.monotonic() + 2
        while (self.solver.previous_stamp is None
               or abs(self.solver.previous_stamp - self.sim_time) > 1e-6
               or self.sim.commands_received <= previous):
            if time.monotonic() > deadline:
                raise RuntimeError('DDS scan/velocity/command delivery timed out')
            self.executor.spin_once(timeout_sec=0.001)
        return self.solver.navigator.command

    def close(self):
        self.executor.shutdown()
        self.sim.destroy_node()
        self.solver.destroy_node()
        rclpy.shutdown()
