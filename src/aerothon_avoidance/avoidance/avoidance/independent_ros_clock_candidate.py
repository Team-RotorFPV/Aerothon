"""Keep actual ROS clock delivery running while a planner callback computes."""
import threading
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter


class IndependentRosClock:
    def __init__(self, node):
        self.node=node
        self.driver=None
        self.executor=None
        self.thread=None
        if not node.get_parameter('use_sim_time').value:
            return
        # The original TimeSource subscribes in the node's default mutually
        # exclusive callback group. Merely adding executor threads does not
        # let that subscription run during its navigation timer callback.
        node._time_source.detach_node()
        self.driver=Node(node.get_name()+'_clock_driver',context=node.context,
            use_global_arguments=False,enable_rosout=False,
            parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.driver._time_source.attach_clock(node.get_clock())
        self.executor=SingleThreadedExecutor(context=node.context)
        self.executor.add_node(self.driver)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True,
            name='ros-clock-delivery')
        self.thread.start()

    def close(self):
        if self.executor is not None:
            self.executor.shutdown(timeout_sec=2.)
            self.thread.join(timeout=2.)
            if self.thread.is_alive():
                raise RuntimeError('ROS clock executor did not stop')
            self.driver.destroy_node()
            self.driver=None
