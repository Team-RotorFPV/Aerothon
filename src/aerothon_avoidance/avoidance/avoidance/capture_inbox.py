"""Receive onboard captures independently; apply them only at tick boundaries.

The IO thread never mutates navigation state. A decision uses one drained
batch and retains its own source timestamps and existing freshness checks.
No truth, contact, world position or attachment topic is subscribed here.
"""
from collections import deque
import threading
from types import MethodType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class CaptureInbox:
    def __init__(self, node):
        self.node = node
        self.lock = threading.Lock()
        self.pending = {'pose': deque(maxlen=100), 'velocity': deque(maxlen=5), 'scan': deque(maxlen=5)}
        self.received = dict.fromkeys(self.pending, 0)
        self.overflow = dict.fromkeys(self.pending, 0)
        self.callbacks = {kind: getattr(node, '_on_'+kind) for kind in self.pending}
        subscriptions = []
        for subscription in list(node.subscriptions):
            for kind, callback in self.callbacks.items():
                if subscription.callback == callback:
                    subscriptions.append((kind, subscription, subscription.msg_type, subscription.topic_name))
        assert sorted(kind for kind, *_ in subscriptions) == ['pose', 'scan', 'velocity']
        self.driver = Node(node.get_name()+'_capture_driver', namespace=node.get_namespace(),
            context=node.context, use_global_arguments=False, enable_rosout=False)
        for kind, subscription, message_type, topic in subscriptions:
            self.driver.create_subscription(message_type, topic,
                lambda message, channel=kind: self.receive(channel, message), qos_profile_sensor_data)
            node.destroy_subscription(subscription)
        original_tick = node._tick
        def tick(_node):
            with self.lock:
                batch = {kind: list(queue) for kind, queue in self.pending.items()}
                for queue in self.pending.values():
                    queue.clear()
            # Pose history brackets scan captures. The existing buffered
            # pairing handler retains a previous fresh scan if a new one
            # still awaits its later pose sample. All validation is retained.
            for kind in ('pose', 'velocity', 'scan'):
                for message in batch[kind]:
                    self.callbacks[kind](message)
            original_tick()
        wrapped = MethodType(tick, node)
        timers = [timer for timer in node.timers if timer.callback == original_tick]
        assert len(timers) == 1
        timers[0].callback = wrapped
        node._tick = wrapped
        self.executor = SingleThreadedExecutor(context=node.context)
        self.executor.add_node(self.driver)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True,
                                       name='onboard-capture-delivery')
        self.thread.start()

    def receive(self, kind, message):
        with self.lock:
            queue = self.pending[kind]
            if len(queue) == queue.maxlen:
                self.overflow[kind] += 1
            queue.append(message)
            self.received[kind] += 1

    def close(self):
        self.executor.shutdown(timeout_sec=2.)
        self.thread.join(timeout=2.)
        if self.thread.is_alive():
            raise RuntimeError('Onboard capture executor did not stop')
        self.driver.destroy_node()
