from rclpy.executors import *
from rclpy.context import Context
from rclpy.subscription import Subscription


class MyExecutor(Executor):
    def __init__(self, *, context: Context = None) -> None:
        super().__init__(context=context)
        self.queue_else = []
        self.queue_H = []
        self.queue_M = []
        self.queue_L = []
        self.callback_empty = True

    def spin_once(self, timeout_sec: float = None) -> None:

        while True:
            try:
                timeout_sec = None if self.callback_empty else 0
                handler, entity, node = self.wait_for_ready_callbacks(
                    timeout_sec=timeout_sec)
            except ShutdownException:
                pass
            except TimeoutException:
                break
            else:
                self.callback_empty = False
                if isinstance(entity, Subscription):
                    topic_name = entity.topic_name
                    if '_H' in topic_name:
                        self.queue_H.append(handler)
                    elif '_M' in topic_name:
                        self.queue_M.append(handler)
                    elif '_L' in topic_name:
                        self.queue_L.append(handler)
                    else:
                        self.queue_else.append(handler)
                else:
                    self.queue_else.append(handler)
        if len(self.queue_else) != 0:
            handler = self.queue_else[0]
            self.callback_empty = False
        elif len(self.queue_H) != 0:
            handler = self.queue_H[0]
            self.callback_empty = False
        elif len(self.queue_M) != 0:
            handler = self.queue_M[0]
            self.callback_empty = False
        elif len(self.queue_L) != 0:
            handler = self.queue_L[0]
            self.callback_empty = False
        else:
            # you should block next time
            self.callback_empty = True

        if self.callback_empty is False:
            handler()
            if handler.exception() is not None:
                raise handler.exception()
