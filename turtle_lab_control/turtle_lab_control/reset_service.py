"""Reset turtlesim using its native services, with the pen off for teleport."""
import threading
from typing import Any
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger
from turtlesim.srv import SetPen, TeleportAbsolute
from .common import START, bounded, run

class ResetService(Node):
    """Serialize reset sequences without blocking client response callbacks."""
    def __init__(self) -> None:
        super().__init__('reset_service')
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.pen = self.create_client(SetPen, '/turtle1/set_pen', callback_group=self.group)
        self.teleport = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute',
                                           callback_group=self.group)
        self.clear = self.create_client(Empty, '/clear', callback_group=self.group)
        self.create_service(Trigger, 'reset_turtle', self.reset, callback_group=self.group)

    async def reset(self, request: Any, response: Any) -> Any:
        """Pen off, teleport, clear, then restore the specified pen."""
        del request
        if not self.lock.acquire(blocking=False):
            response.message = 'Reset already in progress'
            return response
        pen_disabled = False
        try:
            if not all(client.service_is_ready() for client in
                       (self.pen, self.teleport, self.clear)):
                raise RuntimeError('Turtlesim services are not ready')
            await bounded(self, self.pen.call_async(SetPen.Request(off=1)))
            pen_disabled = True
            await bounded(self, self.teleport.call_async(
                TeleportAbsolute.Request(x=START, y=START, theta=0.0)))
            await bounded(self, self.clear.call_async(Empty.Request()))
            response.success = True
            response.message = 'Turtle reset and canvas cleared'
        except Exception as error:
            response.success = False
            response.message = str(error)
        finally:
            if pen_disabled:
                try:
                    await bounded(self, self.pen.call_async(
                        SetPen.Request(r=179, g=184, b=255, width=3, off=0)))
                except Exception as error:
                    response.success = False
                    response.message = f'Could not restore pen: {error}'
            self.lock.release()
        return response

def main(args: list[str] | None = None) -> None:
    """Executable entry point."""
    run(ResetService, args)
