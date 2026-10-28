"""Verify reset ordering and pen restoration after failures."""
import asyncio
import threading
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from std_srvs.srv import Trigger
from turtle_lab_control import reset_service
from turtle_lab_control.common import START

@pytest.mark.parametrize('fail_teleport', [False, True])
def test_pen_protocol(monkeypatch: pytest.MonkeyPatch, fail_teleport: bool) -> None:
    """Teleport is always preceded by pen-off and followed by pen restoration."""
    calls = []
    def client(name: str) -> Mock:
        def send(request: object) -> tuple[str, object]:
            calls.append((name, request))
            return name, request
        return Mock(service_is_ready=lambda: True, call_async=send)
    async def bounded(node: object, operation: tuple[str, object]) -> None:
        if fail_teleport and operation[0] == 'teleport':
            raise TimeoutError('Injected teleport failure')
    monkeypatch.setattr(reset_service, 'bounded', bounded)
    fake = SimpleNamespace(lock=threading.Lock(), pen=client('pen'),
                           teleport=client('teleport'), clear=client('clear'))
    response = asyncio.run(reset_service.ResetService.reset(fake, None, Trigger.Response()))
    assert response.success == (not fail_teleport)
    assert [name for name, _ in calls] == (
        ['pen', 'teleport', 'pen'] if fail_teleport else ['pen', 'teleport', 'clear', 'pen'])
    assert calls[0][1].off == 1
    teleport = calls[1][1]
    assert teleport.x == pytest.approx(START) and teleport.y == pytest.approx(START)
    assert teleport.theta == 0.0
    pen = calls[-1][1]
    assert (pen.r, pen.g, pen.b, pen.width, pen.off) == (179,184,255,3,0)
    assert not fake.lock.locked()
