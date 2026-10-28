"""Check controller geometry and phase transitions without a simulator."""
import math
import pytest
from turtle_lab_control.common import normalize
from turtle_lab_control.controller import Phase, ProportionalController

@pytest.mark.parametrize('angle', [-7.0, -math.pi, 0.0, math.pi, 7.0])
def test_normalization(angle: float) -> None:
    """Wrapping preserves direction and stays in the documented range."""
    wrapped = normalize(angle)
    assert -math.pi <= wrapped <= math.pi
    assert math.sin(wrapped) == pytest.approx(math.sin(angle))

@pytest.mark.parametrize('x,y,yaw', [(2.0,0.0,0.0),(-2.0,0.0,math.pi),
                                  (0.0,2.0,math.pi/2),(0.0,-2.0,-math.pi/2)])
def test_drive_in_each_direction(x: float, y: float, yaw: float) -> None:
    """Aligned goals drive forward regardless of world direction."""
    controller = ProportionalController()
    linear, angular, _, _, done = controller.velocity(0,0,yaw,x,y,yaw)
    assert linear>0 and abs(angular)<1e-10 and not done
    assert controller.phase == Phase.DRIVE

def test_rotate_then_final_yaw() -> None:
    """Behind targets rotate in place; reaching position aligns final yaw."""
    controller = ProportionalController()
    linear, angular, _, _, _ = controller.velocity(0,0,0,-2,0,1)
    assert linear==0 and abs(angular)>0
    linear, angular, _, _, done = controller.velocity(-2,0,0,-2,0,1)
    assert linear==0 and angular>0 and not done
    assert controller.velocity(-2,0,1,-2,0,1)[-1]
