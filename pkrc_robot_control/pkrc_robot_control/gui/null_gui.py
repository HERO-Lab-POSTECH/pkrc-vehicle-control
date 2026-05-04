"""Null Object pattern for the GUI dependency.

기존 코드는 `gui` 파라미터를 받아 `update_system / update_motors / update_joystick /
update_relays` 4개 메서드를 호출. 통합 Qt GUI(Jetson→Laptop)가 도입되기 전까지 no-op
placeholder로 주입한다. 호출 site에서 `if self.gui:` 가드를 제거할 수 있어 코드 단순화.

미래 GUI 구현체는 본 클래스의 인터페이스(같은 4개 메서드 시그니처)를 만족시키면 그대로
drop-in 교체 가능.
"""


class NullGUI:
    """No-op GUI placeholder. 모든 메서드는 임의 인자 수용 후 None 반환."""

    def update_system(self, *_, **__):
        pass

    def update_motors(self, *_, **__):
        pass

    def update_joystick(self, *_, **__):
        pass

    def update_relays(self, *_, **__):
        pass
