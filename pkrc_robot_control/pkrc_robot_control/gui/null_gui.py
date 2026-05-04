"""Null Object pattern for the GUI dependency.

Satisfies the `GUIProtocol` (see `gui_protocol.py`) structurally — every
method consumers call (`update_system`, `update_motors`, `update_joystick`,
`update_relays`, `update_led`, `update_sonar_tilt`, `update_battery`) is
present as a no-op. Future Qt GUI integration just needs to provide
non-trivial implementations of the same method names; no inheritance
required.

Prior to D5 only the first 4 were defined, which caused an
AttributeError in `sensors/sonar_tilt.py:71/106` (naked `self.gui.update_sonar_tilt(...)`
call) on any session that exercised the sonar callback path.
"""


class NullGUI:
    """No-op GUI placeholder. All methods accept arbitrary args, return None."""

    def update_system(self, *_, **__):
        pass

    def update_motors(self, *_, **__):
        pass

    def update_joystick(self, *_, **__):
        pass

    def update_relays(self, *_, **__):
        pass

    def update_led(self, *_, **__):
        pass

    def update_sonar_tilt(self, *_, **__):
        pass

    def update_battery(self, *_, **__):
        pass
