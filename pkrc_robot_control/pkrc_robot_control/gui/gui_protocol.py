"""GUI surface contract.

`GUIProtocol` is a `typing.Protocol` (PEP 544) declaring every method any GUI
implementation must provide. Both `NullGUI` (no-op placeholder) and any future
Qt GUI satisfy this protocol structurally — no inheritance required.

Reason for existence: prior to D5, `NullGUI` only implemented 4 of the 7
methods consumers actually called. `update_sonar_tilt` was called naked
(no try/except guard), so any sonar tilt callback would raise AttributeError
in a NullGUI environment. Formalizing the contract here lets pyright catch
similar gaps at edit time, before they fire at runtime.
"""
from typing import Protocol


class GUIProtocol(Protocol):
    """Contract every GUI implementation satisfies.

    Methods accept arbitrary positional / keyword args because consumers
    pass varying signatures (status / mode / brightness / etc.). The
    runtime contract is "exists and is callable"; type-level contract is
    structural matching of the method names.
    """

    def update_system(self, *args, **kwargs) -> None: ...
    def update_motors(self, *args, **kwargs) -> None: ...
    def update_joystick(self, *args, **kwargs) -> None: ...
    def update_relays(self, *args, **kwargs) -> None: ...
    def update_led(self, *args, **kwargs) -> None: ...
    def update_sonar_tilt(self, *args, **kwargs) -> None: ...
    def update_battery(self, *args, **kwargs) -> None: ...
