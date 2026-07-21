"""Angle display units.

The protocol is radians and stays radians - every value on the wire, in
:mod:`robstride.params` and in the manuals is SI. This module is purely a
presentation layer: it converts on the way to a widget and back again on the
way to :meth:`robstride.Motor.write`, so nothing below the GUI ever sees a
degree.

One process-wide :data:`units` object holds the preference and emits
``changed`` so every view can re-render itself at once.
"""

from __future__ import annotations

import math

from PySide6.QtCore import QObject, Signal
from PySide6.QtWidgets import QDoubleSpinBox

RAD_TO_DEG = 180.0 / math.pi

#: Canonical unit -> the degree-based unit shown in its place. Units absent
#: from this map (A, Nm, V, C, Hz) are not angular and are never converted.
ANGULAR = {
    "rad": "deg",
    "rad/s": "deg/s",
    "rad/s^2": "deg/s^2",
}


def is_angular(unit: str) -> bool:
    return unit in ANGULAR


def _nice_step(value: float) -> float:
    """Round a converted step to the nearest 1-2-5 decade value.

    0.1 rad is a sensible step; its literal conversion, 5.73 deg, is not - a
    spin box should tick in round numbers whatever the unit.
    """
    if value <= 0:
        return 1.0
    decade = 10.0 ** math.floor(math.log10(value))
    return min((m * decade for m in (1, 2, 5, 10)),
               key=lambda candidate: abs(candidate - value))


class UnitPreference(QObject):
    """Which angle unit the GUI shows. Degrees by default."""

    #: True when degrees are in use.
    changed = Signal(bool)

    def __init__(self) -> None:
        super().__init__()
        self._degrees = True

    @property
    def degrees(self) -> bool:
        return self._degrees

    def set_degrees(self, on: bool) -> None:
        on = bool(on)
        if on != self._degrees:
            self._degrees = on
            self.changed.emit(on)

    # -- conversion -------------------------------------------------------

    def factor(self, unit: str) -> float:
        """Multiplier taking a canonical (radian) value to the shown one."""
        return RAD_TO_DEG if self._degrees and is_angular(unit) else 1.0

    def label(self, unit: str) -> str:
        return ANGULAR[unit] if self._degrees and is_angular(unit) else unit

    def to_display(self, value: float, unit: str) -> float:
        return value * self.factor(unit)

    def to_canonical(self, value: float, unit: str) -> float:
        return value / self.factor(unit)

    def text(self, value: float, unit: str, rad: int = 4, deg: int = 2,
             sign: bool = False) -> str:
        """Format a canonical value with its displayed unit and precision."""
        decimals = deg if self.factor(unit) != 1.0 else rad
        return (f"{self.to_display(value, unit):{'+' if sign else ''}.{decimals}f}"
                f" {self.label(unit)}")


#: Process-wide preference. Views read and write this single object.
units = UnitPreference()


class AngleSpin(QDoubleSpinBox):
    """A spin box that holds radians and shows whatever the user prefers.

    Range, step and value are all given in canonical units; :meth:`rad` gives
    the value back the same way. Rebuilding on a unit change goes through the
    stored radian value rather than the displayed one, so flipping the toggle
    repeatedly does not accumulate rounding error.
    """

    def __init__(self, minimum: float, maximum: float, value: float = 0.0,
                 step: float = 0.1, unit: str = "rad", parent=None):
        super().__init__(parent)
        self._unit = unit
        self._min = minimum
        self._max = maximum
        self._step = step
        self._rad = value
        self._syncing = False
        self.valueChanged.connect(self._on_value_changed)
        self._apply()
        units.changed.connect(self._on_units_changed)

    # -- canonical access -------------------------------------------------

    def rad(self) -> float:
        """The value in canonical units, whatever is on screen."""
        return self._rad

    def setRad(self, value: float) -> None:
        self._rad = value
        self._apply()

    def setRadRange(self, minimum: float, maximum: float) -> None:
        self._min = minimum
        self._max = maximum
        self._apply()

    # -- internals --------------------------------------------------------

    def _on_value_changed(self, shown: float) -> None:
        if not self._syncing:
            self._rad = units.to_canonical(shown, self._unit)

    def _on_units_changed(self, _degrees: bool) -> None:
        self._apply()

    def _display_step(self, factor: float) -> float:
        if factor == 1.0:
            return self._step
        step = _nice_step(self._step * factor)
        # A converted position step lands on 2 or 5 deg, which is too coarse
        # to nudge a joint into place. Rates keep their rounded step - it is
        # only the absolute angle that gets dialled in a degree at a time.
        return min(step, 1.0) if self._unit == "rad" else step

    def _apply(self) -> None:
        factor = units.factor(self._unit)
        degrees = factor != 1.0
        self._syncing = True
        try:
            self.setDecimals(2 if degrees else 3)
            self.setRange(self._min * factor, self._max * factor)
            self.setSingleStep(self._display_step(factor))
            self.setSuffix(f" {units.label(self._unit)}")
            self.setValue(self._rad * factor)
        finally:
            self._syncing = False
        # A model change can shrink the range under a value already held, so
        # re-clamp. This is deliberately done against the canonical bounds
        # rather than by comparing to what the box now shows: the display is
        # rounded to `decimals`, and treating that rounding as a clamp would
        # feed the truncated number back into _rad on every unit flip.
        self._rad = min(max(self._rad, self._min), self._max)
