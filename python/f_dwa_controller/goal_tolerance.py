"""Use the existing localization reserve inside the requested goal circle."""
import math


def goal_capture_radius(goal_radius, localization_margin):
    """Return the nominal stopping radius whose error reserve fits the goal."""
    radius, margin = float(goal_radius), float(localization_margin)
    if not math.isfinite(radius) or not math.isfinite(margin) or margin < 0.0:
        raise ValueError('Goal radius and localization margin must be finite and nonnegative')
    if radius <= margin:
        raise ValueError('Goal radius must exceed the localization uncertainty margin')
    return radius - margin
