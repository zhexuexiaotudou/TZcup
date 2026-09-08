"""Compatibility helpers for generated ROS diagnostic messages."""


def set_diagnostic_level(status, level: int) -> None:
    """Assign a DiagnosticStatus uint8 across ROS 2 Python generators."""
    value = int(level)
    try:
        status.level = value
    except AssertionError:
        status.level = bytes([value])
