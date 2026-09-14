"""Compatibility helpers for generated ROS diagnostic messages."""


def set_diagnostic_level(status, level: int) -> None:
    """Assign a DiagnosticStatus uint8 across ROS 2 Python generators."""
    value = int(level)
    # Jazzy's diagnostic_msgs generator exposes ``level`` as a one-byte value,
    # but its Python setter also accepts int.  Assigning that accepted int only
    # fails later in the C converter (PyBytes_Check), so preserve the getter's
    # representation instead of using setter rejection as feature detection.
    try:
        byte_level = isinstance(status.level, bytes)
    except AttributeError:
        byte_level = False
    if byte_level:
        status.level = bytes([value])
        return
    try:
        status.level = value
    except AssertionError:
        status.level = bytes([value])
