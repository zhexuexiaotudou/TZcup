from sanitation_perception.diagnostic_compat import set_diagnostic_level


class _IntegerLevel:
    @property
    def level(self):
        return self._level

    @level.setter
    def level(self, value):
        assert isinstance(value, int)
        self._level = value


class _ByteLevel:
    @property
    def level(self):
        return self._level

    @level.setter
    def level(self, value):
        assert isinstance(value, bytes) and len(value) == 1
        self._level = value


def test_set_diagnostic_level_supports_integer_and_byte_generators():
    integer_status = _IntegerLevel()
    byte_status = _ByteLevel()
    set_diagnostic_level(integer_status, 2)
    set_diagnostic_level(byte_status, 2)
    assert integer_status.level == 2
    assert byte_status.level == b"\x02"
