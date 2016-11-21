
class LinearConversion:
    """
    Represents a bidirectional transformation of the form

    a = (b - b_offset) * a_per_b
    """
    def __init__(self, prop, *, offset, scale):
        self._offset = offset
        self._scale = scale
        self._prop = prop

    def __get__(self, obj, objtype=None):
        if obj is None:
            return self
        return (getattr(obj, self._prop) - self._offset) * self._scale

    def __set__(self, obj, value):
        setattr(obj, self._prop, self._offset + value / self._scale)
