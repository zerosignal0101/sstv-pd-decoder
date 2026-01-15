"""
SSTV PD120 Decoder Python Bindings (C++23)
"""
from __future__ import annotations
import collections.abc
import numpy
import numpy.typing
import typing
__all__: list[str] = ['Decoder', 'Pixel', 'SSTVMode']
class Decoder:
    def __init__(self, sample_rate: typing.SupportsFloat = 11025.0) -> None:
        ...
    def process(self, samples: typing.Annotated[numpy.typing.ArrayLike, numpy.float32]) -> None:
        """
        Process audio samples (NumPy array)
        """
    def reset(self) -> None:
        ...
    def set_on_image_complete_callback(self, arg0: collections.abc.Callable[[typing.SupportsInt, typing.SupportsInt], None]) -> None:
        ...
    def set_on_line_decoded_callback(self, arg0: collections.abc.Callable[[typing.SupportsInt, collections.abc.Sequence[Pixel]], None]) -> None:
        ...
    def set_on_mode_detected_callback(self, arg0: collections.abc.Callable[[SSTVMode], None]) -> None:
        ...
class Pixel:
    def __init__(self, arg0: typing.SupportsInt, arg1: typing.SupportsInt, arg2: typing.SupportsInt) -> None:
        ...
    def __repr__(self) -> str:
        ...
    @property
    def b(self) -> int:
        ...
    @b.setter
    def b(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def g(self) -> int:
        ...
    @g.setter
    def g(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def r(self) -> int:
        ...
    @r.setter
    def r(self, arg0: typing.SupportsInt) -> None:
        ...
class SSTVMode:
    @property
    def height(self) -> int:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def vis_code(self) -> int:
        ...
    @property
    def width(self) -> int:
        ...
