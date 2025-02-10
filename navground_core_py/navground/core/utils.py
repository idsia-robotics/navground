from __future__ import annotations

from collections.abc import Iterator
import contextlib
import os


@contextlib.contextmanager
def chdir(path: os.PathLike[str] | None) -> Iterator[None]:
    """
    Like :py:func:`contextlib.chdir` (Python>=3.11)
    but supports an optional argument.

    :param      path:  Temporary desired working directory
    """
    if path is not None:
        _old = os.getcwd()
        os.chdir(os.path.abspath(path))
    else:
        _old = None
    try:
        yield
    finally:
        if _old is not None:
            os.chdir(_old)
