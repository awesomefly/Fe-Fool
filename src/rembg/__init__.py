import sys
import warnings

_version_warning_shown = False
if not (sys.version_info.major == 3 and sys.version_info.minor == 9):
    if not _version_warning_shown:
        _version_warning_shown = True
        warnings.warn("This library is only for Python 3.9", RuntimeWarning)

from . import _version

__version__ = _version.get_versions()["version"]

from .bg import remove
