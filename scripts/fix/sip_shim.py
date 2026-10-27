"""Shadow Ubuntu's broken sip4 module with PyQt5's maintained sip implementation.

PyCharm's skeleton generator imports the legacy top-level ``sip`` module while
building stubs for the container interpreter. Ubuntu 24.04's ``python3-sip``
4.19.25 registers a ``Py_AtExit`` handler that calls ``PyMem_Free()`` during
interpreter finalization. Python 3.12 finalizes the GIL before those handlers
run, so simply importing ``sip`` makes the process die with SIGSEGV (exit code
139). PyCharm reports "Couldn't refresh skeletons ... exit code 139" and leaves
all imports unresolved.

``python3-sip`` cannot just be uninstalled: ROS rqt/rtabmap packages depend on
it. PyQt5 ships the maintained sip implementation (``PyQt5.sip``, sip 6), so
expose that under the legacy top-level name. Importing ``PyQt5.sip`` registers
itself as ``sys.modules['sip']``, which replaces this shim and shadows the
broken sip4 module. This file is installed into
``/usr/local/lib/python3.12/dist-packages/sip.py``, which comes before
``/usr/lib/python3/dist-packages`` on ``sys.path``.

See https://github.com/python/cpython/issues/105690
"""

from PyQt5.sip import *  # noqa: F401,F403
from PyQt5.sip import (  # noqa: F401
    SIP_VERSION,
    SIP_VERSION_STR,
    simplewrapper,
    wrapper,
)
