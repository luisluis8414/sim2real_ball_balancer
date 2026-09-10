"""Vendored third-party code.

``scservo_sdk`` is the official Feetech Python SDK (MIT, see LICENSE.ftservo),
copied from https://github.com/ftservo/FTServo_Python at the commit recorded in
SDK_COMMIT.txt.

It is vendored rather than installed because:

* the official repository ships no ``setup.py``/``pyproject.toml``, so it cannot
  be pip-installed from git;
* the package published on PyPI as ``feetech-servo-sdk`` uses the *same* import
  name ``scservo_sdk`` but a different, incompatible API (it passes the port
  handler into every packet call, the official one binds it at construction) and
  it carries a packet-timeout bug that requires a downstream workaround.

Vendoring under ``ballbal.hardware.vendor`` pins the version and removes any chance of
the two shadowing each other if another robotics package installs the PyPI SDK
alongside this project.
"""
