#!/usr/bin/env python3
"""Best-effort stop after a DOGRk backup process has released the serial port."""
import time

from xgolib import XGO

try:
    dog = XGO("/dev/ttyAMA0")
    dog.stop()
    time.sleep(0.2)
    dog.stop()
    print("SAFE_STOP_OK", flush=True)
except Exception as exc:
    print("SAFE_STOP_FAIL", repr(exc), flush=True)
    raise
