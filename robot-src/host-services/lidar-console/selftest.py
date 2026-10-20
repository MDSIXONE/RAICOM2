#!/usr/bin/env python3
"""Harmless backup-supervisor self test: no serial, camera or motion."""
import time

for index in range(5):
    print(f"SELFTEST {index + 1}/5: no motion", flush=True)
    time.sleep(1)
print("SELFTEST_DONE", flush=True)
