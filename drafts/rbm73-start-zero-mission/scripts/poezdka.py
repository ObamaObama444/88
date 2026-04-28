#!/usr/bin/env python3
"""Compatibility entrypoint for the old movement script name.

The actual mission algorithm lives in drive_sticker_mission.py:
drive through sticker points, count classes, and return to the rarest class.
"""

from drive_sticker_mission import main


if __name__ == "__main__":
    main()
