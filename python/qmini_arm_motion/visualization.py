"""Compatibility module for the single Qarm Viser application.

The standalone motion visualizer was consolidated into ``qarm_viser``.
Use ``qarm-viser`` or the retained ``qmini-motion viz`` command.
"""

from qarm_viser.app import main

__all__ = ["main"]


if __name__ == "__main__":
    raise SystemExit(main())
