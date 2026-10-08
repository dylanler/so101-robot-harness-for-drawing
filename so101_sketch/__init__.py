"""SO-101 pen plotter: kinematics, paper calibration, designs and a high-level drawing session.

See README.md for the full story and docs/TOOL_API.md for the agent-facing API.
"""

from .designs import DESIGNS, get_design, trace_image
from .session import SketchSession, capture_setup, dry_run, list_setups, preview, set_surface
from .setup_profile import PaperSurface, SetupProfile

__all__ = [
    "SketchSession", "SetupProfile", "PaperSurface", "DESIGNS", "get_design", "trace_image",
    "preview", "dry_run", "capture_setup", "set_surface", "list_setups",
]
