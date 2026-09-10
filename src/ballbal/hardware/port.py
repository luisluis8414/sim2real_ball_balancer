"""Serial port discovery.

Hard-coding a ``/dev/serial/by-id/...`` path works right up until the board is
plugged into a different machine or a second adapter appears. Discovery keeps
the stable-path benefit without the brittleness.
"""

from __future__ import annotations

from pathlib import Path

BY_ID = Path("/dev/serial/by-id")

# CH340/CH343 (QinHeng) adapters, as used by the common Feetech/Waveshare bus
# servo driver boards, and Feetech's own URT-1.
KNOWN_VENDOR_HINTS = ("1a86", "ftdi", "silabs", "cp210")


def list_ports() -> list[Path]:
    """Return every stable by-id serial path, newest-looking first."""
    if not BY_ID.is_dir():
        return []
    return sorted(p for p in BY_ID.iterdir() if p.is_symlink() or p.is_char_device())


def find_port(preferred: str | None = None) -> str:
    """Resolve the serial port to use.

    Args:
        preferred: An explicit path from config or the command line. Returned
            as-is if it exists, so an operator can always override discovery.

    Raises:
        FileNotFoundError: Nothing plausible is attached, or the choice is
            ambiguous. The message lists what was actually found.
    """
    if preferred:
        if Path(preferred).exists():
            return preferred
        raise FileNotFoundError(
            f"Configured port {preferred!r} does not exist. "
            f"Available: {_describe(list_ports())}"
        )

    candidates = list_ports()
    likely = [
        p for p in candidates if any(h in p.name.lower() for h in KNOWN_VENDOR_HINTS)
    ] or candidates

    if not likely:
        raise FileNotFoundError(
            "No serial adapters found under /dev/serial/by-id. Is the driver "
            "board plugged in, and is your user in the 'dialout' group?"
        )
    if len(likely) > 1:
        raise FileNotFoundError(
            "Several serial adapters are attached; set 'port' in the rig file "
            f"or pass --port. Found: {_describe(likely)}"
        )
    return str(likely[0])


def _describe(paths: list[Path]) -> str:
    if not paths:
        return "none"
    return ", ".join(f"{p} -> {p.resolve().name}" for p in paths)
