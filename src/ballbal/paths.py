"""Stable project paths shared by commands and generated artifacts."""

from pathlib import Path

from .config import active_profile_name


def repo_root() -> Path:
    working = Path.cwd()
    if (working / "profiles").is_dir():
        return working
    source_checkout = Path(__file__).resolve().parents[2]
    if (source_checkout / "profiles").is_dir():
        return source_checkout
    return working


def runtime_dir(*parts: str) -> Path:
    return repo_root().joinpath("runtime", *parts)


def active_profile_dir() -> Path:
    return repo_root() / "profiles" / active_profile_name()
