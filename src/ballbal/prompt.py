"""Operator prompts.

Kept apart from the modules that use them so a confirmation helper does not
drag a whole command's imports along with it.
"""

from __future__ import annotations

RULE = "=" * 68


def confirm(question: str, expected: str = "yes") -> bool:
    """Ask a yes/no question, accepting the word or its first letter.

    Anything else is a no. A question about whether hardware is safe to move
    should not read a stray keypress as consent.
    """
    answer = input(f"\n{question} [{expected}/no] ").strip().lower()
    return answer in (expected, expected[0])
