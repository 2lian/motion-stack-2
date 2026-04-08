from collections.abc import Iterable
from typing import Any

from colorama import Fore


def list_cyanize(l: Iterable[Any]) -> str:
    """Makes each element of a list cyan.

    Args:
        l: Iterable

    Returns:

    """
    out = "["
    first = True
    for k in l:
        if not first:
            out += ", "
        first = False
        if isinstance(k, str):
            out += f"'{Fore.CYAN}{k}{Fore.RESET}'"
        else:
            out += f"{Fore.CYAN}{k}{Fore.RESET}"
    out += "]"
    return out
