"""URDF helper for rerun: strip collision geometry before logging."""

from __future__ import annotations

import xml.etree.ElementTree as ET


def strip_collision(urdf: str) -> str:
    """Remove all ``<collision>`` elements from a URDF string."""
    root = ET.fromstring(urdf)
    for link in root.iter("link"):
        for col in list(link.findall("collision")):
            link.remove(col)
    return ET.tostring(root, encoding="unicode", xml_declaration=True)
