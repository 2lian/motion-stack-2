import importlib.util
from pathlib import Path


def patch_rtb_numpy_disp(*, backup: bool = True) -> Path:
    """
    Remove `from numpy import disp` from
    `roboticstoolbox/mobile/DistanceTransformPlanner.py`.

    Must be called before importing `roboticstoolbox`.

    Returns the patched file path.
    """
    spec = importlib.util.find_spec("roboticstoolbox")
    if spec is None or not spec.submodule_search_locations:
        raise ModuleNotFoundError("roboticstoolbox is not installed")

    package_dir = Path(next(iter(spec.submodule_search_locations)))
    target = package_dir / "mobile" / "DistanceTransformPlanner.py"

    if not target.exists():
        raise FileNotFoundError(f"Could not find target file: {target}")

    text = target.read_text(encoding="utf-8")
    bad_line = "from numpy import disp\n"

    if bad_line not in text:
        bad_line = "from numpy import disp\r\n"

    if "from numpy import disp" not in text:
        return target  # already patched

    if backup:
        backup_path = target.with_suffix(target.suffix + ".bak")
        if not backup_path.exists():
            backup_path.write_text(text, encoding="utf-8")

    text = text.replace("from numpy import disp\r\n", "")
    text = text.replace("from numpy import disp\n", "")
    target.write_text(text, encoding="utf-8")

    return target

def patch():
    """Applies the patch to rtb"""
    patch_rtb_numpy_disp()
    import roboticstoolbox.tools.urdf.urdf as bad

    import motion_stack.rtb_fix.fixed_urdf as fix

    bad.URDF.__init__ = fix.URDF.__init__
    bad.URDF._recursive_axis_definition = fix.URDF._recursive_axis_definition
    bad.URDF.finalize_linking = fix.URDF.finalize_linking
