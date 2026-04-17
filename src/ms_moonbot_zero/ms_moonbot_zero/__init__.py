from importlib.resources import as_file, files

from jinja2 import Template


def load_moonbot_zero_urdf() -> str:
    """Compiles the jinja with absolute path (file:// instead of ros package://)."""
    assets = files("ms_moonbot_zero").joinpath("assets")
    template = Template(assets.joinpath("moonbot_zero.jinja.urdf").read_text())
    with as_file(assets) as assets_dir:
        mesh_path = assets_dir / "meshes"
        return template.render(
            root_path=assets_dir.as_uri(),
            mesh_path=mesh_path.as_uri(),
        )
