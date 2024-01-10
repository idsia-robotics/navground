import pathlib

from .to_html import notebook_view, html_for_world
from .to_svg import svg_for_world
from .web_ui import WebUI

view_path = pathlib.Path(__file__).parent / "templates" / "view.html"
