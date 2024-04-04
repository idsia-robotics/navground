import pathlib

from .to_html import html_for_world, open_html
from .to_svg import Decorate, Rect, rect_around, svg_color, svg_for_world
from .web_ui import WebUI

view_path = pathlib.Path(__file__).parent / "templates" / "view.html"
