import pathlib

from .common import (Decorate, Rect, RenderConfig, rect_around,
                     render_default_config)
from .to_html import html_for_world, open_html
from .to_svg import svg_color, svg_for_world
from .web_ui import WebUI

view_path = pathlib.Path(__file__).parent / "templates" / "view.html"

__all__ = [
    "html_for_world", "open_html", "Decorate", "Rect", "rect_around",
    "svg_color", "svg_for_world", "WebUI", "view_path",
    "render_default_config", "RenderConfig"
]
