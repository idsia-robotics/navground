import os
import pathlib
import shutil
import tempfile
import webbrowser
from collections import ChainMap
from typing import Any, List, Mapping, Optional

import jinja2

from .. import World
from .to_svg import _svg_for_world

folder = os.path.dirname(os.path.realpath(__file__))
template_folder = os.path.join(folder, 'templates')
jinjia_env = jinja2.Environment(
    loader=jinja2.FileSystemLoader(template_folder))


def open_html(width: int = 640,
              port: int = 8000,
              display_shape: bool = False,
              path: pathlib.Path | None = None) -> None:
    if path is None:
        d = pathlib.Path(tempfile.mkdtemp())
        f = d / "world.html"
    else:
        f = path
    with open(f, "w+") as fp:
        data = html_for_world(with_websocket=True,
                              width=width,
                              port=port,
                              display_shape=display_shape)
        fp.write(data)
    webbrowser.open(f"file://{f.resolve()}", new=1)


def save(world: Optional[World] = None,
         world_name: str = '',
         folder: str = '',
         file_name: str = 'index',
         empty: bool = False,
         include_style_path: Optional[str] = None,
         include_script_path: Optional[str] = None,
         copy_included_files: bool = False,
         simulation: bool = False,
         **kwargs: Any) -> None:
    if simulation:
        kw: Mapping = ChainMap({
            'interactive': True,
            'with_websocket': True
        }, kwargs)
    else:
        kw = kwargs
    paths: List[str] = []
    if copy_included_files:
        if include_style_path:
            paths.append(include_style_path)
            include_style_path = os.path.basename(include_style_path)
        if include_script_path:
            paths.append(include_script_path)
            include_style_path = os.path.basename(include_script_path)
    content = html_for_world(world,
                             world_name,
                             include_style_path=include_style_path,
                             include_script_path=include_script_path,
                             notebook=False,
                             **kw)
    path = os.path.join(folder, f'{file_name}.html')
    with open(path, 'w') as f:
        f.write(content)
    for path in paths:
        shutil.copyfile(path, os.path.join(folder, os.path.basename(path)))


notebook_count = 0


def html_for_world(world: Optional[World] = None,
                   world_name: str = '',
                   with_websocket: bool = False,
                   notebook: bool = False,
                   port: int = 8000,
                   display_shape: bool = False,
                   external_style_path: str = '',
                   style: str = '',
                   include_script_path: Optional[str] = None,
                   **kwargs: Any) -> str:
    """
    Draw the world as a SVG embedded in HTML, optionally with a javascript script to keep
    the view in sync with :py:class:`navground.sim.ui.WebUI`

    :param      world:                  The world to display
    :param      world_name:             The world name used as title
    :param      with_websocket:         Whether to add a websocket client to keep the view in sync
    :param      notebook:               Whether the HTML will be embedded in a notebook
    :param      port:                   The websocket port
    :param      display_shape:          Whether to display the agent circular shape
    :param      external_style_path:    The external style path
    :param      style:                  An inline CSS style to include
    :param      include_script_path:    An alternative script path to be included
    :param      kwargs:                 Arguments forwarded to :py:func:`navground.sim.ui.svg_for_world`

    :returns:   An HTML string
    """

    if notebook:
        global notebook_count
        prefix = f'N{notebook_count}'
        notebook_count += 1
    else:
        prefix = ''
    svg, dims = _svg_for_world(world=world,
                               standalone=False,
                               prefix=prefix,
                               external_style_path='',
                               style='',
                               **kwargs)
    return jinjia_env.get_template('world.html').render(
        svg=svg,
        world_name=world_name,
        style=style,
        external_style_path=external_style_path,
        include_script_path=include_script_path,
        with_websocket=with_websocket,
        notebook=notebook,
        prefix=prefix,
        port=port,
        display_shape=display_shape,
        **dims)
