import os
import shutil
from collections import ChainMap
from typing import Any, Dict, List, Mapping, Optional, cast

import jinja2
from IPython.display import HTML

from .. import World
from .to_svg import svg_for_world

folder = os.path.dirname(os.path.realpath(__file__))
template_folder = os.path.join(folder, 'templates')
jinjia_env = jinja2.Environment(
    loader=jinja2.FileSystemLoader(template_folder))


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
                   style: str = '',
                   include_style_path: Optional[str] = None,
                   include_script_path: Optional[str] = None,
                   with_websocket: bool = False,
                   notebook: bool = False,
                   port: int = 8000,
                   width: float = 600,
                   **kwargs: Any) -> str:
    if notebook:
        global notebook_count
        prefix = f'N{notebook_count}'
        notebook_count += 1
    else:
        prefix = ''
    if not world:
        svg_world = ''
        dims = {
            "viewBox": "0 0 1 1",
            "width": f"{width}",
            "height": f"{width}"
        }
    else:
        svg_world, dims = svg_for_world(world,
                                        standalone=False,
                                        prefix=prefix,
                                        width=width,
                                        **kwargs)
        dims = cast(Dict[str, str], dims)
    return jinjia_env.get_template('world.html').render(
        svg_world=svg_world,
        world_name=world_name,
        style=style,
        include_style_path=include_style_path,
        include_script_path=include_script_path,
        with_websocket=with_websocket,
        notebook=notebook,
        prefix=prefix,
        port=port,
        **dims)


def notebook_view(width: int = 600):
    return HTML(
        html_for_world(world=None,
                       with_websocket=True,
                       width=width,
                       notebook=True))
