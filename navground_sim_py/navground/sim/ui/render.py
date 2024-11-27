from typing import Any, cast

import cairosvg  # type: ignore[import-untyped]
import numpy as np
import numpy.typing

from .. import World
from .to_svg import svg_for_world

Image = np.typing.NDArray[np.uint8]


def _surface_to_npim(surface: cairosvg.surface.PNGSurface) -> Image:
    """ Transforms a Cairo surface into a numpy array. """
    im = +np.frombuffer(surface.get_data(), np.uint8)
    H, W = surface.get_height(), surface.get_width()
    im.shape = (H, W, 4)  # for RGBA
    return im[:, :, 2::-1]


def _svg_to_npim(svg_bytestring: str,
                 dpi: int = 96,
                 background_color: str = "snow") -> Image:
    """ Renders a svg bytestring as a RGB image in a numpy array """
    tree = cairosvg.parser.Tree(bytestring=svg_bytestring)
    surf = cairosvg.surface.PNGSurface(tree,
                                       None,
                                       dpi,
                                       background_color=background_color).cairo
    return _surface_to_npim(surf)


def image_for_world(world: World,
                    background_color: str = "snow",
                    **kwargs: Any) -> Image:
    """
    Renders the world as a raw image

    :param      world:             The world to be rendered
    :param      background_color:  A valid SVG color for the background
    :param      kwargs:            Arguments forwarded to :py:func:`navground.sim.ui.svg_for_world`

    :returns:   An RGB image as a numpy array
    """
    svg_data = svg_for_world(world, **kwargs)
    return _svg_to_npim(svg_data, background_color=background_color)


def png_for_world(
    world: World,
    background_color: str = "snow",
    dpi: int = 96,
    **kwargs: Any,
) -> bytes:
    """
    Renders the world as an png image

    :param      world:             The world to be rendered
    :param      background_color:  A valid SVG color for the background
    :param      kwargs:            Arguments forwarded to :py:func:`navground.sim.ui.svg_for_world`

    :returns:   A png bytestring
    """
    svg_data = svg_for_world(world, **kwargs)
    return cast(
        bytes,
        cairosvg.svg2png(bytestring=svg_data,
                         background_color=background_color,
                         dpi=dpi))


def pdf_for_world(world: World,
                  background_color: str = "snow",
                  **kwargs: Any) -> bytes:
    """
    Renders the world as an pdf image

    :param      world:             The world to be rendered
    :param      background_color:  A valid SVG color for the background
    :param      kwargs:            Arguments forwarded to :py:func:`navground.sim.ui.svg_for_world`

    :returns:   A pdf bytestring
    """
    svg_data = svg_for_world(world, **kwargs)
    return cast(
        bytes,
        cairosvg.svg2pdf(bytestring=svg_data,
                         background_color=background_color))
