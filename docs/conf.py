# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

# import subprocess
import re
import sys
import pathlib as pl

sys.path.append(str(pl.Path(__file__).parent / 'ext'))

from schema import SchemaDirective
from ng_command_output import NGCommandDirective

project = 'navground'
copyright = '2023, Jérôme Guzzi, IDSIA'
author = 'Jérôme Guzzi, IDSIA'
release = '0.8.0.dev'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc', 'sphinx.ext.todo', 'breathe', 'sphinx_rtd_theme',
    'nbsphinx', 'sphinx_tabs.tabs', 'sphinxarg.ext', 'sphinx_design',
    'sphinx_toolbox.collapse', 'sphinxcontrib.luadomain',
    'sphinxcontrib.video', 'sphinx_copybutton', 'sphinxcontrib.tikz',
    'sphinx.ext.intersphinx', 'sphinx-prompt', 'sphinxcontrib.programoutput',
    'enum_tools.autoenum'
    # 'sphinx_autodoc_defaultargs'
]

intersphinx_mapping = {
    'python': ('https://docs.python.org/3', None),
    'numpy': ('https://numpy.org/doc/stable', None),
    'matplotlib': ('https://matplotlib.org/stable', None),
    'IPython': ('https://ipython.readthedocs.io/en/stable/', None),
    'h5py': ('https://docs.h5py.org/en/stable/', None),
    # 'tqdm': ('https://tqdm.github.io', None)
}

tikz_resolution = 600

templates_path = ['_templates']
exclude_patterns = [
    '_build', 'Thumbs.db', '.DS_Store', '**/sensor_combination.rst'
]
#  + [
#     'tutorials', 'packages', 'guides', 'installation', 'scenarios',
#     'background', 'first_steps.rst', 'cli.rst', 'introduction.rst'
# ]

language = 'en'

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

# html_theme = 'alabaster'
# html_theme = "sphinx_rtd_theme"
html_theme = 'sphinx_book_theme'
html_static_path = ['_static']

html_theme_options = {
    "repository_url": "https://github.com/idsia-robotics/navground",
    "repository_branch": "main",
    "path_to_docs": "",
    "launch_buttons": {
        "binderhub_url": "https://mybinder.org"
    },
    # HACK(Jerome): There is some bug that prevent the submenus/headers
    # to open up while scrolling Python classes (generate with autodoc)
    # C++ is working fine. So I just keep them open.
    "show_toc_level": 3
}

# -- Options for todo extension ----------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/extensions/todo.html#configuration

todo_include_todos = True

# subprocess.call('make clean', shell=True)
# subprocess.call('cd ../../doxygen ; doxygen', shell=True)

breathe_projects = {"navground": "_build/doxygen/xml"}
breathe_default_project = "navground"
breathe_show_include = False
breathe_show_enumvalue_initializer = True

toc_object_entries = True
toc_object_entries_show_parents = 'hide'

python_use_unqualified_type_names = True
add_module_names = False
autodoc_typehints_format = 'short'
autodoc_member_order = 'groupwise'
autodoc_class_signature = 'separated'
autodoc_inherit_docstrings = True
autoclass_content = 'class'
autodoc_docstring_signature = True
autodoc_type_aliases = {
    'PropertyField': 'PropertyField',
    'TaskCallback': 'TaskCallback',
    'Vector2': 'Vector2',
    'core.Vector2': 'core.Vector2',
    'navground.core.Vector2': 'navground.core.Vector2',
    'Vector2Like': 'Vector2Like',
    'core.Vector2Like': 'core.Vector2Like',
    'navground.core.Vector2Like': 'navground.core.Vector2Like',
    'Cell': 'Cell',
    'CellLike': 'CellLike',
    'Map': 'Map',
    'T': 'T',
    'Rect': 'Rect',
    'Schema': 'Schema'
}

_replace = {
    "Indices":
    "list[int] | slice",
    "np":
    "numpy",
    "dict[str, object]":
    "navground.core.schema.Schema",
    "dict[str, typing.Any]":
    "navground.core.schema.Schema",
    "tuple[Vector2, Vector2]":
    "Bounds",
    "tuple[navground.core.Vector2, navground.core.Vector2]":
    "Bounds",
    "tuple[core.Vector2, core.Vector2]":
    "Bounds",
    "::core::Behavior":
    "",
    "::core::BehaviorModulation":
    "",
    "::core::Kinematics":
    "",
    "::sim::Task":
    "",
    "::sim::StateEstimation":
    "",
    "::sim::Scenario":
    "",
    "_navground.":
    "",
    "_navground_sim.":
    "",
    'NativeAgent':
    'Agent',
    'NativeWorld':
    'World',
    'Annotated[numpy.typing.NDArray[numpy.uint8], "[m, n]", "flags.writeable", "flags.c_contiguous"]':
    "Map",
    'typing.Annotated[numpy.typing.NDArray[numpy.uint8], "[m, n]", "flags.writeable", "flags.c_contiguous"]':
    "Map",
    'typing.Annotated[numpy.typing.NDArray[numpy.int32], "[2, 1]"]':
    "Cell",
    'typing.Annotated[numpy.typing.ArrayLike, numpy.int32, "[2, 1]"]':
    "CellLike",
    'Annotated[numpy.typing.NDArray[numpy.int32], "[2, 1]"]':
    "Cell",
    'Annotated[numpy.typing.ArrayLike, numpy.int32, "[2, 1]"]':
    "CellLike",
    "numpy.ndarray[numpy.uint8[m, n], flags.writeable, flags.c_contiguous]":
    "Map",
    'typing.Annotated[numpy.typing.NDArray[numpy.float32], "[2, 1]"]':
    "Vector2",
    'typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[2, 1]"]':
    "Vector2Like",
    'typing.Annotated[numpy.typing.NDArray[numpy.float64], "[2, 1]"]':
    "Vector2",
    'typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[2, 1]"]':
    "Vector2Like",
    'Annotated[numpy.typing.NDArray[numpy.float32], "[2, 1]"]':
    "Vector2",
    'Annotated[numpy.typing.ArrayLike, numpy.float32, "[2, 1]"]':
    "Vector2Like",
    'Annotated[numpy.typing.NDArray[numpy.float64], "[2, 1]"]':
    "Vector2",
    'Annotated[numpy.typing.ArrayLike, numpy.float64, "[2, 1]"]':
    "Vector2Like",
    "bool | int | float | str | Vector2 | list[bool] | list[int] | list[float] | list[str] | list[Vector2]":
    "PropertyField",
    "Union[bool, int, float, str, Vector2, List[bool], List[int], List[float], List[str], List[Vector2]]":
    "PropertyField",
    "Callable[[List[float]], None]":
    "TaskCallback",
    'collections.abc.Callable[[typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[2, 1]"], typing.SupportsFloat, typing.SupportsFloat], float]':
    "Projection",
    'collections.abc.Callable[[Vector2Like, typing.SupportsFloat, typing.SupportsFloat], float]':
    "Projection",
    'collections.abc.Callable[[typing.SupportsFloat], tuple[typing.Annotated[numpy.typing.NDArray[numpy.float32], "[2, 1]"], float, float]]':
    "Curve",
    'collections.abc.Callable[[typing.SupportsFloat], tuple[Vector2, float, float]]':
    "Curve",
    "navground.sim.MarkerStateEstimation.ReferenceOrientation":
    "ReferenceOrientation",
    'navground.sim.LocalGridMapStateEstimation.FootprintType':
    'FootprintType',
    'collections.abc.Callable[[collections.abc.Sequence[typing.SupportsFloat]], None]':
    "TaskCallback",
    'bool | typing.SupportsInt | typing.SupportsFloat | str | Vector2Like | collections.abc.Sequence[bool] | collections.abc.Sequence[typing.SupportsInt] | collections.abc.Sequence[typing.SupportsFloat] | collections.abc.Sequence[str] | collections.abc.Sequence[Vector2Like]':
    "PropertyFieldLike",
    "TypeAliasForwardRef('core.Vector2')":
    'navground.core.Vector2',
}


def f(app, what, name, obj, options, lines):
    # print(f"F {what} {name} {obj} {lines}")
    if what == "property":
        if not hasattr(obj.fget, '__annotations__'):
            docs = obj.fget.__doc__
            if docs:
                docs = docs.splitlines()[0]
                rtype = docs.split(' -> ')[-1]
                lines.insert(0, '')
                lines.insert(0, f':type: {rtype}')
        # elif 'return' in obj.fget.__annotations__:
        #     t = str(obj.fget.__annotations__['return'])
        #     found = False
        #     for k, v in _replace.items():
        #         if k in t:
        #             t = t.replace(k, v)
        #             found = True
        #     if found:
        #         obj.fget.__annotations__['return'] = t
    for i, _ in enumerate(lines):
        if 'self' in lines[i]:
            lines[i] = re.sub(r"self: (\w+\.?)+,?\s*", "", lines[i])
        # print('L', lines)
        for k, v in _replace.items():
            if k in lines[i]:
                lines[i] = lines[i].replace(k, v)
        if ':py:class:`Vector2`' in lines[i]:
            lines[i] = lines[i].replace(
                ':py:class:`Vector2`',
                ':py:class:`Vector2 <navground.core.Vector2>`')


def g(app, what, name, obj, options, signature, return_annotation):
    # print(f"G {name} {obj} {signature} {return_annotation}")
    # print('g', what, name, signature, return_annotation)
    # return (signature, return_annotation)
    if signature:
        signature = re.sub(r"self: (\w+\.?)+,?\s*", "", signature)
        for k, v in _replace.items():
            if k in signature:
                signature = signature.replace(k, v)
    if return_annotation:
        for k, v in _replace.items():
            if k in return_annotation:
                return_annotation = return_annotation.replace(k, v)
        # print(return_annotation)
    # if (what == 'property' and return_annotation is None):
    # print('set sig of ', name)
    # return_annotation = '-> int'
    # signature = '()'
    return (signature, return_annotation)


def h(app, obj, bound_method):
    print('h', obj)


def l(app, domain, objtype, contentnode):
    if objtype == 'property':
        print(app, domain, objtype, contentnode)


reftarget_aliases = {}
reftarget_aliases['py'] = {
    "SVG": "IPython.display.SVG",
    "HTML": "IPython.display.HTML",
    "Axes": "matplotlib.axes.Axes",
    "Affine2D": "matplotlib.transforms.Affine2D",
    "Sensor": "navground.sim.Sensor",
    "Sensor.name": "navground.sim.Sensor.name",
    "Sensor.update": "navground.sim.Sensor.update",
    'navground.sim.LocalGridMapStateEstimation.FootprintType':
    'navground.sim.state_estimations.LocalGridMapStateEstimation.FootprintType',
    'navground.sim.LocalGridMapStateEstimation':
    'navground.sim.state_estimations.LocalGridMapStateEstimation',
    'navground.sim.OdometryStateEstimation':
    'navground.sim.state_estimations.OdometryStateEstimation',
    'navground.sim.LidarStateEstimation':
    'navground.sim.state_estimations.LidarStateEstimation',
    'navground.sim.LidarStateEstimation.Scan':
    'navground.sim.state_estimations.LidarStateEstimation.Scan',
    'navground.core._navground.Behavior': 'navground.core.Behavior',
    'navground.core._navground.Kinematics': 'navground.core.Kinematics',
    'Behavior': 'navground.core.Behavior',
    'Neighbor.id': 'navground.core.Neighbor.id',
    'navground.sim._navground_sim.NativeAgent': 'navground.sim.Agent',
    'navground.sim._navground_sim.NativeWorld': 'navground.sim.World',
    'Controller.go_to_position': 'navground.core.Controller.go_to_position',
    'Controller.go_to_pose': 'navground.core.Controller.go_to_pose',
    'navground.register': 'navground.core.register',
    'Vector2': 'navground.core.Vector2',
    'typing.Map': 'Map',
    'navground::sim::Scenario': 'navground.sim.Scenario',
    'navground.Kinematics': 'navground.core.Kinematics',
    'Kinematics': 'navground.core.Kinematics',
    'navground.Behavior': 'navground.core.Behavior',
    'PropertyField': 'navground.core.PropertyField',
    'TaskCallback': 'navground.sim.TaskCallback',
    'navground::core::EnvironmentState': 'navground.core.EnvironmentState',
    'Frame.absolute': 'navground.core.Frame.absolute',
    'Frame.relative': 'navground.core.Frame.relative',
    'behavior.actuated_twist': 'navground.core.Behavior.actuated_twist',
    'behavior': 'navground.core.Behavior',
    'GeometricState': 'navground.core.GeometricState',
    'ndarray[tuple[Literal[2]], dtype[float64]]': 'navground.core.Vector2',
    'HLBehavior': 'navground.core.behaviors.HLBehavior',
    'DynamicTwoWheelsDifferentialDriveKinematics':
    'navground.core.kinematics.DynamicTwoWheelsDifferentialDriveKinematics',
    'SensingState': 'navground.core.SensingState',
    'EnvironmentState': 'navground.core.EnvironmentState',
    'stateestimation.update': 'navground.sim.StateEstimation.update',
    'stateestimation.prepare': 'navground.sim.StateEstimation.prepare',
    'Annotated[numpy.typing.ArrayLike, np.floating[Any], "[2, 2]"]': 'Rect',
    'Rect': 'navground.sim.ui.Rect',
    'Task': 'navground.sim.Task',
    'StateEstimation': 'navground.sim.StateEstimation',
    'WheeledKinematics': 'navground.core.kinematics.WheeledKinematics',
    'twowheelsdifferentialdrivekinematics.wheel_axis':
    'navground.core.kinematics.TwoWheelsDifferentialDriveKinematics.wheel_axis',
    'Task.add_callback': 'navground.sim.Task.add_callback',
    'StateEstimation.update': 'navground.sim.StateEstimation.update',
    'sim.Experiment.number_of_runs': 'navground.sim.Experiment.number_of_runs',
    'navground.core.ORCALine': 'navground.core.behaviors.ORCALine',
    'SchemaModifier': 'navground.core.schema.SchemaModifier',
    'Schema': 'navground.core.schema.Schema',
    # 'navground.sim.MarkerStateEstimation.ReferenceOrientation': 'ReferenceOrientation',
    # 'navground.sim.LidarStateEstimation.Scan': 'Scan',
    # 'navground.sim.state_estimations.LocalGridMapStateEstimation.FootprintType': 'FootprintType'
}

reftarget_aliases['cpp'] = {
    'Node': 'YAML::Node',
    'Behavior': 'navground::core::Behavior',
    'Neighbor': 'navground::core::Neighbor',
    'Kinematics': 'navground::core::Kinematics',
    'Controller': 'navground::core::Controller',
    'Pose2': 'navground::core::Pose2',
    'Twist2': 'navground::core::Twist2',
    'Vector2': 'navground::core::Vector2',
    'Disc': 'navground::core::Disc',
    'LineSegment': 'navground::core::LineSegment',
    'EnvironmentState': 'navground::core::EnvironmentState',
    'BehaviorModulation': 'navground::core::BehaviorModulation',
    'Properties': 'navground::core::Properties',
}

_replace_ref = {
    "TypeAliasForwardRef('core.Vector2')": 'navground.core.Vector2',
}

from docutils.nodes import Text, reference
from sphinx.ext.intersphinx import missing_reference
from sphinx.addnodes import pending_xref

core_cpp_classes = [
    'Behavior',
    'Kinematics',
    'Neighbor',
    'Vector2',
    'Twist2',
    'Pose2',
    'Controller',
    'Disc',
    'LineSegment',
    'EnvironmentState',
    'BehaviorModulation',
    'Properties',
]

_types = [
    'navground.core.Vector2',
    'core.Vector2',
    'Vector2',
    'Vector2Like',
    'PropertyField',
    'T',
    'Curve',
    'Map',
    'Cell',
    'CellLike',
    'Schema',
    'SchemaModifier',
    'PropertyFieldLike',
    'Projection',
    'TaskCallback',
    'Decorate',
    'Rect',
    'navground.core.schema.Schema',
    'ScenarioInitCallback',
]
_attrs = [
    'numpy.uint8', 'numpy.float64', 'numpy.float32', 'numpy.int64',
    'numpy.int32'
]

_data = ['numpy.typing.NDArray', 'numpy.typing.ArrayLike']

_meths = []


def resolve_internal_aliases(app, doctree):
    pending_xrefs = doctree.traverse(condition=pending_xref)
    for node in pending_xrefs:
        # print(node)
        if node['refdomain'] == "py":
            if node['reftarget'] in _types:
                node["reftype"] = "type"
            elif node['reftarget'] in _attrs:
                node["reftype"] = "attr"
            elif node['reftarget'] in _data:
                node["reftype"] = "data"
            elif node['reftarget'] in _meths:
                node["reftype"] = "meth"
    for node in pending_xrefs:
        alias = node.get('reftarget', None)
        d = node.get('refdomain', '')
        if d == 'cpp':
            if node['reftype'] == 'identifier':
                if 'navground::' in node['reftarget']:
                    node['reftarget'] = node['reftarget'][11:]
                if node['reftarget'] in core_cpp_classes:
                    node['reftarget'] = "core::" + node['reftarget']
            else:
                if node['reftarget'] in core_cpp_classes:
                    node['reftarget'] = "navground::core::" + node['reftarget']
            continue
            # elif node['reftarget'] != 'navground':
            #     node['reftarget'] = "core::" + node['reftarget']

        rs = reftarget_aliases.get(d, {})
        if alias is not None and alias in rs:
            node['reftarget'] = rs[alias]
        #     print('S', node['reftarget'])
        # else:
        #     print('M', alias)


def missing_reference(app, env, node, contnode):
    ...


def setup(app):
    app.connect('autodoc-process-docstring', f)
    app.connect('autodoc-process-signature', g)
    # app.connect('missing-reference', missing_reference)
    app.connect('doctree-read', resolve_internal_aliases)
    app.add_directive('schema', SchemaDirective)
    app.add_directive('ng-command-output', NGCommandDirective)

    # app.connect('missing-reference', ref)
    # app.connect('object-description-transform', l)
    # app.connect('autodoc-before-process-signature', h)
    #
