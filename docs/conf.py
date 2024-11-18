# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

# import subprocess
import re

project = 'navground'
copyright = '2023, Jérôme Guzzi, IDSIA'
author = 'Jérôme Guzzi, IDSIA'
release = '0.2.2'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.todo',
    'breathe',
    'sphinx_rtd_theme',
    'nbsphinx',
    'sphinx_tabs.tabs',
    'sphinxarg.ext',
    'sphinx_ros',
    'sphinx_design',
    'sphinx_toolbox.collapse',
    'sphinxcontrib.luadomain',
    'sphinxcontrib.video',
    'sphinx_copybutton',
    'sphinxcontrib.tikz',
    'sphinx.ext.intersphinx',
    'sphinx-prompt'
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
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']
# ['tutorials', 'packages', 'guides', 'installation', 'scenarios',
#  'background', 'first_steps.rst', 'cli.rst', 'introduction.rst']

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
    'T': 'T',
    'Rect': 'Rect',
}

_replace = {
    "_navground.": "",
    "_navground_sim.": "",
    "numpy.ndarray[numpy.float32[2, 1]]": "Vector2",
    "numpy.ndarray[numpy.float64[2, 1]]": "Vector2",
    "bool | int | float | str | Vector2 | list[bool] | list[int] | list[float] | list[str] | list[Vector2]": "PropertyField",
    "Union[bool, int, float, str, Vector2, List[bool], List[int], List[float], List[str], List[Vector2]]": "PropertyField",
    "Callable[[List[float]], None]": "TaskCallback",
    "Callable[[Vector2, float, float], float]": "Projection",
    "Callable[[float], tuple[Vector2, float, float]]": "Curve",
    "~numpy.ndarray[tuple[~typing.Literal[2]], ~numpy.dtype[~numpy.float64]]": "Vector2",
    "numpy.ndarray[tuple[typing.Literal[2]], numpy.dtype[numpy.float64]]": "Vector2",
}

def f(app, what, name, obj, options, lines):
    # print(f"F {what} {name} {obj}")
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
        for k, v in _replace.items():
            if k in lines[i]:
                lines[i] = lines[i].replace(k, v)
        if ':py:class:`Vector2`' in lines[i]:
            lines[i] = lines[i].replace(':py:class:`Vector2`', ':py:class:`Vector2 <navground.core.Vector2>`')

def g(app, what, name, obj, options, signature, return_annotation):
    # print(f"G {name} {obj} {signature} {return_annotation}")
    # print('g', what, name, signature, return_annotation)
    # return (signature, return_annotation)
    if signature:
        signature = re.sub(r"self: (\w+\.?)+,?\s*", "", signature)
        for k, v in _replace.items():
            if k in signature:
                signature = signature.replace(k, v)
        # print(signature)
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
    'navground.core._navground.Behavior': 'navground.core.Behavior',
    'navground.core._navground.Kinematics': 'navground.core.Kinematics',
    'Behavior': 'navground.core.Behavior',
    'Neighbor.id': 'navground.core.Neighbor.id',
    'navground.sim.NativeAgent': 'navground.sim.Agent',
    'navground.sim.NativeWorld': 'navground.sim.World',
    'Controller.go_to_position': 'navground.core.Controller.go_to_position',
    'navground.register': 'navground.core.register',
    'Vector2': 'navground.core.Vector2',
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
    'DynamicTwoWheelsDifferentialDriveKinematics': 'navground.core.kinematics.DynamicTwoWheelsDifferentialDriveKinematics',
    'SensingState': 'navground.core.SensingState',
    'EnvironmentState': 'navground.core.EnvironmentState',
    'stateestimation.update': 'navground.sim.StateEstimation.update',
    'stateestimation.prepare': 'navground.sim.StateEstimation.prepare',
    'Rect': 'navground.sim.ui.Rect',
    'Task': 'navground.sim.Task',
    'StateEstimation': 'navground.sim.StateEstimation',
    'WheeledKinematics': 'navground.core.kinematics.WheeledKinematics',
    'wheeledkinematics.axis': 'navground.core.kinematics.WheeledKinematics.axis',
    'Task.add_callback': 'navground.sim.Task.add_callback',
    'StateEstimation.update': 'navground.sim.StateEstimation.update',
    'sim.Experiment.number_of_runs': 'navground.sim.Experiment.number_of_runs',
    'navground.core.ORCALine': 'navground.core.behaviors.ORCALine'
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

from docutils.nodes import Text, reference
from sphinx.ext.intersphinx import missing_reference
from sphinx.addnodes import pending_xref


_types = ['navground.core.Vector2', 'core.Vector2', 'Vector2', 'Vector2Like', 'PropertyField',
          'T', 'Curve',
          'Projection', 'TaskCallback', 'Decorate', 'Rect']
_attrs = ['numpy.float64', 'numpy.float32']
def resolve_internal_aliases(app, doctree):
    pending_xrefs = doctree.traverse(condition=pending_xref)
    for node in pending_xrefs:
        if node['refdomain'] == "py":
            if node['reftarget'] in _types:
                node["reftype"] = "type"
            elif node['reftarget'] in _attrs:
                node["reftype"] = "attr"
    for node in pending_xrefs:
        alias = node.get('reftarget', None)
        d = node.get('refdomain', '')
        rs = reftarget_aliases.get(d, {})
        if alias is not None and alias in rs:
            node['reftarget'] = rs[alias]


def setup(app):
    app.connect('autodoc-process-docstring', f);
    app.connect('autodoc-process-signature', g);
    app.connect('doctree-read', resolve_internal_aliases)

    # app.connect('missing-reference', ref)
    # app.connect('object-description-transform', l)
    # app.connect('autodoc-before-process-signature', h)
    #
