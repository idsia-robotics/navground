# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import subprocess
import re

project = 'navground'
copyright = '2023, Jérôme Guzzi, IDSIA'
author = 'Jérôme Guzzi, IDSIA'
release = '0.2.0'

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
    'sphinxcontrib.tikz'
    # 'sphinx_autodoc_defaultargs'
]

tikz_resolution = 600

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

language = 'en'

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

# html_theme = 'alabaster'
html_theme = "sphinx_rtd_theme"
html_theme = 'sphinx_book_theme'
html_static_path = ['_static']

html_theme_options = {
    "repository_url": "https://github.com/idsia-robotics/navground",
    "repository_branch": "main",
    "path_to_docs": "",
    "launch_buttons": {
        "binderhub_url": "https://mybinder.org"
    },
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

add_module_names = False
autodoc_typehints_format = 'short'
autodoc_member_order = 'groupwise'
autodoc_class_signature = 'separated'
autodoc_inherit_docstrings = True
autoclass_content = 'class'
autodoc_docstring_signature = True
autodoc_type_aliases = {
    'PropertyField': 'PropertyField',
    'TaskCallback': 'TaskCallback'
}

_replace = {
    "_navground.": "",
    "_navground_sim.": "",
    "numpy.ndarray[numpy.float32[2, 1]]": "Vector2",
    "numpy.ndarray[numpy.float64[2, 1]]": "Vector2",
    "Union[bool, int, float, str, Vector2, List[bool], List[int], List[float], List[str], List[Vector2]]": "PropertyField",
    "Callable[[List[float]], None]": "TaskCallback",
}

def f(app, what, name, obj, options, lines):
    if what == "property":
        if not hasattr(obj.fget, '__annotations__'):
            docs = obj.fget.__doc__
            if docs:
                docs = docs.splitlines()[0]
                rtype = docs.split(' -> ')[-1]
                lines.insert(0, '')
                lines.insert(0, f':type: {rtype}')
    for i, _ in enumerate(lines):
        if 'self' in lines[i]:
            lines[i] = re.sub(r"self: (\w+\.?)+,?\s*", "", lines[i])
        for k, v in _replace.items():
            if k in lines[i]:
                lines[i] = lines[i].replace(k, v)
        if ':py:class:`Vector2`' in lines[i]:
            lines[i] = lines[i].replace(':py:class:`Vector2`', ':py:class:`Vector2 <navground.Vector2>`')

def g(app, what, name, obj, options, signature, return_annotation):
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
    'navground.core.Property': 'navground.core._navground.Property',
    'Controller.go_to_position': 'navground.core.Controller.go_to_position',
    'navground.register': 'navground.core.register',
    'Vector2': 'navground.Vector2',
    'navground::sim::Scenario': 'navground.sim.Scenario',
    'navground.core.SocialMarginModulation': 'navground.core.SocialMargin.Modulation',
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
    # 'Disc': 'navground.core.Disc',
    # 'LineSegment': 'navground.core.LineSegment',
    # 'Neighbor': 'navground.core.Neighbor',
    #
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
}

from docutils.nodes import Text, reference
from sphinx.ext.intersphinx import missing_reference
from sphinx.addnodes import pending_xref

def resolve_internal_aliases(app, doctree):
    pending_xrefs = doctree.traverse(condition=pending_xref)
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
