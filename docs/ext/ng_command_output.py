# Adapted from https://github.com/OpenNTI/sphinxcontrib-programoutput/blob/master/src/sphinxcontrib/programoutput/__init__.py

import os
import pathlib as pl
import subprocess
import sys
import textwrap

import navground.core
import navground.sim
from docutils import nodes
from docutils.parsers.rst.directives import unchanged
from sphinx.util.docutils import SphinxDirective


def _slice(value):
    parts = [int(v.strip()) for v in value.split(',')]
    if len(parts) > 2:
        raise ValueError('too many slice parts')
    return tuple((parts + [None] * 2)[:2])


class NGCommandDirective(SphinxDirective):
    # required_arguments = 1
    # optional_arguments = 100
    option_spec = dict(ellipsis=_slice, package=unchanged)
    has_content = False
    final_argument_whitespace = True
    required_arguments = 1
    optional_arguments = 100

    def run(self) -> list[nodes.Node]:
        wd = pl.Path(os.environ["NAVGROUND_PLUGINS_PREFIX"].split(":")[0])
        wd = wd / "lib"
        if 'package' in self.options:
            wd = wd / pl.Path(self.options['package'])
        cmd = ' '.join(self.arguments)
        path = str(wd / self.arguments.pop(0))
        start = [i for i, a in enumerate(self.arguments) if a.startswith('"')]
        end = [i for i, a in enumerate(self.arguments) if a.endswith('"')]
        if start and end:
            i = start[0]
            j = end[0]
            self.arguments[i] = self.arguments[i][1:]
            self.arguments[j] = self.arguments[j][:-1]
            arg = ' '.join(self.arguments[i:j + 1])
            self.arguments = self.arguments[:i] + [arg
                                                   ] + self.arguments[j + 1:]
        r = subprocess.run([path] + self.arguments, capture_output=True)
        if (r.returncode):
            output = r.stderr.decode('ascii')
            print(f"Error in cmd {cmd}, {path}, {self.arguments}", file=sys.stderr)
        else:
            output = r.stdout.decode('ascii')
        if 'ellipsis' in self.options:
            start, stop = self.options['ellipsis']
            lines = output.splitlines()
            if len(lines[start:stop]):
                lines[start:stop] = ['...']
                output = '\n'.join(lines)

        output = textwrap.indent(output, 3 * ' ')
        text = f'''
.. code-block:: console

   $ {cmd}

{output}

'''
        parsed = self.parse_text_to_nodes(text)
        return parsed


def setup(app):
    app.add_directive('schema', NGCommandDirective)
