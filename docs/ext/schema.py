import textwrap

import navground.core
import navground.sim
import yaml
import sys
from docutils import nodes
from sphinx.util.docutils import SphinxDirective


class SchemaDirective(SphinxDirective):
    required_arguments = 1
    optional_arguments = 100

    def run(self) -> list[nodes.Node]:
        try:
            schema = eval(' '.join(self.arguments))
        except Exception as e:
            print(".. schema:: ", ' '.join(self.arguments), str(e), file=sys.stderr)
            schema = {}
        text = yaml.safe_dump(schema)
        text = textwrap.indent(text, 3 * ' ')
        text = f'''
.. code-block:: yaml

{text}
'''
        parsed = self.parse_text_to_nodes(text)
        return parsed


def setup(app):
    app.add_directive('schema', SchemaDirective)
