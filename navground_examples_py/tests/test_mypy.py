import sys

import mypy.api


def test_mypy() -> None:
    stdout, stderr, exitcode = mypy.api.run(
        ['-p', 'navground_examples_py', '--strict'])
    if exitcode:
        print(stderr, file=sys.stderr)
    else:
        print(stdout)
    assert exitcode == 0
