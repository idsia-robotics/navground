import sys

from . import main

if __name__ == '__main__':
    # `None` is accepted by sys.exit and equivalent to 0
    sys.exit(main.main())  # type: ignore[func-returns-value]
