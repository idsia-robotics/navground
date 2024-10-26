import argparse
from typing import Dict, List, Tuple

from navground import core


def description() -> str:
    return "Load and list plugins."


def init_parser(parser: argparse.ArgumentParser) -> None:
    parser.description = description()


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser()
    init_parser(p)
    return p


def title(name: str) -> str:
    return name.replace('_', ' ').title()


def list_plugins(arg: argparse.Namespace,
                 plugins: Dict[str, Dict[str, List[Tuple[str, str]]]]) -> None:
    for pkg, vs in plugins.items():
        s = ""
        for kind, ps in vs.items():
            items = [f"{name} [{lang}]" for name, lang in ps]
            if items:
                s += f"{title(kind)}: {', '.join(items)}\n"
        if s:
            print(pkg)
            print("-" * len(pkg))
            print(s)
            print("")


def _main(arg: argparse.Namespace) -> None:
    list_plugins(arg, core.get_loaded_plugins())


def main() -> None:
    _main(parser().parse_args())
