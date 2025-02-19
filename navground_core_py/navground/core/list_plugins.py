from __future__ import annotations

import argparse

from navground import core


def description() -> str:
    return "Load and list plugins."


def init_parser_with_version(parser: argparse.ArgumentParser,
                             version: str) -> None:
    parser.description = description()
    parser.add_argument('-v', '--version', action='version', version=version)
    parser.add_argument('--dependencies',
                        help="Display dependencies of C++ plugins",
                        action='store_true')


def init_parser(parser: argparse.ArgumentParser) -> None:
    init_parser_with_version(parser, core.get_build_info().version_string)


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser()
    init_parser(p)
    return p


def title(name: str) -> str:
    return name.replace('_', ' ').title()


def list_plugins(arg: argparse.Namespace,
                 plugins: dict[str, dict[str, list[tuple[str, str]]]],
                 pkg_dependencies: core.PkgDependencies) -> None:
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
            if arg.dependencies and pkg in pkg_dependencies:
                print("Dependencies:")
                for path, ds in pkg_dependencies[pkg].items():
                    print(f"- {path.name}: ")
                    for name, dep in ds.items():
                        print(f"  - {name}: {dep}")
            print("")


def _main(arg: argparse.Namespace) -> None:
    core.load_plugins()
    list_plugins(arg, core.get_loaded_plugins(),
                 core.get_plugins_dependencies())


def main() -> None:
    _main(parser().parse_args())
