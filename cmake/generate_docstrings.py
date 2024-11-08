import os
import re
import subprocess
import sys
import pathlib


def ref(_renamed_classes, _methods, _force_properties):
    def f(matchobj):
        namespace, name = matchobj.groups()
        owner = ''
        if namespace:
            if namespace[0].isupper():
                namespace = namespace.split("::")[-2]
                owner = _renamed_classes.get(namespace, namespace)
            else:
                print(f"Unexpected namespace {namespace}", sys.stderr)
        if owner:
            owner += "."

        if name[0].isupper():
            # it's a class
            name = _renamed_classes.get(name, name)
            return f":py:class:`{name}`"
        if 'set_' in name or 'get_' in name:
            pname = name[4:]
            if pname in _force_properties:
                return f":py:attr:`{owner}{pname}`"
        if name in _methods:
            return f":py:meth:`{owner}{name}`"
        if 'set_' in name or 'get_' in name:
            name = name[4:]
        return f":py:attr:`{owner}{name}`"

    return f


def main():

    destination = sys.argv[1]
    python_src = sys.argv[2]
    library = sys.argv[3]
    deps = sys.argv[4:]
    deps = [str(pathlib.Path(p).as_posix()) for p in deps]

    headers = []

    for path, subdirs, files in os.walk(library):
        for name in files:
            if name.endswith(".h"):
                headers.append(os.path.join(path, name))

    includes = sum([["-I", path] for path in deps], [])

    headers = [str(pathlib.Path(p).as_posix()) for p in headers]

    args = [sys.executable, "-m", "pybind11_mkdoc"
            ] + headers + includes + ["--std=c++17"]

    # if it runs in a venv ...
    result = subprocess.run(args, capture_output=True, env=os.environ)

    # destination = 'docstrings.h'

    try:
        header = result.stdout.decode("utf-8")
    except:
        header = ''

    if result.returncode or len(header) == 0:
        print("Failed running pybind11_mkdoc", file=sys.stderr)
        print(result.stderr.decode("ascii"), file=sys.stderr)
        print(result.stdout.decode("ascii"), file=sys.stdout)
        with open(destination, 'w') as f:
            f.write('#define DOC(...) ""')
        return

    with open(python_src, 'r') as f:
        python = f.read()

    _renamed_classes = {
        "NativeAgent": "Agent",
        "NativeWorld": "World",
        "Heading": "Behavior.Heading",
        "Field": "PropertyField"
    }

    _methods = re.findall(r".def\(\"(\w+)\",", python)
    _properties = re.findall(r".def_property\(\"(\w+)\",", python)

    header = re.sub(r"\\ref\s+(\w*::)*(\w+)", ref(_renamed_classes, _methods, {'max_angular_speed'}), header)
    extras = ''
    exposed_properties = set()

    for owner, name, doc in re.findall(r"static const char \*__doc_(\w+)_get_(\w+) =\s*R\"doc\(([\s\S\w\W.]*?)(?=:return:|\)doc)", header):
        p = f'{owner}_property_{name}'
        if p in exposed_properties:
            continue
        exposed_properties.add(p)
        extras += "\n"
        if doc.startswith("Gets "):
            doc = doc[5:]
            doc = doc.capitalize()
        doc = doc[:-1]
        extras += f'static const char *__doc_{owner}_property_{name} =\nR"doc({doc})doc";\n'

    for ns, doc in re.findall(r"static const char \*__doc_(\w+) =\s*R\"doc\(([\s\S\w\W.]*?)(?=:return:|\)doc)", header):
        ss = ns.split('_')
        if len(ss) < 4:
            continue
        if ss[2] == "get" or ss[2] == "set":
            continue
        owner = '_'.join(ss[:3])
        name = '_'.join(ss[3:])
        if name not in _properties:
            continue
        p = f'{owner}_property_{name}'
        if p in exposed_properties:
            continue
        exposed_properties.add(p)
        extras += "\n"
        if doc.startswith("Returns "):
            doc = doc[8:]
            doc = doc.capitalize()
        doc = doc[:-1]
        extras += f'static const char *__doc_{owner}_property_{name} =\nR"doc({doc})doc";\n'

    header += extras
    docs = ['__doc_' + '_'.join([y.strip() for y in x.split(',')])
            for x in re.findall(r"DOC\(([^\)]*)\)", python) if '/' not in x]
    extras = '\n\n// ********** missing docstrings ********** //\n\n'
    for doc in docs:
        if doc not in header:
            # warnings.warn(f"missing {doc}")
            print(f"missing {doc}", file=sys.stderr)
            extras += f'static const char *{doc} = "";\n'

    header += extras

    print('Save docstrings to', destination, file=sys.stderr)

    with open(destination, 'w') as f:
        f.write(header)


if __name__ == '__main__':
    main()
