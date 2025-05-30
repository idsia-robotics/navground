from navground import core
import numpy as np

scalar_types = [int, float, bool, str, core.Vector2]


def make_value(type_name: str):
    if type_name in ('int', 'str', 'bool', 'float'):
        return eval(type_name)(1)
    if type_name == 'vector':
        return np.ones(2, dtype=np.float32)
    scalar_type = type_name[1:-1]
    return [make_value(scalar_type)]


def equal(type_name, v1, v2):
    if type_name == 'vector':
        return all(v1 == v2)
    elif type_name == '[vector]':
        return np.all(x == y for x, y in zip(v1, v2, strict=True))
    else:
        return v1 == v2


valid_conversions = {
    'int': ('float', 'bool'),
    'float': ('int', 'bool'),
    'bool': ('int', 'float'),
    'vector': ('[int]', '[float]', '[bool]'),
    '[int]': ('[float]', '[bool]', 'vector'),
    '[float]': ('[int]', '[bool]', 'vector'),
    '[bool]': ('[int]', '[float]', 'vector'),
}


class M(core.BehaviorModulation, name="M"):
    _vars = {
        t.__name__ if t is not core.Vector2 else 'vector':
        t() if t is not core.Vector2 else np.zeros(2, dtype=np.float32)
        for t in scalar_types
    }
    _lvars = {
        t.__name__ if t is not core.Vector2 else 'vector': []
        for t in scalar_types
    }

    for name, default in _vars.items():

        r_type = 'core.Vector2' if name == 'vector' else name

        def fget(self, name=name):
            return self._vars[name]

        fget.__annotations__['return'] = r_type
        fget.__doc__ = f'my {name} property'

        def fset(self, value, name=name):
            self._vars[name] = value

        locals()[f'p_{name}'] = property(
            fget=core.register(default_value=default)(fget), fset=fset)

    for name, default in _lvars.items():

        r_type = 'core.Vector2' if name == 'vector' else name

        def fget(self, name=name):
            return self._lvars[name]

        fget.__annotations__['return'] = f'list[{r_type}]'
        fget.__doc__ = f'my [{name}] property'

        def fset(self, value, name=name):
            self._lvars[name] = value

        locals()[f'p_{name}s'] = property(
            fget=core.register(default_value=default)(fget), fset=fset)


def test_type_name():
    m = M()
    for n, p in m.properties.items():
        if n[-1] == 's':
            type_name = f'[{n[2:-1]}]'
        else:
            type_name = n[2:]
        assert p.type_name == type_name


def test_getter():
    m = M()
    for n, p in m.properties.items():
        assert type(m.get(n)) is type(p.default_value)


def test_setter():
    m = M()
    for n, p in m.properties.items():
        value = make_value(p.type_name)
        m.set(n, value)
        assert equal(p.type_name, m.get(n), value)


def test_number_conversion():
    m = M()
    for t1 in (float, bool, int):
        for t2 in (float, bool, int):
            m.set(f'p_{t1.__name__}', t2(2))
            assert m.get(f'p_{t1.__name__}') == t1(t2(2))


def test_number_list_conversion():
    m = M()
    for t1 in (float, bool, int):
        for t2 in (float, bool, int):
            m.set(f'p_{t1.__name__}s', [t2(2)])
            assert m.get(f'p_{t1.__name__}s') == [t1(t2(2))]


def test_vector_conversion():
    m = M()
    for t1 in (float, bool, int):
        type_name = f'[{t1.__name__}]'
        name = f'p_{t1.__name__}s'
        v_value = np.full(2, float(t1(3)), np.float32)
        l_value = [t1(3) for _ in range(2)]
        m.set(name, v_value)
        assert equal(type_name, m.get(name), l_value)
        m.set('p_vector', l_value)
        assert equal('vector', m.get('p_vector'), v_value)


def test_invalid_conversion():
    m = M()
    all_type_names = set(p.type_name for p in m.properties.values())
    for n, p in m.properties.items():
        original_value = m.get(n)
        types = all_type_names - set(valid_conversions.get(p.type_name, ()))
        types -= {p.type_name}
        print(p.type_name, types)
        for t in types:
            value = make_value(t)
            m.set(n, value)
            assert equal(p.type_name, m.get(n), original_value)
