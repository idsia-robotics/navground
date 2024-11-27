import sys

def test_import():
    try:
        import navground.sim
    except Exception as e:
        print(e, file=sys.stderr)
        raise e


