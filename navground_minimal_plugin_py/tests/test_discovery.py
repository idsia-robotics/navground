def test_discovery():
    import navground.core
    navground.core.load_plugins()

    assert "Minimal" in navground.core.Behavior.types
