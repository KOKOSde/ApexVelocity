def test_loader_imports_and_surface():
    # Loader should import even if osmnx is not installed; instantiation may fail.
    import apexvelocity.loader as loader

    assert hasattr(loader, "OSMLoader")
    assert hasattr(loader, "LoadedPath")
    # Class should have expected methods/attributes
    assert hasattr(loader.OSMLoader, "graph_from_place")


def test_osm_fetcher_marked_experimental():
    import apexvelocity.osm_fetcher as of

    doc = (of.__doc__ or "").lower()
    assert "experimental" in doc
