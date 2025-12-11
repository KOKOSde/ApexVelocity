import re
import os


def test_version_matches_pyproject():
    import apexvelocity as av

    repo_root = os.path.dirname(os.path.dirname(__file__))
    pyproject_path = os.path.join(repo_root, "pyproject.toml")
    with open(pyproject_path, "r", encoding="utf-8") as f:
        content = f.read()
    m = re.search(r"^version\s*=\s*\"([^\"]+)\"", content, re.MULTILINE)
    assert m, "Could not find version in pyproject.toml"
    py_version = m.group(1)
    assert av.__version__ == py_version
