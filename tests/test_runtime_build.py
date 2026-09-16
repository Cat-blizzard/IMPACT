import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from verify_runtime_build import MARKER, inventory, verify


@pytest.fixture
def bound(tmp_path):
    install = tmp_path / "install"
    install.mkdir()
    (install / "node.py").write_text("version = 1\n")
    m = dict(schema_version=2, install_root=str(install), build_kind="full",
             git={"code": 0, "output": "commit-a"}, source_sha256="source-a",
             installed_files=inventory(install))
    external = tmp_path / "build-manifest.json"
    for p in [external, install / MARKER]:
        p.write_text(json.dumps(m))
    return install, external


def test_matching_install_is_verified(bound):
    assert verify(*bound, "source-a", "commit-a", False)["passed"]


def test_new_source_rejects_old_build_even_if_external_marker_exists(bound):
    with pytest.raises(ValueError, match="source differs"):
        verify(*bound, "source-b", "commit-a", False)


def test_new_commit_rejects_old_build_even_if_source_tree_is_identical(bound):
    with pytest.raises(ValueError, match="Git HEAD differs"):
        verify(*bound, "source-a", "commit-b", False)


@pytest.mark.parametrize("change", ["edit", "delete", "extra"])
def test_installed_bytes_cannot_be_approved_by_unchanged_manifest(bound, change):
    install, _ = bound
    if change == "edit":
        (install / "node.py").write_text("version = 2\n")
    elif change == "delete":
        (install / "node.py").unlink()
    else:
        (install / "unexpected.so").write_text("stale overlay")
    with pytest.raises(ValueError, match="inventory mismatch"):
        verify(*bound, "source-a", "commit-a", False)


def test_external_manifest_cannot_describe_another_install(bound, tmp_path):
    install, external = bound
    m = json.loads(external.read_text())
    m["install_root"] = str(tmp_path / "other")
    external.write_text(json.dumps(m))
    with pytest.raises(ValueError, match="external manifest"):
        verify(*bound, "source-a", "commit-a", False)


def test_python_caches_do_not_change_installed_code_identity(bound):
    (bound[0] / "__pycache__").mkdir()
    (bound[0] / "__pycache__/node.pyc").write_bytes(b"cache")
    assert verify(*bound, "source-a", "commit-a", False)["passed"]
