#!/usr/bin/env python3
"""Bind source identity, installed bytes and actual ROS/Python resolution before spawn."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

MARKER = ".impact-build-manifest.json"

# Keep the flight entrypoints bound to their actual installed modules.  The
# generic IMPACT runner and the dedicated P4 runner intentionally have
# different result schemas and launch graphs, so checking only ExternalNav is
# insufficient to identify what a run will execute.
REQUIRED_ENTRYPOINTS = {
    "xq_p4_external_nav": "xq_autonomy.p4_external_nav_node:main",
    "xq_p4_mission": "xq_autonomy.p4_mission_node:main",
    "impact_mission": "xq_autonomy.sitl_mission_node:main",
}


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def inventory(install):
    install = Path(install).resolve()
    result = {}
    for p in sorted(install.rglob("*")):
        if "__pycache__" in p.parts or p.suffix == ".pyc" or p.name == MARKER:
            continue
        if p.is_symlink() and not p.resolve().is_relative_to(install):
            raise ValueError(f"installed symlink escapes frozen tree: {p}")
        if p.is_file():
            result[p.relative_to(install).as_posix()] = sha256(p)
    return result


def check_installed_python(root, install):
    """Catch stale copied Python even when a build tool reports success."""
    checked = 0
    for package in (Path(root) / "src").iterdir():
        source = package / package.name
        if not (source / "__init__.py").is_file():
            continue
        targets = list((Path(install) / package.name / "lib").glob(
            f"python*/site-packages/{package.name}"))
        if len(targets) != 1:
            raise ValueError(f"missing/ambiguous installed Python package: {package.name}")
        for p in source.rglob("*.py"):
            target = targets[0] / p.relative_to(source)
            if not target.is_file() or sha256(p) != sha256(target):
                raise ValueError(f"installed Python differs from source: {target}")
            checked += 1
    return checked


def resolve_runtime_entrypoints(install, expected_files):
    """Resolve console-script metadata and Python origins in the sourced env."""
    code = r'''
import importlib, importlib.metadata, json, sys
names = json.loads(sys.argv[1])
dist = importlib.metadata.distribution("xq-autonomy")
entries = {entry.name: entry.value for entry in dist.entry_points
           if entry.group == "console_scripts" and entry.name in names}
modules = {}
for target in names.values():
    module_name = target.split(":", 1)[0]
    module = importlib.import_module(module_name)
    modules[module_name] = str(module.__file__)
print(json.dumps({"entries": entries, "modules": modules}))
'''
    result = subprocess.run(
        [sys.executable, "-c", code, json.dumps(REQUIRED_ENTRYPOINTS)],
        check=True, capture_output=True, text=True, timeout=10,
    )
    resolved = json.loads(result.stdout)
    if resolved.get("entries") != REQUIRED_ENTRYPOINTS:
        raise ValueError(
            f"installed console entrypoints differ: {resolved.get('entries')}"
        )
    module_paths = {}
    for target in REQUIRED_ENTRYPOINTS.values():
        module_name = target.split(":", 1)[0]
        module = Path(resolved["modules"][module_name]).resolve()
        if not module.is_relative_to(install):
            raise ValueError(f"Python resolves {module_name} outside frozen install: {module}")
        relative = module.relative_to(install).as_posix()
        if relative not in expected_files:
            raise ValueError(f"Python module is absent from frozen inventory: {module}")
        module_paths[module_name] = str(module)
    return {"console_scripts": REQUIRED_ENTRYPOINTS, "python_modules": module_paths}


def verify(install, supplied_manifest, current_source_hash, current_git=None,
           resolve_environment=True):
    install = Path(install).resolve(strict=True)
    manifest_path = install / MARKER
    manifest = json.loads(manifest_path.read_text())
    if manifest != json.loads(Path(supplied_manifest).read_text()):
        raise ValueError("external manifest does not match the actual install manifest")
    if manifest.get("schema_version") != 2 or manifest.get("build_kind") != "full":
        raise ValueError("a complete bound build is required")
    if manifest.get("install_root") != str(install):
        raise ValueError("manifest belongs to a different installation")
    if manifest.get("source_sha256") != current_source_hash:
        raise ValueError("source differs from the built snapshot; rebuild before running")
    build_git = manifest.get("git", {})
    if current_git is not None and (build_git.get("code") != 0 or build_git.get("output") != current_git):
        raise ValueError("Git HEAD differs from the build manifest; rebuild after committing")
    actual = inventory(install)
    expected = manifest.get("installed_files", {})
    changed = sorted(k for k in actual.keys() | expected.keys() if actual.get(k) != expected.get(k))
    if not expected or changed:
        raise ValueError(f"installed file inventory mismatch: {changed[:10]}")
    resolved = {}
    if resolve_environment:
        for name in manifest["installed_packages"]:
            # Use the same sourced environment as the processes about to launch.
            p = subprocess.run([sys.executable, "-c",
                "from ament_index_python.packages import get_package_prefix; "
                "import sys; print(get_package_prefix(sys.argv[1]))", name],
                check=True, capture_output=True, text=True, timeout=10)
            prefix = Path(p.stdout.strip()).resolve()
            if prefix != install / name:
                raise ValueError(f"ROS resolves {name} outside selected installation: {prefix}")
            resolved[name] = str(prefix)
        p = subprocess.run([sys.executable, "-c",
            "import xq_autonomy.p4_external_nav_node as n; print(n.__file__)"],
            check=True, capture_output=True, text=True, timeout=10)
        module = Path(p.stdout.strip()).resolve()
        if not module.is_relative_to(install) or module.relative_to(install).as_posix() not in expected:
            raise ValueError(f"Python resolves ExternalNav outside frozen inventory: {module}")
        resolved["external_nav_module"] = str(module)
        resolved["flight_entrypoints"] = resolve_runtime_entrypoints(install, expected)
    return dict(passed=True, install_root=str(install), build_git=manifest["git"],
                source_sha256=current_source_hash, manifest_sha256=sha256(manifest_path),
                installed_files_checked=len(actual), resolved=resolved)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--install", required=True)
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--output", required=True)
    args = ap.parse_args()
    from impact import source_hash, capture, ROOT, write_json
    try:
        runtime_git = capture(["git", "-C", str(ROOT), "rev-parse", "HEAD"])
        if runtime_git["code"] != 0:
            raise ValueError("cannot resolve runtime Git HEAD")
        report = verify(args.install, args.manifest, source_hash(), runtime_git["output"])
        report["runtime_git"] = runtime_git
    except Exception as e:
        report = dict(passed=False, install_root=args.install, error=str(e))
    write_json(args.output, report)
    print(json.dumps(report, ensure_ascii=False))
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
