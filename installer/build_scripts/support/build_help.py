#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import asyncio
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

from cool_cache import cache, settings as cool_cache_settings

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent.parent.parent
PYPROJECT_PATH = REPO_ROOT / "pyproject.toml"
INSTALLER_DIR = SCRIPT_DIR.parent.parent
OUT_PATH = INSTALLER_DIR / "installer.pyz"
BUILD_DIR = INSTALLER_DIR / ".build_pyz"
APP_SRC = INSTALLER_DIR / "pyz_app"
APP_DEST = BUILD_DIR / "app" / "pyz_app"
REQUIREMENTS = APP_SRC / "requirements.txt"

DISTRIBUTED_DEP_DB_DIR = INSTALLER_DIR / "dep_database.ignore"
CONSOLIDATED_DEP_DB_DIR = APP_SRC / "bundled_files" / "pip_dependency_database.json"
DEPENDENCY_OUT = APP_SRC / "bundled_files" / "pip_dependency_database.json"
PYPROJECT_LINK = APP_SRC / "bundled_files" / "pyproject.toml"

try:
    import tomllib  # Python 3.11+
except ModuleNotFoundError:  # pragma: no cover
    import tomli as tomllib  # type: ignore

cool_cache_settings.default_folder = "cache.ignore/"

DEP_LIST_KEYS = ["apt_dependencies", "brew_dependencies", "nix_dependencies"]
REQUIRED_KEYS = ["requirement", *DEP_LIST_KEYS]


async def _run_cmd_async(*args: str, cwd: Path | None = None, inherit_io: bool = False) -> None:
    """Run a subprocess and raise on failure."""
    kwargs = {}
    if cwd:
        kwargs["cwd"] = str(cwd)
    if inherit_io:
        kwargs.update(stdin=sys.stdin, stdout=sys.stdout, stderr=sys.stderr)
    proc = await asyncio.create_subprocess_exec(*args, **kwargs)
    code = await proc.wait()
    if code != 0:
        raise RuntimeError(f"Command {' '.join(args)} failed with exit code {code}")


async def reset_build_dir() -> None:
    if BUILD_DIR.exists():
        await asyncio.to_thread(shutil.rmtree, BUILD_DIR)
    await asyncio.to_thread((BUILD_DIR / "app").mkdir, parents=True, exist_ok=True)


def read_distributed_dep_json() -> dict:
    aggregated: dict[str, object] = {}
    for path in sorted(DISTRIBUTED_DEP_DB_DIR.glob("*.json")):
        name = path.stem.lower()
        try:
            aggregated[name] = json.loads(path.read_text())
        except Exception as exc:  # pragma: no cover - build-time guard
            print(f"{name} had an error: {exc}", file=sys.stderr)
    return aggregated


def read_consolidated_dep_json() -> dict:
    with open(CONSOLIDATED_DEP_DB_DIR) as in_file:
        return json.load(in_file)


def consolidate_and_validate_distributed_deps() -> None:
    """Aggregate dep_database JSON and hardlink pyproject into bundled_files."""
    aggregated = read_distributed_dep_json()
    DEPENDENCY_OUT.parent.mkdir(parents=True, exist_ok=True)
    print("- saving unvalidated json")
    DEPENDENCY_OUT.write_text(json.dumps(aggregated, indent=2, sort_keys=True) + "\n")
    aggregated = validate_names_and_load(aggregated)
    print("- saving validated json")
    DEPENDENCY_OUT.write_text(json.dumps(aggregated, indent=2, sort_keys=True) + "\n")

    PYPROJECT_LINK.parent.mkdir(parents=True, exist_ok=True)
    try:
        if PYPROJECT_LINK.exists():
            try:
                if PYPROJECT_LINK.samefile(PYPROJECT_PATH):
                    return
            except FileNotFoundError:
                pass
            PYPROJECT_LINK.unlink()
        os.link(PYPROJECT_PATH, PYPROJECT_LINK)
    except Exception as exc:
        print(f"Failed to hardlink pyproject.toml: {exc}", file=sys.stderr)


async def copy_app_sources() -> None:
    """Copy the pyz_app sources into the build directory."""
    if shutil.which("rsync"):
        await _run_cmd_async("rsync", "-a", f"{APP_SRC}/", str(APP_DEST))
    else:  # pragma: no cover - fallback path
        await asyncio.to_thread(shutil.copytree, APP_SRC, APP_DEST, dirs_exist_ok=True)


async def install_dependencies_into_pyz() -> None:
    """Install Python dependencies into the build app directory."""
    await _run_cmd_async(
        sys.executable,
        "-m",
        "pip",
        "install",
        "-r",
        str(REQUIREMENTS),
        "-t",
        str(BUILD_DIR / "app"),
    )


async def call_build_pyz() -> None:
    """Create the zipapp from the prepared build directory."""
    await _run_cmd_async(
        sys.executable,
        "-m",
        "zipapp",
        str(BUILD_DIR / "app"),
        "-o",
        str(OUT_PATH),
        "-m",
        "pyz_app.__main__:main",
        inherit_io=True,
    )


async def unconsolidate_deps() -> None:
    """
    If a combined pip_dependency_database.json exists, split it into per-package files.

    Overwrites existing per-package files to keep them in sync before prompting for
    any missing entries.
    """
    consolidated_deps = read_consolidated_dep_json()
    DISTRIBUTED_DEP_DB_DIR.mkdir(parents=True, exist_ok=True)

    for name, entry in consolidated_deps.items():
        dest = DISTRIBUTED_DEP_DB_DIR / f"{name}.json"
        # Keep the existing structure as-is; sorted keys for readability.
        text = json.dumps(entry, indent=2, sort_keys=True) + "\n"
        await asyncio.to_thread(dest.write_text, text)


async def get_pip_deps_from_pyproject() -> list[str]:
    """
    Load dependency lists from pyproject.toml.

    Example:
        >>> deps = asyncio.run(_load_dependencies())
        >>> isinstance(deps, list)
        True
    """
    raw = await asyncio.to_thread(PYPROJECT_PATH.read_bytes)
    data = tomllib.loads(raw.decode())
    project = data.get("project", {})
    deps = list(project.get("dependencies", []))
    for _, extras in project.get("optional-dependencies", {}).items():
        deps.extend(extras)
    return deps


async def existing_entry_is_complete(path: Path) -> bool:
    """
    Check whether a dep_database entry has all required keys.

    Example:
        >>> tmp = "_example.json"
        >>> tmp.write_text('{"requirement":"foo","apt_dependencies":[],"brew_dependencies":[],"nix_dependencies":[]}')
        >>> asyncio.run(existing_entry_is_complete(tmp))
        True
    """
    try:
        raw = await asyncio.to_thread(path.read_text)
        obj = json.loads(raw)
    except Exception:
        return False

    overlap = set(obj.keys()) & set(REQUIRED_KEYS)
    if len(overlap) != len(REQUIRED_KEYS):
        return False
    return all(isinstance(obj.get(key), list) for key in DEP_LIST_KEYS)


def normalize_pip_requirement(requirement: str) -> str:
    """
    Example:
        >>> normalize_pip_requirement("Torch>=2.0; python_version>='3.9'")
        'torch'
    """
    return re.sub(r"[=>,;].+", "", requirement).strip().lower()


@cache()
def is_valid_brew_package_name(name: str) -> bool:
    res = subprocess.run(["brew", "info", name], capture_output=True)
    return b"No available formula" not in res.stdout


@cache()
def is_valid_apt_package_name(name: str) -> bool:
    res = subprocess.run(
        ["apt-cache", "show", name],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    return not bool(res.returncode)


@cache()
def get_valid_nixpkgs_attr_name(name: str) -> str | None:
    if name.startswith("stdenv.cc"):
        return None
    # surprisingly-annoyingly hard (to do in any acceptable amount of time)
    if shutil.which("nix-search") is None:
        print("please install nix-search: https://github.com/peterldowns/nix-search-cli")
        return name
    if shutil.which("nvs") is None:
        print(
            "please install nvs:\n    nix profile install 'https://github.com/jeff-hykin/nix_version_search_cli/archive/50a3fef5c9826d1e08b360b7255808e53165e9b2.tar.gz#nvs'"
        )
        return name

    # remove prefix
    if name.startswith("pkgs."):
        name = name[5:]
    name = name.lower()
    res = subprocess.run(
        ["nvs", name, "--json"],
        capture_output=True,
        text=True,
    )
    nvs_json_object = None
    try:
        stdout = res.stdout
        if stdout.startswith("\nNo exact results, let me broaden the search...\n\n"):
            stdout = stdout[len("\nNo exact results, let me broaden the search...\n\n") :]
        nvs_json_object = json.loads(stdout)
    except Exception:
        # print(f"note: couldn't parse nvs output for {name}: {error}\nstdout:{stdout}\nstderr:{res.stderr}", file=sys.stderr)
        # this case should only happen if the name is not valid
        pass
    if nvs_json_object:
        for each_key, each_value in nvs_json_object.items():
            if each_key.lower() == name:
                return each_value["attrPath"]

        # starts with name optionally ends with number-like thing
        pattern = re.escape(name) + r"[\.\-_@]?([0-9\.\-_]*)$"
        prefixed_names = {
            key: value for key, value in nvs_json_object.items() if re.match(pattern, key.lower())
        }

        def get_number(value):
            number = re.match(pattern, value["attrPath"])[1]
            if len(number) == 0:
                return 0
            all_digits = re.sub(r"\D", "", number)
            # TODO: probably should do full version compare here
            return int(re.sub(r"\D", "", all_digits))

        sorted_prefixed_names = sorted(prefixed_names.values(), key=get_number, reverse=True)
        for value in sorted_prefixed_names:
            return value["attrPath"]

    # if this^ doesn't match anything we fall back on nix-search (which for some reason does't work well on basic stuff)

    res = subprocess.run(
        ["nix-search", name, "--json"],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    items = res.stdout.split("\n")
    packages = [json.loads(each) for each in items if each.strip() != ""]
    # if it matches an attrname, we are done (attrnames are unique)
    for each_pkg in packages:
        if each_pkg.get("package_attr_name").lower() == name:
            return each_pkg.get("package_attr_name")
    # stuff like "python" matches python310, python311, "python3-minimal", etc
    # we use the heuristic of finding the shortest one with the highest version number
    # e.g. if "python3-minimal" "python310" "python310" we choose "python310"
    pname_matches = sorted(
        [
            pkg
            for pkg in packages
            if pkg.get("package_pname", "").lower() == name and pkg.get("package_attr_name")
        ],
        key=lambda x: x["package_attr_name"],
    )
    # no such package
    if len(pname_matches) == 0:
        if "." in name:
            return get_valid_nixpkgs_attr_name(name.split(".", 1)[1])
        return None
    name_no_lib_prefix = name
    if name.startswith("lib"):
        name_no_lib_prefix = name[3:]
    name_no_lib_prefix = name_no_lib_prefix.lower()
    # prefer things that start with the name
    sorted_by_name_prefix = sorted(
        pname_matches,
        key=lambda x: -2
        if x["package_attr_name"].lower().startswith(name)
        else (-1 if x["package_attr_name"].lower().startswith(name_no_lib_prefix) else 0),
    )
    shortest_match_len = len(sorted_by_name_prefix[0].get("package_attr_name"))
    short_matches = [
        each for each in pname_matches if len(each["package_attr_name"]) == shortest_match_len
    ]
    return short_matches[-1][
        "package_attr_name"
    ]  # the last should be the largest number (its already sorted)


def validate_names_and_load(dep_db) -> dict:
    print()
    print(
        "NOTE: validation takes a while partly because its pinging endpoints.\n- Doing them in parallel will cause rate-limiting / ip ban\n- This function builds a cold storage cache so it should only be painfully slow the first time"
    )
    print()

    brew_removed = []
    if shutil.which("brew") is not None:
        print("validating brew packages")
        for name, each_pkg in dep_db.items():
            print(f"- {name}                    ", end="\r")
            time.sleep(0.05)
            if each_pkg.get("brew_dependencies", None) is not None:
                start = set(each_pkg["brew_dependencies"])
                each_pkg["brew_dependencies"] = [
                    each_dep
                    for each_dep in each_pkg["brew_dependencies"]
                    if is_valid_brew_package_name(each_dep)
                ]
                end = set(each_pkg["brew_dependencies"])
                brew_removed.extend(start - end)

    nix_changed = {}
    if shutil.which("nix") is not None:
        print("validating nix packages")
        for name, each_pkg in dep_db.items():
            print(f"- {name}                    ", end="\r")
            if each_pkg.get("nix_dependencies", None) is not None:
                start = set(each_pkg["nix_dependencies"])
                new_list = []
                for each in each_pkg["nix_dependencies"]:
                    new_name = get_valid_nixpkgs_attr_name(each)
                    if new_name != each:
                        nix_changed[each] = new_name
                    if new_name is not None:
                        new_list.append(new_name)
                each_pkg["nix_dependencies"] = new_list

    apt_removed = []
    if shutil.which("apt-cache") is not None:
        print("validating apt-cache packages")
        for name, each_pkg in dep_db.items():
            print(f"- {name}                    ", end="\r")
            if each_pkg.get("apt_dependencies", None) is not None:
                start = set(each_pkg["apt_dependencies"])
                each_pkg["apt_dependencies"] = [
                    each_dep
                    for each_dep in each_pkg["apt_dependencies"]
                    if is_valid_apt_package_name(each_dep)
                ]
                end = set(each_pkg["apt_dependencies"])
                apt_removed.extend(start - end)

    print(f"brew removed: {brew_removed}")
    print(f"nix changed: {nix_changed}")
    print(f"apt removed: {apt_removed}")

    return dep_db
