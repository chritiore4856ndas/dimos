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

"""
Uses Claude to generate a mapping (dep_db) from a pip package name to system-dependencies for that package

!IMPORTANT Notes!
- The answers do not need to be perfect, they just make the user-install more smooth the more accurate they are
- The answers are validated to make sure those package names actually exist
- Once the values have been calculated they are cached (claude is only needed for new pip dependencies)
- The output can be hand-edited without claude overwriting the edits
"""

import argparse
import asyncio
import json
from pathlib import Path
import sys

from support.build_help import (
    DEP_LIST_KEYS,
    DISTRIBUTED_DEP_DB_DIR,
    REQUIRED_KEYS,
    consolidate_and_validate_distributed_deps,
    existing_entry_is_complete,
    get_pip_deps_from_pyproject,
    normalize_pip_requirement,
    unconsolidate_deps,
)
from support.claude import run_claude_named_prompts

DEFAULT_CONCURRENT_CLAUDE_REQUESTS = 5


def _build_prompt(name: str, requirement: str) -> str:
    """
    Build the Claude prompt for a dependency.

    Example:
        >>> _build_prompt("torch", "torch>=2.0")
        'list all apt-get dependencies, nix, and brew dependencies for the torch>=2.0 pip module...'
    """
    key_list = " ".join(json.dumps(key) for key in REQUIRED_KEYS)
    dep_keys = " ".join(json.dumps(key) for key in DEP_LIST_KEYS)
    return (
        f"list all apt-get dependencies, nix, and brew dependencies for the {requirement} "
        f"pip module. The result should be a json object with the following {key_list} "
        f'and optionally "description", "notes". These ({dep_keys}) should be list of '
        f"strings. Store that resulting json inside {DISTRIBUTED_DEP_DB_DIR}/{name}.json"
    )


async def _gather_prompts(dependencies: list[str]) -> tuple[list[tuple[str, str]], list[str]]:
    """
    Build prompts for deps that are missing or incomplete.

    Example:
        >>> asyncio.run(_gather_prompts(Path("."), ["torch>=2.0"]))
        ([(..., ...)], [...])
    """
    prompts: list[tuple[str, str]] = []
    missing: list[str] = []

    for requirement in dependencies:
        name = normalize_pip_requirement(requirement)
        if name.startswith("types-") or name.endswith("-stubs") or name.startswith("pytest-"):
            continue

        dest_path = DISTRIBUTED_DEP_DB_DIR / f"{name}.json"
        if not dest_path.exists():
            missing.append(name)
            prompts.append((name, _build_prompt(name, requirement)))
            continue

        if await existing_entry_is_complete(dest_path):
            continue

        prompts.append((name, _build_prompt(name, requirement)))

    return prompts, missing


async def main(argv: list[str]) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true", help="List work without calling claude.")
    parser.add_argument(
        "--max-concurrent", type=int, default=5, help="Number of simultaneous claude prompts."
    )
    parser.add_argument("--log-dir", default="./.claude", help="Where to store claude logs.")
    parser.add_argument("extra", nargs="*", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)

    await unconsolidate_deps()
    dependencies = await get_pip_deps_from_pyproject()
    prompts, missing = await _gather_prompts(dependencies)
    tools_to_ask_about = [each[0] for each in prompts]
    print("asking claude about:")
    for tool in tools_to_ask_about:
        print("- " + tool)

    list_only = args.dry_run or bool(args.extra)
    if list_only:
        for name, _ in prompts:
            status = "missing" if name in missing else "needs modification"
            print(f"{status:18}: {name}.json")
        total = len(prompts)
        print(f"total: {total}")
        print(f"missing: {len(missing)}")
        print(f"need modification: {total - len(missing)}")
        return

    if not prompts:
        print("No prompts to run; dep database already complete.")
        return

    await run_claude_named_prompts(
        prompts,
        max_concurrent=max(args.max_concurrent, 1),
        log_dir=Path(args.log_dir),
    )

    print()
    print("All data gathered; now validating and consolidating")
    consolidate_and_validate_distributed_deps()


if __name__ == "__main__":
    try:
        asyncio.run(main(sys.argv[1:]))
    except KeyboardInterrupt:
        pass
