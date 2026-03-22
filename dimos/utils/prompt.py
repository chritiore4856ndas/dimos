# Copyright 2025-2026 Dimensional Inc.
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

"""Safe interactive prompts for module code.

Use ``confirm`` instead of bare ``typer.confirm`` or ``input()`` in any
code that may run during module build/start (non-interactive environments).
"""

from __future__ import annotations

import sys
from typing import Any


def confirm(message: str, *, default: bool = True, **kwargs: Any) -> bool:
    """Ask yes/no. Returns *default* if stdin is not a TTY."""
    if not sys.stdin.isatty():
        return default
    import typer

    return typer.confirm(message, default=default, **kwargs)
