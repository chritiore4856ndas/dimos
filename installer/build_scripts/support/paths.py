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

from pathlib import Path

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
