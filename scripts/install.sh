#!/usr/bin/env bash
# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0
#
# Interactive installer for DimOS — the agentive operating system for generalist robotics.
#
# Usage:
#   curl -fsSL https://dimensional.ai/install.sh | bash
#   curl -fsSL https://dimensional.ai/install.sh | bash -s -- --help
#
# Non-interactive:
#   curl -fsSL https://dimensional.ai/install.sh | bash -s -- --non-interactive --mode library --extras base,unitree
#
set -euo pipefail

# ─── version ──────────────────────────────────────────────────────────────────
INSTALLER_VERSION="0.1.0"

# ─── defaults ─────────────────────────────────────────────────────────────────
INSTALL_MODE="${DIMOS_INSTALL_MODE:-}"          # library | dev
EXTRAS="${DIMOS_EXTRAS:-}"                      # comma-separated extras
NON_INTERACTIVE="${DIMOS_NO_PROMPT:-0}"
GIT_BRANCH="${DIMOS_BRANCH:-dev}"
NO_CUDA="${DIMOS_NO_CUDA:-0}"
NO_SYSCTL="${DIMOS_NO_SYSCTL:-0}"
DRY_RUN="${DIMOS_DRY_RUN:-0}"
PROJECT_DIR="${DIMOS_PROJECT_DIR:-}"
VERBOSE=0

# ─── colors (matching DimOS theme: cyan #00eeee, white #b5e4f4) ──────────────
if [[ -t 1 ]] && command -v tput &>/dev/null && [[ $(tput colors 2>/dev/null || echo 0) -ge 8 ]]; then
    CYAN=$'\033[38;5;44m'
    GREEN=$'\033[32m'
    YELLOW=$'\033[33m'
    RED=$'\033[31m'
    BOLD=$'\033[1m'
    DIM=$'\033[2m'
    RESET=$'\033[0m'
else
    CYAN="" GREEN="" YELLOW="" RED="" BOLD="" DIM="" RESET=""
fi

# ─── helpers ──────────────────────────────────────────────────────────────────
info()  { printf "%s▸%s %s\n" "$CYAN" "$RESET" "$*"; }
ok()    { printf "%s✓%s %s\n" "$GREEN" "$RESET" "$*"; }
warn()  { printf "%s⚠%s %s\n" "$YELLOW" "$RESET" "$*" >&2; }
err()   { printf "%s✗%s %s\n" "$RED" "$RESET" "$*" >&2; }
die()   { err "$@"; exit 1; }
dim()   { printf "%s%s%s\n" "$DIM" "$*" "$RESET"; }

run_cmd() {
    if [[ "$DRY_RUN" == "1" ]]; then
        dim "[dry-run] $*"
        return 0
    fi
    if [[ "$VERBOSE" == "1" ]]; then
        dim "$ $*"
    fi
    eval "$@"
}

has_cmd() {
    command -v "$1" &>/dev/null
}

prompt_yn() {
    local msg="$1" default="${2:-y}"
    if [[ "$NON_INTERACTIVE" == "1" ]]; then
        [[ "$default" == "y" ]]
        return
    fi
    local yn
    if [[ "$default" == "y" ]]; then
        printf "%s [Y/n] " "$msg"
    else
        printf "%s [y/N] " "$msg"
    fi
    read -r yn </dev/tty || yn="$default"
    yn="${yn:-$default}"
    [[ "$yn" =~ ^[Yy] ]]
}

prompt_choice() {
    local msg="$1" default="$2"
    shift 2
    local -a options=("$@")
    if [[ "$NON_INTERACTIVE" == "1" ]]; then
        echo "$default"
        return
    fi
    printf "\n%s%s%s\n\n" "$BOLD" "$msg" "$RESET"
    local i=1
    for opt in "${options[@]}"; do
        if [[ "$i" == "$default" ]]; then
            printf "  %s❯%s %s\n" "$CYAN" "$RESET" "$opt"
        else
            printf "    %s\n" "$opt"
        fi
        ((i++))
    done
    printf "\n  enter choice [%s]: " "$default"
    local choice
    read -r choice </dev/tty || choice="$default"
    choice="${choice:-$default}"
    echo "$choice"
}

prompt_multi() {
    local msg="$1"
    shift
    local -a items=("$@")
    if [[ "$NON_INTERACTIVE" == "1" ]]; then
        local all=""
        for ((i=1; i<=${#items[@]}; i++)); do
            [[ -n "$all" ]] && all+=","
            all+="$i"
        done
        echo "$all"
        return
    fi
    printf "\n%s%s%s %s(comma-separated numbers, enter for all)%s\n\n" "$BOLD" "$msg" "$RESET" "$DIM" "$RESET"
    local i=1
    for item in "${items[@]}"; do
        printf "  %s%d%s) %s\n" "$CYAN" "$i" "$RESET" "$item"
        ((i++))
    done
    printf "\n  selection: "
    local sel
    read -r sel </dev/tty || sel=""
    if [[ -z "$sel" ]]; then
        local all=""
        for ((i=1; i<=${#items[@]}; i++)); do
            [[ -n "$all" ]] && all+=","
            all+="$i"
        done
        echo "$all"
    else
        echo "$sel"
    fi
}

# ─── ascii banner ─────────────────────────────────────────────────────────────
show_banner() {
    if [[ "$NON_INTERACTIVE" == "1" ]] && [[ -z "${DIMOS_SHOW_BANNER:-}" ]]; then
        return
    fi
    printf "\n"
    while IFS= read -r line; do
        printf "%s%s%s\n" "$CYAN" "$line" "$RESET"
        sleep 0.03
    done <<'BANNER'
   ▇▇▇▇▇▇╗ ▇▇╗▇▇▇╗   ▇▇▇╗▇▇▇▇▇▇▇╗▇▇▇╗   ▇▇╗▇▇▇▇▇▇▇╗▇▇╗ ▇▇▇▇▇▇╗ ▇▇▇╗   ▇▇╗ ▇▇▇▇▇╗ ▇▇╗
   ▇▇╔══▇▇╗▇▇║▇▇▇▇╗ ▇▇▇▇║▇▇╔════╝▇▇▇▇╗  ▇▇║▇▇╔════╝▇▇║▇▇╔═══▇▇╗▇▇▇▇╗  ▇▇║▇▇╔══▇▇╗▇▇║
   ▇▇║  ▇▇║▇▇║▇▇╔▇▇▇▇╔▇▇║▇▇▇▇▇╗  ▇▇╔▇▇╗ ▇▇║▇▇▇▇▇▇▇╗▇▇║▇▇║   ▇▇║▇▇╔▇▇╗ ▇▇║▇▇▇▇▇▇▇║▇▇║
   ▇▇║  ▇▇║▇▇║▇▇║╚▇▇╔╝▇▇║▇▇╔══╝  ▇▇║╚▇▇╗▇▇║╚════▇▇║▇▇║▇▇║   ▇▇║▇▇║╚▇▇╗▇▇║▇▇╔══▇▇║▇▇║
   ▇▇▇▇▇▇╔╝▇▇║▇▇║ ╚═╝ ▇▇║▇▇▇▇▇▇▇╗▇▇║ ╚▇▇▇▇║▇▇▇▇▇▇▇║▇▇║╚▇▇▇▇▇▇╔╝▇▇║ ╚▇▇▇▇║▇▇║  ▇▇║▇▇▇▇▇▇▇╗
   ╚═════╝ ╚═╝╚═╝     ╚═╝╚══════╝╚═╝  ╚═══╝╚══════╝╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚═╝  ╚═╝╚══════╝
BANNER
    printf "\n"
    printf "   %sthe agentive operating system for generalist robotics%s\n" "$DIM" "$RESET"
    printf "   %sinstaller v%s%s\n\n" "$DIM" "$INSTALLER_VERSION" "$RESET"
}

# ─── argument parsing ─────────────────────────────────────────────────────────
usage() {
    cat <<EOF
${BOLD}DimOS Interactive Installer${RESET} v${INSTALLER_VERSION}

${BOLD}USAGE${RESET}
    curl -fsSL https://dimensional.ai/install.sh | bash
    curl -fsSL https://dimensional.ai/install.sh | bash -s -- [OPTIONS]

${BOLD}OPTIONS${RESET}
    --mode library|dev     Install mode (default: interactive prompt)
    --extras <list>        Comma-separated pip extras (e.g. base,unitree,drone)
    --branch <branch>      Git branch for dev mode (default: dev)
    --project-dir <path>   Project directory (default: ~/dimos-project or ~/dimos)
    --non-interactive      Accept defaults, no prompts
    --no-cuda              Force CPU-only (skip CUDA extras)
    --no-sysctl            Skip LCM sysctl configuration
    --dry-run              Print commands without executing
    --verbose              Show all commands being run
    --help                 Show this help

${BOLD}ENVIRONMENT VARIABLES${RESET}
    DIMOS_INSTALL_MODE     library | dev
    DIMOS_EXTRAS           Comma-separated extras
    DIMOS_NO_PROMPT        1 = non-interactive
    DIMOS_BRANCH           Git branch (default: dev)
    DIMOS_NO_CUDA          1 = skip CUDA
    DIMOS_NO_SYSCTL        1 = skip sysctl
    DIMOS_DRY_RUN          1 = dry run
    DIMOS_PROJECT_DIR      Project directory

${BOLD}EXAMPLES${RESET}
    # interactive install
    curl -fsSL https://dimensional.ai/install.sh | bash

    # non-interactive library install with unitree + drone
    curl -fsSL https://dimensional.ai/install.sh | bash -s -- \\
        --non-interactive --mode library --extras base,unitree,drone

    # developer install, cpu-only
    curl -fsSL https://dimensional.ai/install.sh | bash -s -- --mode dev --no-cuda

    # dry run to see what would happen
    curl -fsSL https://dimensional.ai/install.sh | bash -s -- --dry-run
EOF
    exit 0
}

parse_args() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --mode)            INSTALL_MODE="$2"; shift 2 ;;
            --extras)          EXTRAS="$2"; shift 2 ;;
            --branch)          GIT_BRANCH="$2"; shift 2 ;;
            --project-dir)     PROJECT_DIR="$2"; shift 2 ;;
            --non-interactive) NON_INTERACTIVE=1; shift ;;
            --no-cuda)         NO_CUDA=1; shift ;;
            --no-sysctl)       NO_SYSCTL=1; shift ;;
            --dry-run)         DRY_RUN=1; shift ;;
            --verbose)         VERBOSE=1; shift ;;
            --help|-h)         usage ;;
            *)                 warn "unknown option: $1"; shift ;;
        esac
    done
}

# ─── detection ────────────────────────────────────────────────────────────────
DETECTED_OS=""
DETECTED_OS_VERSION=""
DETECTED_ARCH=""
DETECTED_GPU=""
DETECTED_CUDA=""
DETECTED_PYTHON=""
DETECTED_PYTHON_VER=""
DETECTED_RAM_GB=0
DETECTED_DISK_GB=0

detect_os() {
    DETECTED_ARCH="$(uname -m)"
    local uname_s
    uname_s="$(uname -s)"

    if [[ "$uname_s" == "Darwin" ]]; then
        DETECTED_OS="macos"
        DETECTED_OS_VERSION="$(sw_vers -productVersion 2>/dev/null || echo "unknown")"
    elif [[ "$uname_s" == "Linux" ]]; then
        if grep -qi microsoft /proc/version 2>/dev/null; then
            DETECTED_OS="wsl"
        elif [[ -f /etc/NIXOS ]] || has_cmd nixos-version; then
            DETECTED_OS="nixos"
        else
            DETECTED_OS="ubuntu"
        fi
        DETECTED_OS_VERSION="$(. /etc/os-release 2>/dev/null && echo "${VERSION_ID:-unknown}" || echo "unknown")"
    else
        die "unsupported operating system: $uname_s"
    fi

    # RAM
    if [[ "$uname_s" == "Darwin" ]]; then
        DETECTED_RAM_GB=$(( $(sysctl -n hw.memsize 2>/dev/null || echo 0) / 1073741824 ))
    else
        DETECTED_RAM_GB=$(( $(grep MemTotal /proc/meminfo 2>/dev/null | awk '{print $2}' || echo 0) / 1048576 ))
    fi

    # Disk free
    if [[ "$uname_s" == "Darwin" ]]; then
        DETECTED_DISK_GB=$(df -g "${HOME}" 2>/dev/null | awk 'NR==2 {print $4}' || echo 0)
    else
        DETECTED_DISK_GB=$(df -BG "${HOME}" 2>/dev/null | awk 'NR==2 {gsub(/G/,"",$4); print $4}' || echo 0)
    fi
}

detect_gpu() {
    if [[ "$DETECTED_OS" == "macos" ]]; then
        if [[ "$DETECTED_ARCH" == "arm64" ]]; then
            DETECTED_GPU="apple-silicon"
        else
            DETECTED_GPU="none"
        fi
    elif has_cmd nvidia-smi; then
        DETECTED_GPU="nvidia"
        DETECTED_CUDA="$(nvidia-smi 2>/dev/null | grep -oP 'CUDA Version: \K[0-9.]+' || echo "")"
    else
        DETECTED_GPU="none"
    fi
}

detect_python() {
    local candidates=("python3.12" "python3.11" "python3.10" "python3")
    for cmd in "${candidates[@]}"; do
        if has_cmd "$cmd"; then
            local ver
            ver="$("$cmd" --version 2>&1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -1 || echo "")"
            if [[ -n "$ver" ]]; then
                local major minor
                major="$(echo "$ver" | cut -d. -f1)"
                minor="$(echo "$ver" | cut -d. -f2)"
                if [[ "$major" -eq 3 ]] && [[ "$minor" -ge 10 ]]; then
                    DETECTED_PYTHON="$(command -v "$cmd")"
                    DETECTED_PYTHON_VER="$ver"
                    return
                fi
            fi
        fi
    done
    DETECTED_PYTHON=""
    DETECTED_PYTHON_VER=""
}

print_sysinfo() {
    printf "\n"
    info "detecting system..."
    printf "\n"

    local os_display gpu_display
    case "$DETECTED_OS" in
        ubuntu) os_display="Ubuntu ${DETECTED_OS_VERSION} (${DETECTED_ARCH})" ;;
        macos)  os_display="macOS ${DETECTED_OS_VERSION} (${DETECTED_ARCH})" ;;
        nixos)  os_display="NixOS ${DETECTED_OS_VERSION} (${DETECTED_ARCH})" ;;
        wsl)    os_display="WSL2 / Ubuntu ${DETECTED_OS_VERSION} (${DETECTED_ARCH})" ;;
        *)      os_display="Unknown" ;;
    esac

    case "$DETECTED_GPU" in
        nvidia)
            local gpu_name
            gpu_name="$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 || echo "NVIDIA GPU")"
            gpu_display="${gpu_name} (CUDA ${DETECTED_CUDA})"
            ;;
        apple-silicon) gpu_display="Apple Silicon (Metal/MPS)" ;;
        none)          gpu_display="CPU only" ;;
    esac

    local python_display
    if [[ -n "$DETECTED_PYTHON_VER" ]]; then
        python_display="$DETECTED_PYTHON_VER"
    else
        python_display="${YELLOW}not found (uv will install 3.12)${RESET}"
    fi

    printf "  %sOS:%s       %s\n" "$DIM" "$RESET" "$os_display"
    printf "  %sPython:%s   %s\n" "$DIM" "$RESET" "$python_display"
    printf "  %sGPU:%s      %s\n" "$DIM" "$RESET" "$gpu_display"
    printf "  %sRAM:%s      %s GB\n" "$DIM" "$RESET" "$DETECTED_RAM_GB"
    printf "  %sDisk:%s     %s GB free\n" "$DIM" "$RESET" "$DETECTED_DISK_GB"
    printf "\n"
}

# ─── system dependencies ─────────────────────────────────────────────────────
install_system_deps() {
    info "installing system dependencies..."
    case "$DETECTED_OS" in
        ubuntu|wsl)
            run_cmd "sudo apt-get update -qq"
            run_cmd "sudo apt-get install -y -qq curl g++ portaudio19-dev git-lfs libturbojpeg python3-dev pre-commit"
            ;;
        macos)
            if ! has_cmd brew; then
                info "installing homebrew..."
                run_cmd '/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"'
            fi
            run_cmd "brew install gnu-sed gcc portaudio git-lfs libjpeg-turbo python pre-commit"
            ;;
        nixos)
            info "NixOS detected — system deps managed via nix develop"
            ;;
    esac
    ok "system dependencies ready"
}

# ─── uv installation ─────────────────────────────────────────────────────────
install_uv() {
    if has_cmd uv; then
        ok "uv already installed ($(uv --version 2>/dev/null || echo 'unknown'))"
        return
    fi
    info "installing uv..."
    run_cmd 'curl -LsSf https://astral.sh/uv/install.sh | sh'
    export PATH="$HOME/.local/bin:$PATH"
    # try sourcing shell profiles if uv not found yet
    if ! has_cmd uv; then
        for rc in "$HOME/.bashrc" "$HOME/.zshrc" "$HOME/.profile" "$HOME/.cargo/env"; do
            [[ -f "$rc" ]] && source "$rc" 2>/dev/null || true
        done
    fi
    has_cmd uv || die "uv installation failed — install manually: https://docs.astral.sh/uv/"
    ok "uv installed ($(uv --version 2>/dev/null || echo 'unknown'))"
}

# ─── install mode prompt ─────────────────────────────────────────────────────
prompt_install_mode() {
    if [[ -n "$INSTALL_MODE" ]]; then
        return
    fi
    local choice
    choice=$(prompt_choice \
        "how do you want to use DimOS?" \
        "1" \
        "Library    — pip install into your project (recommended for users)" \
        "Developer  — git clone + editable install (recommended for contributors)")

    case "$choice" in
        1) INSTALL_MODE="library" ;;
        2) INSTALL_MODE="dev" ;;
        *) INSTALL_MODE="library" ;;
    esac
}

# ─── extras selection ─────────────────────────────────────────────────────────
prompt_extras() {
    if [[ -n "$EXTRAS" ]]; then
        return
    fi

    if [[ "$INSTALL_MODE" == "dev" ]]; then
        EXTRAS="all"
        info "developer mode: installing all extras (except dds)"
        return
    fi

    # Platform selection
    local platforms
    platforms=$(prompt_multi \
        "which robot platforms will you use?" \
        "Unitree (Go2, G1, B1)" \
        "Drone (Mavlink / DJI)" \
        "Manipulators (xArm, Piper, OpenARMs)")

    # Feature selection
    local features
    features=$(prompt_multi \
        "which features do you need?" \
        "AI Agents (LangChain, voice control)" \
        "Perception (object detection, VLMs)" \
        "Visualization (Rerun 3D viewer)" \
        "Simulation (MuJoCo)" \
        "Web Interface (FastAPI dashboard)" \
        "Misc (extra ML models, vector embedding)")

    # Build extras list
    local -a extras_list=()

    # Platforms → extras
    if [[ "$platforms" == *"1"* ]]; then extras_list+=("unitree"); fi
    if [[ "$platforms" == *"2"* ]]; then extras_list+=("drone"); fi
    if [[ "$platforms" == *"3"* ]]; then extras_list+=("manipulation"); fi

    # Features → extras
    if [[ "$features" == *"1"* ]]; then extras_list+=("agents"); fi
    if [[ "$features" == *"2"* ]]; then extras_list+=("perception"); fi
    if [[ "$features" == *"3"* ]]; then extras_list+=("visualization"); fi
    if [[ "$features" == *"4"* ]]; then extras_list+=("sim"); fi
    if [[ "$features" == *"5"* ]]; then extras_list+=("web"); fi
    if [[ "$features" == *"6"* ]]; then extras_list+=("misc"); fi

    # GPU extras
    if [[ "$DETECTED_GPU" == "nvidia" ]] && [[ "$NO_CUDA" != "1" ]]; then
        if prompt_yn "  ${CYAN}▸${RESET} NVIDIA GPU detected — install CUDA support?" "y"; then
            extras_list+=("cuda")
        else
            extras_list+=("cpu")
        fi
    else
        extras_list+=("cpu")
    fi

    # Dev tools
    if prompt_yn "  ${CYAN}▸${RESET} include development tools (ruff, pytest, mypy)?" "n"; then
        extras_list+=("dev")
    fi

    # Ensure at least base
    if [[ ${#extras_list[@]} -eq 0 ]]; then
        extras_list=("base")
    fi

    EXTRAS="$(IFS=,; echo "${extras_list[*]}")"
    printf "\n"
    ok "selected extras: ${CYAN}${EXTRAS}${RESET}"
}

# ─── installation ─────────────────────────────────────────────────────────────
do_install_library() {
    local dir="${PROJECT_DIR:-$HOME/dimos-project}"
    info "library install → ${dir}"

    run_cmd "mkdir -p '$dir'"

    info "creating virtual environment (python 3.12)..."
    if [[ "$DRY_RUN" == "1" ]]; then
        dim "[dry-run] cd '$dir' && uv venv --python 3.12"
    else
        (cd "$dir" && uv venv --python 3.12)
    fi

    local pip_extras="$EXTRAS"
    info "installing dimos[${pip_extras}]..."
    if [[ "$DRY_RUN" == "1" ]]; then
        dim "[dry-run] cd '$dir' && source .venv/bin/activate && uv pip install 'dimos[${pip_extras}]'"
    else
        (
            cd "$dir"
            source .venv/bin/activate
            uv pip install "dimos[${pip_extras}]"
        )
    fi

    ok "dimos installed in ${dir}"
}

do_install_dev() {
    local dir="${PROJECT_DIR:-$HOME/dimos}"
    info "developer install → ${dir}"

    if [[ -d "$dir/.git" ]]; then
        info "existing clone found, pulling latest..."
        run_cmd "cd '$dir' && git pull --rebase origin $GIT_BRANCH"
    else
        info "cloning dimos (branch: ${GIT_BRANCH})..."
        run_cmd "GIT_LFS_SKIP_SMUDGE=1 git clone -b $GIT_BRANCH https://github.com/dimensionalOS/dimos.git '$dir'"
    fi

    info "syncing dependencies (all extras, excluding dds)..."
    if [[ "$DRY_RUN" == "1" ]]; then
        dim "[dry-run] cd '$dir' && uv sync --all-extras --no-extra dds"
    else
        (cd "$dir" && uv sync --all-extras --no-extra dds)
    fi

    ok "dimos developer environment ready in ${dir}"
}

do_install() {
    case "$INSTALL_MODE" in
        library) do_install_library ;;
        dev)     do_install_dev ;;
        *)       die "invalid install mode: $INSTALL_MODE" ;;
    esac
}

# ─── system configuration ────────────────────────────────────────────────────
configure_system() {
    if [[ "$NO_SYSCTL" == "1" ]]; then
        dim "  skipping sysctl configuration (--no-sysctl)"
        return
    fi
    if [[ "$DETECTED_OS" == "macos" ]]; then
        return
    fi
    if [[ "$DETECTED_OS" == "nixos" ]]; then
        info "NixOS: add networking.kernel.sysctl to configuration.nix for LCM buffers"
        dim "  networking.kernel.sysctl.\"net.core.rmem_max\" = 67108864;"
        dim "  networking.kernel.sysctl.\"net.core.rmem_default\" = 67108864;"
        return
    fi

    local current_rmem
    current_rmem="$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)"
    local target=67108864

    if [[ "$current_rmem" -ge "$target" ]]; then
        ok "LCM buffers already configured (rmem_max=${current_rmem})"
        return
    fi

    printf "\n"
    info "DimOS uses LCM transport which needs larger UDP buffers:"
    dim "  sudo sysctl -w net.core.rmem_max=67108864"
    dim "  sudo sysctl -w net.core.rmem_default=67108864"
    printf "\n"

    if prompt_yn "  apply sysctl changes?" "y"; then
        run_cmd "sudo sysctl -w net.core.rmem_max=67108864"
        run_cmd "sudo sysctl -w net.core.rmem_default=67108864"

        if prompt_yn "  persist across reboots (/etc/sysctl.d/99-dimos.conf)?" "y"; then
            if [[ "$DRY_RUN" != "1" ]]; then
                printf "# DimOS LCM transport buffers\nnet.core.rmem_max=67108864\nnet.core.rmem_default=67108864\n" \
                    | sudo tee /etc/sysctl.d/99-dimos.conf >/dev/null
            else
                dim "[dry-run] would write /etc/sysctl.d/99-dimos.conf"
            fi
        fi
        ok "LCM buffers configured"
    fi
}

# ─── verification ─────────────────────────────────────────────────────────────
verify_install() {
    info "verifying installation..."

    local install_dir
    if [[ "$INSTALL_MODE" == "library" ]]; then
        install_dir="${PROJECT_DIR:-$HOME/dimos-project}"
    else
        install_dir="${PROJECT_DIR:-$HOME/dimos}"
    fi

    if [[ "$DRY_RUN" == "1" ]]; then
        dim "[dry-run] would verify: python imports, CLI, GPU"
        ok "verification skipped (dry-run)"
        return
    fi

    local venv_python="${install_dir}/.venv/bin/python3"

    if [[ ! -f "$venv_python" ]]; then
        warn "venv python not found at ${venv_python}, skipping verification"
        return
    fi

    # Check python import
    if "$venv_python" -c "import dimos" 2>/dev/null; then
        ok "python import: dimos ✓"
    else
        warn "python import check failed — this may be expected for some configurations"
    fi

    # Check CLI
    local cli_path="${install_dir}/.venv/bin/dimos"
    if [[ -x "$cli_path" ]]; then
        ok "dimos CLI available"
    else
        dim "  dimos CLI not in PATH (activate venv first: source .venv/bin/activate)"
    fi

    # Check CUDA if applicable
    if [[ "$DETECTED_GPU" == "nvidia" ]] && [[ "$NO_CUDA" != "1" ]] && [[ "$EXTRAS" == *"cuda"* ]]; then
        if "$venv_python" -c "import torch; assert torch.cuda.is_available(); print(f'CUDA: {torch.cuda.get_device_name(0)}')" 2>/dev/null; then
            ok "CUDA available"
        else
            dim "  CUDA check: torch not available or no GPU in this environment"
        fi
    fi
    printf "\n"
}

# ─── quickstart ───────────────────────────────────────────────────────────────
print_quickstart() {
    local install_dir
    if [[ "$INSTALL_MODE" == "library" ]]; then
        install_dir="${PROJECT_DIR:-$HOME/dimos-project}"
    else
        install_dir="${PROJECT_DIR:-$HOME/dimos}"
    fi

    printf "\n"
    printf "  %s%s🎉 installation complete!%s\n\n" "$BOLD" "$GREEN" "$RESET"

    printf "  %sget started:%s\n\n" "$BOLD" "$RESET"
    printf "    %s# activate the environment%s\n" "$DIM" "$RESET"
    printf "    cd %s && source .venv/bin/activate\n\n" "$install_dir"

    if [[ "$EXTRAS" == *"unitree"* ]] || [[ "$EXTRAS" == "all" ]] || [[ "$EXTRAS" == *"base"* ]]; then
        printf "    %s# run unitree go2 (simulation)%s\n" "$DIM" "$RESET"
        printf "    dimos --simulation run unitree-go2\n\n"
        printf "    %s# connect to real hardware%s\n" "$DIM" "$RESET"
        printf "    ROBOT_IP=192.168.1.100 dimos run unitree-go2\n\n"
    fi

    if [[ "$EXTRAS" == *"sim"* ]] || [[ "$EXTRAS" == "all" ]] || [[ "$EXTRAS" == *"base"* ]]; then
        printf "    %s# MuJoCo simulation with click-nav%s\n" "$DIM" "$RESET"
        printf "    dimos --simulation run unitree-go2-click-nav --viewer-backend rerun\n\n"
    fi

    if [[ "$INSTALL_MODE" == "dev" ]]; then
        printf "    %s# run tests%s\n" "$DIM" "$RESET"
        printf "    uv run pytest dimos\n\n"
        printf "    %s# type checking%s\n" "$DIM" "$RESET"
        printf "    uv run mypy dimos\n\n"
    fi

    printf "  %sdocs:%s       https://github.com/dimensionalOS/dimos\n" "$DIM" "$RESET"
    printf "  %sdiscord:%s    https://discord.gg/dimos\n\n" "$DIM" "$RESET"
}

# ─── cleanup on error ────────────────────────────────────────────────────────
cleanup() {
    local exit_code=$?
    if [[ $exit_code -ne 0 ]]; then
        printf "\n"
        err "installation failed (exit code: ${exit_code})"
        err "for help: https://github.com/dimensionalOS/dimos/issues"
        err "or join discord: https://discord.gg/dimos"
    fi
}
trap cleanup EXIT

# ─── main ─────────────────────────────────────────────────────────────────────
main() {
    parse_args "$@"

    show_banner

    detect_os
    detect_gpu
    detect_python
    print_sysinfo

    # Pre-flight checks
    if [[ "$DETECTED_OS" == "ubuntu" ]] || [[ "$DETECTED_OS" == "wsl" ]]; then
        local ver_major
        ver_major="$(echo "$DETECTED_OS_VERSION" | cut -d. -f1)"
        if [[ "$ver_major" -lt 22 ]] 2>/dev/null; then
            warn "Ubuntu ${DETECTED_OS_VERSION} — 22.04 or newer is recommended"
        fi
    fi

    if [[ "$DETECTED_OS" == "macos" ]]; then
        local mac_major
        mac_major="$(echo "$DETECTED_OS_VERSION" | cut -d. -f1)"
        if [[ "$mac_major" -lt 12 ]] 2>/dev/null; then
            die "macOS ${DETECTED_OS_VERSION} is too old — 12.6+ required"
        fi
    fi

    install_system_deps
    install_uv

    # re-detect python after uv install
    if [[ -z "$DETECTED_PYTHON" ]]; then
        detect_python
        if [[ -z "$DETECTED_PYTHON" ]]; then
            info "python 3.12 will be installed by uv automatically"
        fi
    fi

    prompt_install_mode
    prompt_extras
    do_install
    configure_system
    verify_install
    print_quickstart
}

main "$@"
