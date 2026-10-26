#!/usr/bin/env bash
# Resilient dependency installer for the irobots project.
#
# Rules this script exists to enforce:
#   1. Every network operation is retried with exponential backoff.
#   2. Every download resumes from where it left off:
#        - uv/pip use a PERSISTENT cache directory, so an interrupted wheel
#          download is reused on the next attempt instead of starting over.
#        - raw HTTP downloads use `curl --continue-at -`, which resumes a
#          partial file byte-for-byte.
#   3. Installing is idempotent: running it twice is a no-op.
#   4. It registers the 5 in-tree workspace packages via an `irobots_workspace.pth`
#      file in `.venv`'s site-packages so `pytest` and `python -m irobots_eval.*`
#      work directly without manual `PYTHONPATH` exports.
#   5. It verifies the result by importing every third-party and workspace module,
#      so a "successful" install that produced a broken environment fails loudly.
#
# Usage:
#   tools/install_deps.sh              # install everything
#   tools/install_deps.sh --verify     # only verify
#   tools/install_deps.sh --reinstall  # force reinstall of the locked set
#   RETRIES=12 tools/install_deps.sh   # more retries
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
VENV="${VENV:-$REPO_ROOT/.venv}"
LOCKFILE="$SCRIPT_DIR/requirements.lock.txt"

# Persistent caches: this is what makes "resume where you left off" work.
export UV_CACHE_DIR="${UV_CACHE_DIR:-$HOME/.cache/uv}"
export PIP_CACHE_DIR="${PIP_CACHE_DIR:-$HOME/.cache/pip}"
export UV_HTTP_TIMEOUT="${UV_HTTP_TIMEOUT:-300}"
export UV_HTTP_RETRIES="${UV_HTTP_RETRIES:-10}"

RETRIES="${RETRIES:-6}"
RETRY_BASE_SECONDS="${RETRY_BASE_SECONDS:-5}"
TORCH_INDEX="${TORCH_INDEX:-https://download.pytorch.org/whl/cpu}"
TORCH_PIN="${TORCH_PIN:-torch==2.14.1}"
# torchvision must match torch AND come from the CPU index: the PyPI build is
# the CUDA one, and ultralytics will silently install it if it is missing,
# which breaks inference with "operator torchvision::nms does not exist".
TORCHVISION_PIN="${TORCHVISION_PIN:-torchvision==0.29.1}"

MODE="install"
for arg in "$@"; do
  case "$arg" in
    --verify)    MODE="verify" ;;
    --reinstall) MODE="reinstall" ;;
    *) echo "unknown argument: $arg" >&2; exit 2 ;;
  esac
done

log()  { printf '\033[36m[install]\033[0m %s\n' "$*"; }
warn() { printf '\033[33m[install]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[31m[install]\033[0m %s\n' "$*" >&2; exit 1; }

# retry <description> <command...>
# Runs the command until it succeeds or RETRIES attempts are exhausted.
retry() {
  local desc="$1"; shift
  local attempt=1
  while true; do
    if "$@"; then
      return 0
    fi
    if (( attempt >= RETRIES )); then
      warn "FAILED after $attempt attempt(s): $desc"
      return 1
    fi
    local wait=$(( RETRY_BASE_SECONDS * attempt ))
    warn "attempt ${attempt}/${RETRIES} failed: ${desc}"
    warn "  retrying in ${wait}s (partial downloads are resumed from cache)"
    sleep "$wait"
    attempt=$(( attempt + 1 ))
  done
}

# download <url> <destination>
# Resumable HTTP GET. The partial file lives at <destination>.part so a
# failure never leaves a truncated file that looks complete.
download() {
  local url="$1" dest="$2"
  mkdir -p "$(dirname "$dest")"
  if [[ -s "$dest" ]]; then
    log "already present, skipping: $(basename "$dest")"
    return 0
  fi
  retry "GET $url" \
    curl -L --fail --continue-at - \
         --retry 5 --retry-delay 3 --retry-all-errors \
         --connect-timeout 20 --speed-time 60 --speed-limit 1024 \
         -o "${dest}.part" "$url"
  mv -f "${dest}.part" "$dest"
}

install_workspace_pth() {
  log "registering workspace packages in $VENV (irobots_workspace.pth)"
  REPO_ROOT_ENV="$REPO_ROOT" "$VENV/bin/python" - <<'PY'
import os
import sysconfig
from pathlib import Path

repo_root = Path(os.environ["REPO_ROOT_ENV"]).resolve()
purelib = Path(sysconfig.get_paths()["purelib"])
purelib.mkdir(parents=True, exist_ok=True)
pth_file = purelib / "irobots_workspace.pth"
packages = [
    "irobots_core",
    "irobots_nodes",
    "irobots_msgs",
    "irobots_eval",
    "irobots_bringup",
]
lines = [str(repo_root / "ros2_ws" / "src" / pkg) for pkg in packages]
pth_file.write_text("\n".join(lines) + "\n", encoding="utf-8")
print(f"  wrote {pth_file}")
PY
}

verify() {
  install_workspace_pth
  log "verifying the environment..."
  "$VENV/bin/python" - <<'PY'
import importlib, sys
mods = [
    "numpy", "yaml", "scipy", "cv2", "PIL", "torch", "mujoco",
    "ultralytics", "matplotlib", "pytest",
    "irobots_core", "irobots_nodes", "irobots_msgs", "irobots_eval", "irobots_bringup",
]
bad = []
for m in mods:
    try:
        importlib.import_module(m)
        print(f"  OK   {m}")
    except Exception as exc:  # noqa: BLE001
        bad.append((m, exc))
        print(f"  FAIL {m}: {type(exc).__name__}: {exc}")
if bad:
    print(f"\n{len(bad)} module(s) failed to import", file=sys.stderr)
    sys.exit(1)
import torch
print(f"\n  torch {torch.__version__}  cuda_available={torch.cuda.is_available()}")
PY
  log "verification passed"
}

if [[ "$MODE" == "verify" ]]; then
  verify
  exit 0
fi

command -v uv >/dev/null 2>&1 || die "uv not found. Install it: curl -LsSf https://astral.sh/uv/install.sh | sh"

if [[ ! -x "$VENV/bin/python" ]]; then
  log "creating virtualenv at $VENV"
  retry "create venv" uv venv --python 3.12 "$VENV"
fi
UV=("$VENV/bin/python" -m pip)
if ! "$VENV/bin/python" -m pip --version >/dev/null 2>&1; then
  retry "bootstrap pip" uv pip install --python "$VENV/bin/python" pip
fi
UV=(uv pip install --python "$VENV/bin/python")

# --- torch FIRST, from the CPU-only index -----------------------------------
# Installing torch from PyPI pulls ~2.5GB of CUDA wheels on Linux. We are
# CPU-only, and disk is a real constraint on this box.
log "installing ${TORCH_PIN} from the CPU-only index (${TORCH_INDEX})"
if [[ "$MODE" == "reinstall" ]]; then
  retry "torch (cpu)" "${UV[@]}" --reinstall --index-url "$TORCH_INDEX" "$TORCH_PIN" "$TORCHVISION_PIN"
else
  # --reinstall-package torchvision: the PyPI build of the same version is the
  # CUDA one, and a resolver that sees the version already satisfied will not
  # replace it. The result is an import-time "operator torchvision::nms does
  # not exist" that looks like a broken ultralytics.
  retry "torch (cpu)" "${UV[@]}" --reinstall-package torchvision \
         --index-url "$TORCH_INDEX" "$TORCH_PIN" "$TORCHVISION_PIN"
fi

# --- the rest --------------------------------------------------------------
log "installing the locked dependency set"
if [[ "$MODE" == "reinstall" ]]; then
  retry "locked requirements" "${UV[@]}" --reinstall -r "$LOCKFILE"
else
  retry "locked requirements" "${UV[@]}" -r "$LOCKFILE"
fi

verify

# --- model weights ---------------------------------------------------------
# Fetched with a resumable curl so a dropped connection does not cost the
# 30-50MB re-download.
log "fetching model weights (resumable)"
SCRIPT_DIR_WEIGHTS="$SCRIPT_DIR/fetch_weights.sh"
if [[ -x "$SCRIPT_DIR_WEIGHTS" ]]; then
  retry "fetch weights" "$SCRIPT_DIR_WEIGHTS"
else
  warn "tools/fetch_weights.sh not found or not executable; skipping weights"
fi

log "done. Activate with: source $VENV/bin/activate"
