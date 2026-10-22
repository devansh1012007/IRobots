#!/usr/bin/env bash
# Resumable model-weight fetcher.
#
# Model weights are large enough that a dropped connection is a real cost, so
# every download:
#   - uses `curl --continue-at -` to resume a partial file byte-for-byte
#   - is retried with exponential backoff
#   - lands in a .part file first, so a truncated download never looks complete
#
# The SHA-256 of every fetched file is written to weights/SHA256SUMS so that a
# results file can record exactly which weights produced it
# (PLAN:irobots-v2:C62).
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
WEIGHTS_DIR="${WEIGHTS_DIR:-$REPO_ROOT/weights}"

RETRIES="${RETRIES:-6}"
RETRY_BASE_SECONDS="${RETRY_BASE_SECONDS:-5}"
ULTRALYTICS_TAG="${ULTRALYTICS_TAG:-v8.4.0}"

log()  { printf '\033[36m[weights]\033[0m %s\n' "$*"; }
warn() { printf '\033[33m[weights]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[31m[weights]\033[0m %s\n' "$*" >&2; exit 1; }

retry() {
  local desc="$1"; shift
  local attempt=1
  while true; do
    if "$@"; then return 0; fi
    if (( attempt >= RETRIES )); then
      warn "FAILED after $attempt attempt(s): $desc"
      return 1
    fi
    local wait=$(( RETRY_BASE_SECONDS * attempt ))
    warn "attempt ${attempt}/${RETRIES} failed: ${desc}; resuming in ${wait}s"
    sleep "$wait"
    attempt=$(( attempt + 1 ))
  done
}

download() {
  local url="$1" dest="$2"
  mkdir -p "$(dirname "$dest")"
  if [[ -s "$dest" ]]; then
    log "already present: $(basename "$dest")"
    return 0
  fi
  log "downloading $(basename "$dest")"
  retry "GET $url" \
    curl -L --fail --continue-at - \
         --retry 5 --retry-delay 3 --retry-all-errors \
         --connect-timeout 20 --speed-time 60 --speed-limit 1024 \
         -o "${dest}.part" "$url" || die "could not fetch $url"
  mv -f "${dest}.part" "$dest"
}

mkdir -p "$WEIGHTS_DIR"

BASE="https://github.com/ultralytics/assets/releases/download/${ULTRALYTICS_TAG}"

# YOLO-World: the open-vocabulary detector. This is the component that
# supplies the CLASS LABEL the legacy pipeline never had.
download "${BASE}/yolov8s-world.pt" "$WEIGHTS_DIR/yolov8s-world.pt"

# SAM 2.1: optional mask refinement, off the critical path.
# Only fetched when explicitly requested, because it is ~900MB.
if [[ "${FETCH_SAM:-0}" == "1" ]]; then
  download "https://github.com/ultralytics/assets/releases/download/v8.3.0/sam2.1_l.pt" \
           "$WEIGHTS_DIR/sam2.1_l.pt"
fi

log "checksumming"
( cd "$WEIGHTS_DIR" && sha256sum ./*.pt 2>/dev/null > SHA256SUMS || true )
if [[ -s "$WEIGHTS_DIR/SHA256SUMS" ]]; then
  cat "$WEIGHTS_DIR/SHA256SUMS"
fi
log "weights ready in $WEIGHTS_DIR"
