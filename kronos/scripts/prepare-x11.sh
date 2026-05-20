#!/usr/bin/env bash
set -euo pipefail

if ! command -v xhost >/dev/null 2>&1; then
  echo "xhost is not installed; skipping X11 access setup."
  exit 0
fi

has_xhost_entry() {
  local expected="$1"
  xhost 2>/dev/null | grep -Fqx "$expected"
}

ensure_xhost_entry() {
  local rule="$1"
  local expected="$2"

  if has_xhost_entry "$expected"; then
    echo "xhost rule already present: $expected"
    return 0
  fi

  xhost +"$rule" >/dev/null || true
  echo "xhost rule added: $expected"
}

runtime="${CONTAINER_RUNTIME:-auto}"

if [[ "$runtime" == "auto" ]]; then
  if command -v docker >/dev/null 2>&1; then
    runtime="docker"
  elif command -v podman >/dev/null 2>&1; then
    runtime="podman"
  else
    runtime="unknown"
  fi
fi

case "$runtime" in
  docker)
    ensure_xhost_entry "local:docker" "LOCAL:docker"
    ;;
  podman)
    ensure_xhost_entry "si:localuser:$(id -un)" "SI:localuser:$(id -un)"
    ensure_xhost_entry "si:localuser:root" "SI:localuser:root"
    ensure_xhost_entry "local:podman" "LOCAL:podman"
    ;;
  *)
    ensure_xhost_entry "local:" "LOCAL:"
    ;;
esac

echo "X11 access prepared for runtime: $runtime"
