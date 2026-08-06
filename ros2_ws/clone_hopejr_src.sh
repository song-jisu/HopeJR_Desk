#!/usr/bin/env bash
# Clone/update the HopeJR workspace packages into ros2_ws/src.
#
# By default it pulls the FIVE repos this workspace uses. With --dynamic it asks
# GitHub for song-jisu's 5 most-recently-updated repos instead (handy if the set
# changes, but it will refuse anything not named hopejr_* so a stray repo can't
# sneak in).
#
#   ./clone_hopejr_src.sh            # the 5 known packages (recommended)
#   ./clone_hopejr_src.sh --dynamic  # song-jisu's 5 latest hopejr_* repos
#
# Existing package dirs are handled safely:
#   - a git checkout      -> git pull --ff-only (kept, just updated)
#   - a non-git directory -> moved to <name>.bak_YYYYmmdd_HHMMSS, then cloned
#
# WARNING: uncommitted local edits in a NON-git package dir are preserved only in
# the .bak copy — the live dir becomes a fresh clone. Commit/push first if you
# want your changes upstream.
set -euo pipefail

USER=song-jisu
SRC="$(cd "$(dirname "$0")" && pwd)/src"
mkdir -p "$SRC"

REPOS=(
  hopejr_arm_description
  hopejr_hand_description
  hopejr_right_arm_description
  hopejr_state_publisher
  hopejr_real_bridge
)

if [[ "${1:-}" == "--dynamic" ]]; then
  echo "querying ${USER}'s 5 latest hopejr_* repos…"
  mapfile -t REPOS < <(
    curl -fsSL "https://api.github.com/users/${USER}/repos?sort=updated&per_page=30" \
      | grep -oE '"name": *"[^"]+"' | sed -E 's/.*"name": *"([^"]+)"/\1/' \
      | grep -E '^hopejr_' | head -5
  )
  [[ ${#REPOS[@]} -gt 0 ]] || { echo "no hopejr_* repos found"; exit 1; }
  printf '  -> %s\n' "${REPOS[@]}"
fi

for r in "${REPOS[@]}"; do
  dst="$SRC/$r"
  url="https://github.com/${USER}/${r}.git"
  if [[ -d "$dst/.git" ]]; then
    echo "[pull ] $r"
    git -C "$dst" pull --ff-only
  else
    if [[ -e "$dst" ]]; then
      bak="${dst}.bak_$(date +%Y%m%d_%H%M%S)"
      echo "[backup] $r -> $(basename "$bak")"
      mv "$dst" "$bak"
    fi
    echo "[clone] $r"
    git clone --depth 1 "$url" "$dst"
  fi
done

echo "done. packages in $SRC:"
ls -1 "$SRC" | grep -E '^hopejr_' | grep -v '\.bak_'
