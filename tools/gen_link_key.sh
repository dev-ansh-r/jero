#!/usr/bin/env bash
# Write a random 32-byte hex key for the Jero link. Same file goes on the robot and the brain.
#   tools/gen_link_key.sh ~/.config/jero/link.key
set -euo pipefail
out="${1:?usage: gen_link_key.sh <output-file>}"
[ -e "$out" ] && { echo "refusing to overwrite $out" >&2; exit 1; }
mkdir -p "$(dirname "$out")"
umask 077
python3 -c 'import secrets; print(secrets.token_hex(32))' > "$out"
echo "wrote $out"
