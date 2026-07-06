#!/bin/bash

set -euo pipefail

DEST="/Users/matthewpye/Documents/blog/mjpye.github.io/content/posts"
CREATE3="/Users/matthewpye/Documents/Obsidian_Vault/Create 3 Robot/"
SO101="/Users/matthewpye/Documents/Obsidian_Vault/SO-101/"

STAGING=$(mktemp -d)
trap 'rm -rf "$STAGING"' EXIT

RSYNC_MD_OPTS=(
  -av
  --delete
  --include='*/'
  --include='*.md'
  --exclude='*'
  --prune-empty-dirs
)

# Sync each vault into its own staging dir (--delete is safe here)
rsync "${RSYNC_MD_OPTS[@]}" "$CREATE3" "$STAGING/create3/"
rsync "${RSYNC_MD_OPTS[@]}" --exclude='non-blog/' "$SO101" "$STAGING/so101/"

# Merge both staging dirs, then sync to posts with --delete
mkdir -p "$STAGING/combined"
rsync -a "$STAGING/create3/" "$STAGING/combined/"
rsync -a "$STAGING/so101/" "$STAGING/combined/"

rsync -av --delete "$STAGING/combined/" "$DEST"
