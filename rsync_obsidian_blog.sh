#!/bin/bash

rsync -av --delete \
  --include='*/' \
  --include='*.md' \
  --exclude='*' \
  --prune-empty-dirs \
  "/Users/matthewpye/Documents/Obsidian_Vault/Create 3 Robot/" \
  "/Users/matthewpye/Documents/blog/mjpye.github.io/content/posts"
