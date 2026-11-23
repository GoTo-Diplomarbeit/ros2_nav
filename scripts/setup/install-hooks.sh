#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."

sudo pacman -S pre-commit
pre-commit install
