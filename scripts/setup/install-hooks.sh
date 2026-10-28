#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."

sudo apt install -y pre-commit
pre-commit install
