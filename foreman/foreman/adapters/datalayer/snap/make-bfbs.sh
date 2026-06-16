#!/usr/bin/env bash
set -e

# execute from anywhere in foreman/ repo

FOREMAN_REPO_ROOT="$(git rev-parse --show-toplevel)"

# TODO: make this foreman build for arm as well
# TODO: automate this in setup.py
# assuming you have rt_sdk/ or ctrlx_sdk/ in src/ dir of your workspace, target their fixed version of `flatc`
FLATC_BINARY="$FOREMAN_REPO_ROOT/../rt_sdk/public/bin/oss.flatbuffers/ubuntu24-gcc-x64/flatc"
FOREMAN_SCHEMA="$FOREMAN_REPO_ROOT/foreman/foreman/adapters/datalayer/flatbuffer_foreman.fbs"
OUT_BFBS_DIR="$FOREMAN_REPO_ROOT/foreman/foreman/adapters/datalayer/"
OUT_FBS_PYTHON_MODULE="$FOREMAN_REPO_ROOT/foreman/"

[ -f "$FLATC_BINARY" ] || { echo "Error: flatc missing"; exit 1; }
[ -f "$FOREMAN_SCHEMA" ] || { echo "Error: schema missing"; exit 1; }

# https://github.com/boschrexroth/ctrlx-automation-sdk/blob/main/samples-python/datalayer.provider/make-bfbs.sh
"$FLATC_BINARY" -o "$OUT_BFBS_DIR/" --schema --binary --bfbs-comments --bfbs-builtins --no-warnings "$FOREMAN_SCHEMA"
"$FLATC_BINARY" -o "$OUT_FBS_PYTHON_MODULE/" --python --gen-object-api --gen-mutable --no-warnings "$FOREMAN_SCHEMA"
