#!/usr/bin/env bash
set -e

source venv/bin/activate
python -m pip uninstall notros2 -y
rm ~/.notros2/config.json || true
rm -r test_package* || true
cd ..
python -m pip install ./notros2
cd notros2
