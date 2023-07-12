#!/usr/bin/env bash
source venv/bin/activate
python -m pip uninstall notros2 -y
cd ..
python -m pip install ./deadturtles
cd deadturtles