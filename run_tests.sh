#!/bin/bash
SCRIPT_DIR=$(dirname "$0")
python -m unittest discover -s $SCRIPT_DIR/test