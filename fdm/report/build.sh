#!/bin/sh
# Rebuild the report from the cached results (report/results/*.json; rerun report/mkresults.py to refresh them)
# and copy the PDF to the repository root.
set -e
cd "$(dirname "$0")"
/opt/homebrew/bin/python3 mktables.py
/opt/homebrew/bin/python3 figures.py
tectonic -X compile al3_report.tex
cp al3_report.pdf ../../Alouette-3-JSBSim-model-report.pdf
