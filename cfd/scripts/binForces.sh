#!/bin/bash
source /opt/openfoam13/etc/bashrc
for C in Wing_pos_3_full Wing_pos_3; do
  D=/home/rcibeast/OpenFOAM/rcibeast-13/run/$C
  cd $D
  echo "=== $C ==="
  postProcess -func forceBins -fields '(U p nut nuTilda)' -latestTime > log.forceBins 2>&1
  grep -a "FATAL" log.forceBins | head -2
  find postProcessing/forceBins -type f 2>/dev/null | head -5
done
