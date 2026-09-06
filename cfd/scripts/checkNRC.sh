#!/bin/bash
# Status + process hygiene for the NRC refined runs.  Arg = sleep seconds.
RUN=/home/rcibeast/OpenFOAM/rcibeast-13/run
sleep "${1:-1800}"
date '+%Y-%m-%d %H:%M:%S'

RUNNER=$(pgrep -f runNRCnew.sh | head -1)
ACTIVE=""
for C in NRCstatic_new NRCstaticMin_new; do
  [ -d "$RUN/$C" ] || continue
  if [ -f "$RUN/$C/log.foamRun" ] || [ -f "$RUN/$C/log.foamRun.resume" ]; then ACTIVE=$C; fi
done

echo "runner pid: ${RUNNER:-NONE}   current case: ${ACTIVE:-none}"
tail -3 "$RUN/nrcnew.log"

if [ -n "$ACTIVE" ]; then
  cd "$RUN/$ACTIVE"
  python3 -c "
import numpy as np,glob,os
try:
    T=[];E=[]
    import os
    LOG='log.foamRun.resume' if os.path.exists('log.foamRun.resume') else 'log.foamRun'
    for line in open(LOG,errors='ignore'):
        if line.startswith('Time = '): T.append(float(line.split('=')[1].strip().rstrip('s')))
        elif line.startswith('ExecutionTime'): E.append(float(line.split('=')[1].split('s')[0]))
    n=min(len(T),len(E))
    if n>20:
        k=min(200,n-1); dT=T[n-1]-T[n-1-k]; dE=E[n-1]-E[n-1-k]
        print('  sim %.5f/0.05 (%.1f%%)  dt %.2e  %.2f s/step  ETA %.1f h'%(T[n-1],100*T[n-1]/0.05,dT/k,dE/k,(0.05-T[n-1])/(dT/k)*(dE/k)/3600))
    else: print('  solver just started (%d steps)'%n)
except Exception as e: print('  (no solver progress yet)')
f=glob.glob('postProcessing/forceCoeffs/**/forceCoeffs.dat',recursive=True)
if f:
    d=np.vstack([np.loadtxt(x,comments='#') for x in sorted(f)]); d=d[np.argsort(d[:,0])]
    if d.ndim==2 and len(d)>200:
        w=d[d[:,0]>=0.9*d[-1,0]]
        print('  Cd %.5f (sd %.5f) over last 10%% of history so far'%(w[:,2].mean(),w[:,2].std()))
"
  grep -a "Courant Number" log.foamRun.resume log.foamRun 2>/dev/null | tail -1 | sed 's/^/  /'
fi

echo "  disk: $(du -sBG "$RUN" | cut -f1)  (cap 130G of the 140G budget)"

echo "-- openfoam processes --"
ps -eo pid,ppid,etimes,comm > /tmp/psnrc.txt
python3 - <<'PY'
FOAM={'foamRun','snappyHexMesh','blockMesh','checkMesh','decomposePar','reconstructPar','surfaceFeatures','postProcess','mpirun','orterun'}
rows=[]
for line in open('/tmp/psnrc.txt').read().splitlines()[1:]:
    p=line.split()
    if len(p)<4: continue
    pid,ppid,et,comm=p[0],p[1],p[2],p[3]
    if comm in FOAM: rows.append((int(pid),int(ppid),int(et),comm))
if not rows:
    print("  none running")
else:
    orph=[]
    for pid,ppid,et,comm in rows:
        tag=""
        if ppid==1:
            tag="  <== ORPHAN"; orph.append(pid)
        print("  %-16s pid %-7d ppid %-7d %5ds%s"%(comm,pid,ppid,et,tag))
    open('/tmp/orphans.txt','w').write("\n".join(str(o) for o in orph))
PY
if [ -s /tmp/orphans.txt ]; then
  echo "  killing orphaned OpenFOAM processes:"
  while read -r o; do [ -n "$o" ] && kill "$o" 2>/dev/null && echo "    killed $o"; done < /tmp/orphans.txt
  rm -f /tmp/orphans.txt
fi
