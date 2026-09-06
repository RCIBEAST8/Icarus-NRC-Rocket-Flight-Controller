#!/usr/bin/env python3
"""Time-weighted Cd analysis with a convergence-selected window and an
autocorrelation-aware uncertainty.  Reads a case read-only; writes nothing."""
import sys, glob, os
import numpy as np

def load(case):
    f = glob.glob(os.path.join(case,'postProcessing/forceCoeffs/**/forceCoeffs.dat'), recursive=True)
    if not f: return None
    cols=None; rows=[]
    for line in open(sorted(f)[0]):
        if line.startswith('#'):
            if line.strip().startswith('# Time'): cols=line.split()[1:]
            continue
        p=line.split()
        if len(p)>3: rows.append([float(x) for x in p])
    d=np.array(rows)
    ic = cols.index('Cd') if cols and 'Cd' in cols else 2
    return d[:,0], d[:,ic]

def tw_mean(t,y):
    """time-weighted (trapezoidal) mean - the correct average under adaptive dt"""
    if len(t)<2: return float(y.mean())
    return float(np.trapezoid(y,t)/(t[-1]-t[0]))

def drift_pct(t,y):
    """linear trend across the window, as % of the mean"""
    a=np.polyfit(t,y,1)[0]
    return 100*a*(t[-1]-t[0])/abs(tw_mean(t,y))

def neff(y):
    """effective sample count after autocorrelation"""
    y=y-y.mean(); n=len(y)
    if n<10 or y.std()==0: return max(n,1)
    c=np.correlate(y,y,'full')[n-1:]/ (y@y)
    s=0.0
    for k in range(1,min(n//4,2000)):
        if c[k]<0.05: break
        s+=c[k]
    return max(n/(1+2*s),1.0)

def report(name, case):
    r=load(case)
    if r is None: print("%-18s no forceCoeffs data"%name); return None
    t,cd=r
    tend=t[-1]
    # widest window whose residual drift stays under 0.05% of the mean
    best=None
    for frac in np.arange(0.10,0.91,0.02):
        t0=tend*(1-frac); m=t>=t0
        if m.sum()<50: continue
        if abs(drift_pct(t[m],cd[m]))<0.05: best=frac
    if best is None: best=0.10
    m=t>=tend*(1-best)
    tw,ar = tw_mean(t[m],cd[m]), float(cd[m].mean())
    ne=neff(cd[m]); se=float(cd[m].std())/np.sqrt(ne)
    print("%-18s  Cd = %.6f  +/- %.6f"%(name,tw,se))
    print("%-18s    window  last %.0f%% of run  (t %.4f..%.4f, %d samples)"%("",best*100,t[m][0],t[m][-1],m.sum()))
    print("%-18s    drift %+.4f%%   ripple %.4f%%   N_eff %.0f"%("",drift_pct(t[m],cd[m]),100*cd[m].std()/abs(tw),ne))
    print("%-18s    time-weighted %.6f  vs  plain sample mean %.6f   (diff %+.4f%%)"%("",tw,ar,100*(tw-ar)/abs(tw)))
    for f2 in (0.50,0.25,0.10):
        mm=t>=tend*(1-f2)
        print("%-18s    window sensitivity  last %2.0f%%: %.6f"%("",f2*100,tw_mean(t[mm],cd[mm])))
    return tw

if __name__=="__main__":
    RUN=os.path.dirname(os.path.abspath(__file__))
    for n in sys.argv[1:] or ["NRCstatic","NRCstaticMin"]:
        report(n, os.path.join(RUN,n)); print()
