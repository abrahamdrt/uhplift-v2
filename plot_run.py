#!/usr/bin/env python3
"""
UHplift — Post-Run Plot Generator (Final)
Reads CSV from ./logs/ and generates 5 PNGs.

Usage:
    python3 plot_run.py                     # Latest log
    python3 plot_run.py logs/run_*.csv      # Specific file
"""
import sys, os, glob, csv
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

def find_csv():
    for d in [os.path.join(os.getcwd(),"logs"),
              os.path.expanduser("~/crane_logs"),
              os.path.join(os.getcwd(),"crane_logs")]:
        f = glob.glob(os.path.join(d, "run_*.csv"))
        if f: return max(f, key=os.path.getmtime)
    print("[ERROR] No CSV found"); sys.exit(1)

def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows: print("[ERROR] Empty CSV"); sys.exit(1)
    d = {}
    for k in rows[0]:
        if k == "mode": d[k] = [r[k] for r in rows]
        else: d[k] = np.array([float(r[k]) for r in rows])
    return d

def shade(ax, d):
    t = d["t_s"]; m = d["mode"]
    c = {"DISABLED":("#9E9E9E",0.08),"MANUAL":("#2196F3",0.08),
         "AUTO":("#4CAF50",0.10),"FAULT":("#F44336",0.12)}
    if len(t)<2: return
    i0=0; cur=m[0]
    for i in range(1,len(m)):
        if m[i]!=cur or i==len(m)-1:
            col,a = c.get(cur,("#9E9E9E",0.05))
            ax.axvspan(float(t[i0]),float(t[min(i,len(t)-1)]),alpha=a,color=col,zorder=0)
            i0=i; cur=m[i]

def plot_velocity(d, base, axis):
    """Plot velocity tracking for bridge or trolley."""
    t = d["t_s"]
    sfx = "bridge" if axis=="bridge" else "trolley"
    lab = "Bridge (X)" if axis=="bridge" else "Trolley (Z)"
    fig,ax = plt.subplots(figsize=(10,4))
    ax.plot(t, d[f"v_ref_{sfx}"], label="v_ref", color="#2196F3", lw=1.5)
    ax.plot(t, d[f"v_cmd_{sfx}"], label="v_cmd", color="#FF9800", lw=1.2, ls="--")
    ax.plot(t, d[f"v_{sfx}"], label=f"v_{sfx} (enc)", color="#4CAF50", lw=1, alpha=0.8)
    shade(ax,d); ax.set_xlabel("Time [s]"); ax.set_ylabel("Velocity [in/s]")
    ax.set_title(f"{lab} Velocity Tracking"); ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(f"{base}_vel_{sfx}.png", dpi=150); plt.close(fig)
    print(f"  [SAVED] {base}_vel_{sfx}.png")

def plot_sway(d, base):
    t = d["t_s"]
    fig,(a1,a2) = plt.subplots(2,1,figsize=(10,6),sharex=True)
    # Bridge sway (theta_x)
    if "theta_x_raw_deg" in d:
        a1.plot(t, d["theta_x_raw_deg"], color="#E91E63", lw=0.8, label="θx (bridge)")
    if "theta_z_raw_deg" in d:
        a1.plot(t, d["theta_z_raw_deg"], color="#2196F3", lw=0.8, label="θz (trolley)")
    a1.axhline(2,color="#999",ls="--",lw=0.8); a1.axhline(-2,color="#999",ls="--",lw=0.8)
    a1.fill_between(t,-2,2,alpha=0.06,color="#4CAF50")
    shade(a1,d); a1.set_ylabel("Sway [deg]"); a1.set_title("Payload Sway")
    a1.legend(fontsize=8); a1.grid(alpha=0.3)
    # Sway rates
    if "theta_x_dot" in d:
        a2.plot(t, d["theta_x_dot"], color="#E91E63", lw=0.8, label="θx_dot")
    if "theta_z_dot" in d:
        a2.plot(t, d["theta_z_dot"], color="#2196F3", lw=0.8, label="θz_dot")
    shade(a2,d); a2.set_xlabel("Time [s]"); a2.set_ylabel("Rate [rad/s]")
    a2.legend(fontsize=8); a2.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(f"{base}_sway.png", dpi=150); plt.close(fig)
    print(f"  [SAVED] {base}_sway.png")

def plot_force(d, base):
    t = d["t_s"]
    fig,(a1,a2) = plt.subplots(2,1,figsize=(10,6),sharex=True)
    a1.plot(t, d["F_bridge"], color="#F44336", lw=1, label="F_bridge")
    a1.plot(t, d["F_trolley"], color="#2196F3", lw=1, label="F_trolley")
    shade(a1,d); a1.set_ylabel("Force [lbf]"); a1.set_title("LQR Force Output")
    a1.legend(fontsize=8); a1.grid(alpha=0.3)
    if "L_current" in d:
        a2.plot(t, d["L_current"], color="#9C27B0", lw=1)
        shade(a2,d); a2.set_ylabel("Cable L [in]")
    a2.set_xlabel("Time [s]"); a2.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(f"{base}_force.png", dpi=150); plt.close(fig)
    print(f"  [SAVED] {base}_force.png")

def plot_timing(d, base):
    lm = d["loop_ms"]
    fig,ax = plt.subplots(figsize=(8,4))
    ax.hist(lm, bins=np.linspace(0,min(float(np.max(lm))*1.2,20),60),
            color="#2196F3",alpha=0.7,edgecolor="white",lw=0.5)
    ax.axvline(5,color="#F44336",ls="--",lw=1.5,label="5ms deadline")
    mn=float(np.mean(lm)); p99=float(np.percentile(lm,99))
    mx=float(np.max(lm)); ov=int(np.sum(lm>5)); n=len(lm)
    s=f"mean:{mn:.2f}ms\np99:{p99:.2f}ms\nmax:{mx:.2f}ms\noverruns:{ov}/{n}"
    ax.text(0.97,0.95,s,transform=ax.transAxes,fontsize=8,va='top',ha='right',
            fontfamily='monospace',bbox=dict(boxstyle='round',fc='white',alpha=0.8))
    ax.set_xlabel("Loop [ms]"); ax.set_ylabel("Count"); ax.set_title("Timing")
    ax.legend(fontsize=8); ax.grid(alpha=0.3,axis='y')
    fig.tight_layout(); fig.savefig(f"{base}_timing.png", dpi=150); plt.close(fig)
    print(f"  [SAVED] {base}_timing.png")

def main():
    p = sys.argv[1] if len(sys.argv)>1 else find_csv()
    if not os.path.isfile(p): print(f"[ERROR] {p}"); sys.exit(1)
    print(f"[PLOT] {p}")
    d = load(p); n=len(d["t_s"]); dur=float(d["t_s"][-1]) if n else 0
    print(f"  {n} ticks, {dur:.1f}s")
    base = p.rsplit(".csv",1)[0]
    # Detect which axis has motion
    b_range = float(np.ptp(d.get("v_bridge", np.zeros(1))))
    t_range = float(np.ptp(d.get("v_trolley", np.zeros(1))))
    if b_range > 0.1: plot_velocity(d, base, "bridge")
    if t_range > 0.1: plot_velocity(d, base, "trolley")
    if b_range < 0.1 and t_range < 0.1:
        plot_velocity(d, base, "bridge")  # Default
    plot_sway(d, base)
    plot_force(d, base)
    plot_timing(d, base)
    print(f"\n[DONE] Plots saved alongside {os.path.basename(p)}")

if __name__ == "__main__":
    main()
