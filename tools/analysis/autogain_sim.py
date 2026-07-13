#!/usr/bin/env python3
"""
Autogain validation gate-1 — host Monte-Carlo simulation (no device).

Settles the two Critical findings of the adversarial review
(docs/superpowers/specs/2026-07-12-autogain-review.md) against the v1
knee-finder design (docs/superpowers/specs/2026-07-12-autogain-redesign-proposal.md):

  Critical 1 (count starvation): at this install's REAL satellite-pass rate,
    per-arm decode counts are single-digit, the Poisson band 2*sqrt(Dmax+1)
    swallows the plateau floor, and the v1 picker degenerates to "always pick
    the lowest-gain arm" -> systematic downward gain drift into deafness.

  Critical 2 (wrong signal): the sweep parks on the strong IRA LO (1626.2 MHz)
    whose decode "knee" sits at LOWER gain than the weak-ACARS knee we actually
    care about. So even a "correct" IRA pick is too low for the mission signal.

The sim MEASURES on the IRA knee curve (what the sweep sees) and EVALUATES the
converged gain against the ACARS knee curve (what matters) -- exactly the
measure-IRA / evaluate-ACARS split the design bakes in (proposal Sec.2 reference
LO; review Critical 2). It then contrasts:

  * v1 picker            -- proposal Sec.6 exactly (THETA=0.75, K=2, MIN_COUNTS=12,
                            DEADBAND_STEPS=2, MAX_STEPS_PER_RUN=4, offsets {-6,-4,-2,0,+2})
  * coarse 3-state       -- review's prescription: deaf/good/saturating with a wide
                            guard band, pick a FIXED MARGIN (+2 steps) ABOVE the lowest
                            "good" edge -- never sit on the knee.

Everything is parameterized (see Params below). numpy only, standalone.

Usage:  python3 tools/analysis/autogain_sim.py [--sweeps 1000] [--chains 400]
                                               [--seed 12345] [--json]

NOTE: this is an analysis/design-validation tool, not a repo unit test; it does
not touch any device/serial. Cite lines: proposal Sec.0/1.2/2/3/4/5/6; review
Critical 1/2, Important 3.
"""
from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass, field

import numpy as np

# --------------------------------------------------------------------------
# R828D discrete gain table (tenths of a dB), copied from
# p4-usb-host/main/autotune_gainset.h : AUTOTUNE_R828D_GAINS[].  29 steps.
# --------------------------------------------------------------------------
R828D_DBX10 = np.array(
    [0, 9, 14, 27, 37, 77, 87, 125, 144, 157,
     166, 197, 207, 229, 254, 280, 297, 328, 338, 364,
     372, 386, 402, 421, 434, 439, 445, 480, 496],
    dtype=int,
)
GAIN_DB = R828D_DBX10 / 10.0            # dB
N_STEPS = len(R828D_DBX10)             # 29
IDX_43_4 = 24                          # table idx of 43.4 dB (proposal example incumbent)
IDX_33_8 = 18                          # table idx of 33.8 dB (low edge of ACARS plateau)


def idx_of_db(db: float) -> int:
    """Nearest table index for a gain in dB."""
    return int(np.argmin(np.abs(GAIN_DB - db)))


def clampi(x: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, x))


# ==========================================================================
# Params -- base case; every number cited to a doc line.
# ==========================================================================
@dataclass
class Params:
    # ---- gain arms (proposal Sec.3) -------------------------------------
    arm_offsets: tuple = (-6, -4, -2, 0, +2)   # R828D table-index offsets
    min_idx: int = 0                            # autotune_gain_min_dbx10 clip
    max_idx: int = N_STEPS - 1                  # autotune_gain_max_dbx10 clip
    start_inc_idx: int = IDX_43_4               # incumbent for cross-run drift = 43.4 dB

    # ---- schedule (proposal Sec.2 / Sec.5) ------------------------------
    n_arms: int = 5
    slice_s: float = 20.0                       # autotune_slice_s
    prime_s: float = 3.0                        # AUTOTUNE_PRIME_MS = 3000
    rounds: int = 8                             # autotune_rounds
    auto_extend: bool = True                    # +8 rounds once if Dmax<MIN_COUNTS

    # ---- pass process (review Critical 1 reseeded numbers) --------------
    pass_dur_s: float = 120.0                   # ~2 min satellite pass
    passes_per_sweep: float = 1.7               # a 15.3-min sweep sees ~1.7 passes
    decodes_per_pass: float = 13.0              # IRA decodes/pass at plateau gain
    # base decodes/pass is scaled by pass_scale for the sensitivity sweep (Part D)
    pass_scale: float = 1.0

    # ---- v1 picker constants (proposal Sec.6) ---------------------------
    theta_pct: int = 75                         # PLATEAU_THETA_PCT
    plateau_k: int = 2                          # PLATEAU_K
    min_counts: int = 12                        # MIN_COUNTS
    deadband_steps: int = 2                     # DEADBAND_STEPS
    max_steps_per_run: int = 4                  # MAX_STEPS_PER_RUN

    # ---- coarse 3-state picker (review Verdict prescription) ------------
    coarse_good_frac: float = 0.5               # good = D_i >= 0.5*Dmax
    coarse_margin_steps: int = 2                # pick +2 table steps ABOVE low good edge

    # ---- knee curves (relative decode probability, normalised to peak=1) -
    # IRA (what the sweep measures): 2026-07-08 data 0/2/6/0/1 at 8/15/25/35/44 dB
    #   -> rise at low end, peak ~mid, saturation roll-off at top.
    ira_klo: float = 13.0
    ira_wlo: float = 3.0
    ira_khi: float = 34.0
    ira_whi: float = 2.5
    # ACARS (what we care about): flat plateau ~33.8-43.4, knee just BELOW 33.8,
    #   i.e. ACARS knee is HIGHER-gain than IRA knee (review Critical 2).
    acars_klo: float = 31.0
    acars_wlo: float = 2.0
    acars_khi: float = 48.0
    acars_whi: float = 2.0


def _logistic(x, k, w):
    return 1.0 / (1.0 + np.exp(-(x - k) / w))


def knee_curve(gain_db, klo, wlo, khi, whi):
    """Rise (low knee) * fall (saturation roll-off); NOT yet normalised."""
    return _logistic(gain_db, klo, wlo) * (1.0 - _logistic(gain_db, khi, whi))


def make_curves(p: Params):
    """Return (p_ira, p_acars) over the whole table, each normalised to peak=1."""
    ira = knee_curve(GAIN_DB, p.ira_klo, p.ira_wlo, p.ira_khi, p.ira_whi)
    aca = knee_curve(GAIN_DB, p.acars_klo, p.acars_wlo, p.acars_khi, p.acars_whi)
    ira = ira / ira.max()
    aca = aca / aca.max()
    return ira, aca


# ==========================================================================
# Pass process + schedule
# ==========================================================================
def _slot_len(p: Params) -> float:
    return p.prime_s + p.slice_s                    # 23 s


def simulate_pass_decodes(p: Params, rng: np.random.Generator, max_rounds: int,
                          n_arms: int):
    """Generate decode event times over a max_rounds sweep timeline.

    Passes arrive as a Poisson process (mean `passes_per_sweep` over the base
    8-round wall time). Each pass emits Poisson(decodes_per_pass*pass_scale)
    events uniform over its 120 s. Returns event times (s) within [0, T_max].
    These are the *plateau-gain* candidate decodes; thinning by p_ira happens
    at admission time (per arm).
    """
    slot = _slot_len(p)
    T_base = p.rounds * n_arms * slot               # 8-round wall time
    T_max = max_rounds * n_arms * slot              # extended timeline
    # pass arrival rate so that E[passes over T_base] = passes_per_sweep
    lam = p.passes_per_sweep / T_base               # passes/s
    # generate arrivals over [-pass_dur, T_max] (passes can straddle sweep edges)
    lo, hi = -p.pass_dur_s, T_max
    expected = lam * (hi - lo)
    n_arr = rng.poisson(expected)
    starts = rng.uniform(lo, hi, size=n_arr)
    ev = []
    lam_pass = p.decodes_per_pass * p.pass_scale
    for s0 in starts:
        nd = rng.poisson(lam_pass)
        if nd:
            t = rng.uniform(s0, s0 + p.pass_dur_s, size=nd)
            ev.append(t)
    if not ev:
        return np.empty(0)
    t = np.concatenate(ev)
    return t[(t >= 0.0) & (t < T_max)]


def assign_to_arms(p: Params, event_t: np.ndarray, max_rounds: int, n_arms: int):
    """Map event times to (arm, round) via the rotated round-robin schedule.

    Slot j = round r*n_arms + k (0-based); arm = (r + k) % n_arms (rotation,
    proposal Sec.2). Events in the 3 s prime window are discarded. Returns
    (arm_of_event, round_of_event) for events that land in a slice; discards
    everything else.
    """
    slot = _slot_len(p)
    j = np.floor(event_t / slot).astype(int)
    valid = j < max_rounds * n_arms
    j = j[valid]
    t = event_t[valid]
    offset = t - slot * j
    in_slice = offset >= p.prime_s                  # drop prime-discard window
    j = j[in_slice]
    r = j // n_arms
    k = j % n_arms
    arm = (r + k) % n_arms
    return arm, r


def sweep_counts(p: Params, arm_gain_idx, rng: np.random.Generator,
                 curve, p1_only: bool = False):
    """Run one full sweep (with auto-extend) and return per-arm D_i counts.

    arm_gain_idx : list of table indices for the arms (len == n_arms after dedup)
    curve        : normalised p(gain) used to thin admissions (usually p_ira)
    p1_only      : if True, admit every decode with prob 1 (Part A pass-timing
                   test, no curve thinning).
    Returns (D, used_rounds).
    """
    n = len(arm_gain_idx)
    max_rounds = p.rounds * 2 if p.auto_extend else p.rounds
    ev = simulate_pass_decodes(p, rng, max_rounds, n)
    arm, rnd = assign_to_arms(p, ev, max_rounds, n)

    # admission thinning per event by the curve at that arm's gain
    if p1_only:
        keep = np.ones(len(arm), dtype=bool)
    else:
        probs = curve[np.asarray(arm_gain_idx)][arm]
        keep = rng.random(len(arm)) < probs
    arm_k = arm[keep]
    rnd_k = rnd[keep]

    # 8-round counts
    D8 = np.zeros(n, dtype=int)
    m8 = rnd_k < p.rounds
    for a in arm_k[m8]:
        D8[a] += 1
    if not p.auto_extend or D8.max() >= p.min_counts:
        return D8, p.rounds
    # auto-extend: use full 16-round counts (same pass realisation)
    D16 = np.zeros(n, dtype=int)
    for a in arm_k:
        D16[a] += 1
    return D16, max_rounds


# ==========================================================================
# Arm construction (proposal Sec.3)
# ==========================================================================
def build_arms(p: Params, inc_idx: int):
    """Incumbent-centred offsets, clipped to [min_idx,max_idx], deduped, ascending.
    Returns list of table indices."""
    idxs = []
    for off in p.arm_offsets:
        i = clampi(inc_idx + off, p.min_idx, p.max_idx)
        if i not in idxs:
            idxs.append(i)
    idxs.sort()
    return idxs


# ==========================================================================
# Pickers
# ==========================================================================
def _isqrt(n: int) -> int:
    return int(math.isqrt(int(n)))


def on_plateau(Di: int, Dmax: int, p: Params) -> bool:
    # proposal Sec.6 integer form:
    #   D[i]*100 >= THETA_PCT*Dmax - 100*K*isqrt(Dmax+1)
    return Di * 100 >= p.theta_pct * Dmax - 100 * p.plateau_k * _isqrt(Dmax + 1)


def pick_v1(p: Params, D, arm_idx, inc_idx):
    """v1 knee picker + hysteresis, proposal Sec.6/Sec.4. Returns (new_inc_idx, moved, chosen_arm_idx, have_signal)."""
    D = np.asarray(D, dtype=int)
    Dmax = int(D.max()) if len(D) else 0
    if Dmax < p.min_counts:
        return inc_idx, False, None, False       # no signal: keep incumbent (Sec.4 rule 1)
    plat = [i for i in range(len(D)) if on_plateau(int(D[i]), Dmax, p)]
    if not plat:
        return inc_idx, False, None, True
    lo = min(plat, key=lambda i: arm_idx[i])     # lowest-gain plateau member
    cand_idx = arm_idx[lo]
    inc_on_plateau = any(arm_idx[i] == inc_idx for i in plat)
    step_delta = inc_idx - cand_idx              # >0 = moving down
    if cand_idx == inc_idx:
        chosen = inc_idx
    elif inc_on_plateau and abs(step_delta) < p.deadband_steps:
        chosen = inc_idx                         # tie inside deadband: stay (Sec.4 rule 3)
    else:
        chosen = clampi(cand_idx, inc_idx - p.max_steps_per_run,
                        inc_idx + p.max_steps_per_run)   # clamp walk (Sec.4 rule 5)
    return chosen, (chosen != inc_idx), cand_idx, True


def pick_coarse(p: Params, D, arm_idx, inc_idx):
    """Coarse 3-state picker (review prescription). Classify deaf/good/saturating
    with a wide guard band; pick a FIXED MARGIN above the lowest 'good' edge; apply
    the same deadband + clamp hysteresis. Returns (new_inc_idx, moved, target_idx, have_signal)."""
    D = np.asarray(D, dtype=int)
    Dmax = int(D.max()) if len(D) else 0
    if Dmax < p.min_counts:
        return inc_idx, False, None, False       # no signal: keep incumbent

    # good = D_i >= good_frac*Dmax; saturating = a higher-gain arm whose D collapsed
    # relative to a lower-gain neighbour (compression telltale) -> excluded from good.
    good = []
    for i in range(len(D)):
        if D[i] < p.coarse_good_frac * Dmax:
            continue                             # deaf (well below the good band)
        # saturating telltale: any strictly-lower-gain arm has >=2x this arm's D
        sat = any(arm_idx[j] < arm_idx[i] and D[j] >= 2 * max(D[i], 1)
                  for j in range(len(D)))
        if sat:
            continue
        good.append(i)
    if not good:
        return inc_idx, False, None, True

    low_edge = min(good, key=lambda i: arm_idx[i])   # lowest-gain good arm
    target = clampi(arm_idx[low_edge] + p.coarse_margin_steps, p.min_idx, p.max_idx)
    step = target - inc_idx
    if abs(step) < p.deadband_steps:
        chosen = inc_idx                         # deadband: stay
    else:
        chosen = clampi(target, inc_idx - p.max_steps_per_run,
                        inc_idx + p.max_steps_per_run)   # clamp walk
    return chosen, (chosen != inc_idx), target, True


# ==========================================================================
# Part A -- D_max distribution (pass-timing test; review 6-8 vs design 16)
# ==========================================================================
def part_A(p: Params, rng, n_sweeps: int):
    """Two variants:
      (i)  p1_only=True, arms centred at IRA plateau -> pure pass-timing D_max
           (tests review's ~6-8 vs design's ~16 with NO curve thinning).
      (ii) realistic first sweep: incumbent 43.4, arms 33.8-44.5, thinned by p_ira.
    """
    ira, _ = make_curves(p)
    # (i) all arms at IRA plateau, p=1, base 8-round sweep (NO auto-extend) -->
    #     the pure pass-timing D_max that tests review's ~6-8 vs design's ~16.
    p_noext = Params(auto_extend=False, pass_scale=p.pass_scale)
    ira_peak_idx = int(np.argmax(ira))
    arms_plateau = build_arms(p_noext, ira_peak_idx)
    dmax_p1 = []
    dmax_p1_ext = []
    for _ in range(n_sweeps):
        D, _r = sweep_counts(p_noext, arms_plateau, rng, ira, p1_only=True)
        dmax_p1.append(int(D.max()))
        De, _re = sweep_counts(p, arms_plateau, rng, ira, p1_only=True)  # with extend
        dmax_p1_ext.append(int(De.max()))
    # (ii) realistic first sweep (incumbent 43.4)
    arms_first = build_arms(p, p.start_inc_idx)
    dmax_real = []
    total_real = []
    for _ in range(n_sweeps):
        D, _r = sweep_counts(p, arms_first, rng, ira, p1_only=False)
        dmax_real.append(int(D.max()))
        total_real.append(int(D.sum()))
    return {
        "p1_dmax": np.array(dmax_p1),
        "p1_dmax_ext": np.array(dmax_p1_ext),
        "real_dmax": np.array(dmax_real),
        "real_total": np.array(total_real),
        "arms_plateau_db": [GAIN_DB[i] for i in arms_plateau],
        "arms_first_db": [GAIN_DB[i] for i in arms_first],
    }


# ==========================================================================
# Part B/C/D -- cross-run drift
# ==========================================================================
def run_drift(p: Params, rng, picker, n_sweeps_chain: int, n_chains: int, curve):
    """Run n_chains independent chains of n_sweeps_chain consecutive sweeps,
    starting at start_inc_idx, applying the picker's hysteresis each sweep.
    Returns dict with final indices, first-sweep chosen arm offsets, trajectories."""
    finals = []
    first_chosen_off = []      # chosen arm offset (relative to incumbent) on sweep 1
    last5_spread = []          # table-step spread over last 5 sweeps
    trajectories = []
    no_signal_final = []
    for c in range(n_chains):
        inc = p.start_inc_idx
        traj = [inc]
        for s in range(n_sweeps_chain):
            arms = build_arms(p, inc)
            D, _r = sweep_counts(p, arms, rng, curve, p1_only=False)
            new_inc, moved, cand, have = picker(p, D, arms, inc)
            if s == 0 and cand is not None:
                first_chosen_off.append(cand - inc)
            inc = new_inc
            traj.append(inc)
        finals.append(inc)
        last5 = traj[-5:]
        last5_spread.append(max(last5) - min(last5))
        trajectories.append(traj)
        # was the last sweep a no-signal freeze?
        arms = build_arms(p, inc)
        D, _r = sweep_counts(p, arms, rng, curve, p1_only=False)
        no_signal_final.append(int(D.max()) < p.min_counts)
    return {
        "finals": np.array(finals),
        "first_chosen_off": np.array(first_chosen_off) if first_chosen_off else np.array([]),
        "last5_spread": np.array(last5_spread),
        "trajectories": trajectories,
        "no_signal_frac": float(np.mean(no_signal_final)),
    }


def acars_yield(p: Params, gain_idx: int):
    """ACARS yield fraction at gain_idx relative to the true ACARS plateau (argmax)."""
    _, aca = make_curves(p)
    true_idx = int(np.argmax(aca))
    return aca[gain_idx] / aca[true_idx], true_idx


# ==========================================================================
# Reporting helpers
# ==========================================================================
def stats(a: np.ndarray):
    a = np.asarray(a, dtype=float)
    return dict(mean=float(a.mean()), median=float(np.median(a)),
                p10=float(np.percentile(a, 10)), p90=float(np.percentile(a, 90)),
                mn=float(a.min()), mx=float(a.max()))


def fmt_stats(s):
    return (f"mean={s['mean']:.2f} median={s['median']:.1f} "
            f"p10-p90=[{s['p10']:.1f},{s['p90']:.1f}] range=[{s['mn']:.0f},{s['mx']:.0f}]")


def finals_summary(p: Params, finals: np.ndarray):
    """Summarise a distribution of final table indices -> gains + ACARS yield."""
    idxs, cnt = np.unique(finals, return_counts=True)
    med_idx = int(np.median(finals))
    yld, true_idx = acars_yield(p, med_idx)
    modes = sorted(zip(idxs.tolist(), cnt.tolist()), key=lambda x: -x[1])[:4]
    mode_str = ", ".join(f"{GAIN_DB[i]:.1f}dB(idx{i}):{c}" for i, c in modes)
    return {
        "median_idx": med_idx,
        "median_db": float(GAIN_DB[med_idx]),
        "acars_yield_at_median": float(yld),
        "true_plateau_idx": true_idx,
        "true_plateau_db": float(GAIN_DB[true_idx]),
        "modes": mode_str,
    }


# ==========================================================================
# main
# ==========================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sweeps", type=int, default=1000)
    ap.add_argument("--chains", type=int, default=400)
    ap.add_argument("--chain-len", type=int, default=30)
    ap.add_argument("--seed", type=int, default=12345)
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    p = Params()
    ira, aca = make_curves(p)
    out = {}

    def line(*a):
        if not args.json:
            print(*a)

    line("=" * 78)
    line("AUTOGAIN VALIDATION GATE-1 -- Monte-Carlo simulation")
    line(f"sweeps={args.sweeps} chains={args.chains} chain_len={args.chain_len} seed={args.seed}")
    line("=" * 78)

    # ---- curve sanity ----
    line("\nKnee curves (normalised p, peak=1) at reference gains:")
    line("  gain[dB]:  " + "  ".join(f"{g:5.1f}" for g in
         [GAIN_DB[i] for i in (idx_of_db(8), idx_of_db(15), idx_of_db(25),
                               idx_of_db(33.8), idx_of_db(35), idx_of_db(40.2), idx_of_db(44))]))
    for name, cv in (("p_ira ", ira), ("p_acars", aca)):
        vals = [cv[i] for i in (idx_of_db(8), idx_of_db(15), idx_of_db(25),
                                idx_of_db(33.8), idx_of_db(35), idx_of_db(40.2), idx_of_db(44))]
        line(f"  {name}:   " + "  ".join(f"{v:5.2f}" for v in vals))
    line(f"  IRA peak    @ {GAIN_DB[int(np.argmax(ira))]:.1f} dB (idx {int(np.argmax(ira))})")
    line(f"  ACARS peak  @ {GAIN_DB[int(np.argmax(aca))]:.1f} dB (idx {int(np.argmax(aca))})  "
         f"[true plateau ref]")
    line(f"  ACARS p at IRA-peak gain = {aca[int(np.argmax(ira))]:.3f}  "
         f"(if picker converges to IRA peak, this is the ACARS yield)")
    out["curves"] = {
        "ira_peak_db": float(GAIN_DB[int(np.argmax(ira))]),
        "acars_peak_db": float(GAIN_DB[int(np.argmax(aca))]),
        "acars_at_ira_peak": float(aca[int(np.argmax(ira))]),
    }

    # ========================= PART A =========================
    line("\n" + "-" * 78)
    line("PART A -- D_max distribution (does it land ~6-8 [review] or ~16 [design]?)")
    line("-" * 78)
    A = part_A(p, rng, args.sweeps)
    sp1 = stats(A["p1_dmax"])
    srr = stats(A["real_dmax"])
    stot = stats(A["real_total"])
    sp1e = stats(A["p1_dmax_ext"])
    line(f"  (i)  pass-timing only, all arms at IRA plateau, p=1 (no thinning):")
    line(f"       arms = {['%.1f'%g for g in A['arms_plateau_db']]} dB")
    line(f"       D_max, base 8-round sweep : {fmt_stats(sp1)}")
    line(f"       D_max, with auto-extend   : {fmt_stats(sp1e)}")
    line(f"  (ii) realistic first sweep, incumbent 43.4 dB, thinned by p_ira:")
    line(f"       arms = {['%.1f'%g for g in A['arms_first_db']]} dB")
    line(f"       D_max: {fmt_stats(srr)}")
    line(f"       D_total (all arms): {fmt_stats(stot)}")
    line(f"  design assumption: D_i~16 on plateau (sustained 6/min * 160 s).")
    verdictA = ("CONFIRMS review's ~6-8" if 4 <= sp1['median'] <= 9 else "differs from review")
    line(f"  --> base 8-round D_max median {sp1['median']:.0f} (p=1) vs design's 16 --> {verdictA}.")
    line(f"      (auto-extend lifts it to {sp1e['median']:.0f}; real thinned first-sweep only "
         f"{srr['median']:.0f}.)")
    out["A"] = {"p1_dmax": sp1, "p1_dmax_ext": sp1e, "real_dmax": srr, "real_total": stot}

    # ========================= PART B =========================
    line("\n" + "-" * 78)
    line("PART B -- v1 picker: chosen-offset distribution + 30-sweep cross-run drift")
    line("-" * 78)
    # (b1) single-sweep chosen offset distribution (from first sweep of each chain,
    #      but easier: sample chosen offset directly over many first-sweeps)
    B_drift = run_drift(p, rng, pick_v1, args.chain_len, args.chains, ira)
    if len(B_drift["first_chosen_off"]):
        offs, cts = np.unique(B_drift["first_chosen_off"], return_counts=True)
        line("  v1 first-sweep chosen arm offset (table steps rel. incumbent 43.4):")
        for o, c in sorted(zip(offs.tolist(), cts.tolist())):
            line(f"     offset {o:+d}: {100*c/len(B_drift['first_chosen_off']):5.1f}%")
    fsB = finals_summary(p, B_drift["finals"])
    line(f"\n  30-sweep cross-run drift (start 43.4 dB, idx {p.start_inc_idx}):")
    line(f"     final gain: median {fsB['median_db']:.1f} dB (idx {fsB['median_idx']})")
    line(f"     final-gain modes: {fsB['modes']}")
    line(f"     last-5-sweep step spread: {fmt_stats(stats(B_drift['last5_spread']))}")
    line(f"     no-signal-freeze at end: {100*B_drift['no_signal_frac']:.0f}% of chains")
    line(f"     ACARS true plateau @ {fsB['true_plateau_db']:.1f} dB")
    line(f"     ACARS yield at v1's converged gain = {100*fsB['acars_yield_at_median']:.1f}%"
         f"  --> ACARS-yield LOSS = {100*(1-fsB['acars_yield_at_median']):.1f}%")
    line("     example trajectory (dB): " +
         " -> ".join(f"{GAIN_DB[i]:.1f}" for i in B_drift["trajectories"][0][:12]) + " ...")
    out["B"] = {"final": fsB, "last5_spread": stats(B_drift["last5_spread"]),
                "no_signal_frac": B_drift["no_signal_frac"]}

    # ========================= PART C =========================
    line("\n" + "-" * 78)
    line("PART C -- coarse 3-state picker (review prescription): same 30-sweep drift")
    line("-" * 78)
    # C1: coarse measuring IRA (same LO as v1) -> isolates 'does the coarse rule
    #     fix the collapse?' from the wrong-LO issue.
    C_ira = run_drift(p, rng, pick_coarse, args.chain_len, args.chains, ira)
    fsC = finals_summary(p, C_ira["finals"])
    line(f"  C1: coarse measuring IRA LO (same measurement as v1):")
    line(f"      final gain: median {fsC['median_db']:.1f} dB (idx {fsC['median_idx']})")
    line(f"      final-gain modes: {fsC['modes']}")
    line(f"      last-5-sweep step spread: {fmt_stats(stats(C_ira['last5_spread']))}  "
         f"(<=1 step => stable)")
    line(f"      ACARS yield at converged gain = {100*fsC['acars_yield_at_median']:.1f}%"
         f"  --> loss {100*(1-fsC['acars_yield_at_median']):.1f}%")

    # C2: coarse measuring the OPERATING ACARS LO (review's full prescription =
    #     coarse + operating-LO). Reduced count-rate (design: ACARS LO ~5-10x
    #     slower than IRA); use pass_scale to emulate. Report at a few rates.
    line(f"\n  C2: coarse measuring the OPERATING ACARS LO (review's full fix).")
    line(f"      ACARS LO decode rate is ~5-10x lower than IRA (proposal Sec.2);")
    line(f"      shown at several count-rate scalings of the base 13/pass:")
    c2_rows = []
    for scale in (1.0, 0.5, 0.2):
        pp = Params(pass_scale=scale)
        C_ac = run_drift(pp, rng, pick_coarse, args.chain_len, args.chains, aca)
        fs = finals_summary(pp, C_ac["finals"])
        c2_rows.append((scale, fs, stats(C_ac["last5_spread"]), C_ac["no_signal_frac"]))
        line(f"      x{scale:>4}: final median {fs['median_db']:.1f} dB "
             f"(idx {fs['median_idx']}), spread {fmt_stats(stats(C_ac['last5_spread']))}, "
             f"ACARS yield {100*fs['acars_yield_at_median']:.1f}%, "
             f"no-signal {100*C_ac['no_signal_frac']:.0f}%")
    out["C"] = {"ira": {"final": fsC, "spread": stats(C_ira["last5_spread"])},
                "acars_lo": [{"scale": s, "final": fs} for s, fs, _sp, _ns in c2_rows]}

    # ========================= PART D =========================
    line("\n" + "-" * 78)
    line("PART D -- sensitivity sweep on pass rate (0.5x / 1x / 2x decodes/pass)")
    line("-" * 78)
    line("  Both pickers MEASURE on IRA (as v1 does), evaluated vs ACARS yield.")
    line("  Columns: converged gain (median dB), last-5 step spread, ACARS yield %, no-signal %")
    d_rows = []
    for scale in (0.5, 1.0, 2.0):
        pp = Params(pass_scale=scale)
        row = {"scale": scale}
        line(f"  --- pass_scale x{scale} (decodes/pass = {pp.decodes_per_pass*scale:.1f}) ---")
        # D_max context
        arms_first = build_arms(pp, pp.start_inc_idx)
        dm = [int(sweep_counts(pp, arms_first, rng, ira)[0].max()) for _ in range(300)]
        line(f"      D_max (first sweep, thinned): median {int(np.median(dm))} "
             f"p10-p90 [{int(np.percentile(dm,10))},{int(np.percentile(dm,90))}]")
        for name, picker in (("v1    ", pick_v1), ("coarse", pick_coarse)):
            R = run_drift(pp, rng, picker, args.chain_len, args.chains, ira)
            fs = finals_summary(pp, R["finals"])
            sp = stats(R["last5_spread"])
            line(f"      {name}: {fs['median_db']:5.1f} dB (idx {fs['median_idx']:2d})  "
                 f"spread {sp['median']:.0f} (p90 {sp['p90']:.0f})  "
                 f"ACARS {100*fs['acars_yield_at_median']:5.1f}%  "
                 f"no-sig {100*R['no_signal_frac']:3.0f}%")
            row[name.strip()] = {"final": fs, "spread": sp,
                                 "no_signal_frac": R["no_signal_frac"]}
        d_rows.append(row)
    out["D"] = d_rows

    # ========================= VERDICT =========================
    line("\n" + "=" * 78)
    line("VERDICT")
    line("=" * 78)
    v1_loss = 1 - fsB["acars_yield_at_median"]
    v1_drift_down = p.start_inc_idx - fsB["median_idx"]
    c_stable = stats(C_ira["last5_spread"])["p90"] <= 1
    line(f"  Critical 1 (count starvation): D_max median ~{stats(A['real_dmax'])['median']:.0f} "
         f"(review said 6-8; design assumed 16). {'CONFIRMED.' if stats(A['real_dmax'])['median']<=10 else 'not confirmed.'}")
    line(f"  v1 drifts DOWN {v1_drift_down} table steps to {fsB['median_db']:.1f} dB and "
         f"loses {100*v1_loss:.0f}% of ACARS yield.  "
         f"{'COLLAPSE CONFIRMED.' if v1_drift_down>=3 and v1_loss>0.3 else 'no collapse.'}")
    line(f"  Coarse-on-IRA: last-5 spread p90 = {stats(C_ira['last5_spread'])['p90']:.0f} steps "
         f"({'STABLE' if c_stable else 'NOT stable'}); ACARS yield "
         f"{100*fsC['acars_yield_at_median']:.0f}%.")
    line(f"  Coarse fixes the COLLAPSE/thrash (Critical 1) but measuring IRA still")
    line(f"  {'leaves ACARS yield low (Critical 2 persists)' if fsC['acars_yield_at_median']<0.7 else 'is acceptable'};")
    line(f"  the ACARS-LO variant (C2) is what restores ACARS yield -- at the cost of")
    line(f"  count starvation that freezes near the (good) incumbent.")

    if args.json:
        print(json.dumps(out, indent=2, default=float))


if __name__ == "__main__":
    main()
