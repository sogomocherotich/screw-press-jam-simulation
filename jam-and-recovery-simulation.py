import numpy as np
import matplotlib.pyplot as plt

# ---------------- Illustrative DC motor + auger model  ----------------
R, kt, ke = 1.0, 0.05, 0.05      # ohm, N.m/A, V.s/rad
J, b = 0.002, 0.0005             # inertia, viscous friction
VNOM, T0 = 12.0, 0.02            # supply voltage, normal load torque
dt = 0.002                       # small step: load torque is stiff near w = 0 (0.01 can chatter)
t = np.arange(0, 60, dt)

I_NOM, W_NOM = 2.33, 193.0       # healthy current / speed
I_HIGH = 1.5 * I_NOM             # current trigger (RUN -> SLOW)
I_OK = 0.9 * I_NOM               # "jam gone" check, measured at HALF voltage in SLOW 
W_LOW = 0.7 * W_NOM              # speed trigger

CLEAR_RATE = 0.3                 # plug removed per second at full reverse 

# ---------------- Controller settings  ----------------
DEBOUNCE = 0.15       # s  overload must persist this long before reacting
RUN_GRACE = 3.0       # s  ignore triggers just after entering RUN (startup / resume settling)
SLOW_DWELL = 2.5      # s  time at half speed before judging "jam gone?"
STOP_HOLD = 0.1       # s  stay at zero this long before changing direction
STOP_MAX = 3.0        # s  give up waiting for zero speed
REVERSE_TIME = 1.5    # s
RESUME_MAX = 6.0      # s  safety timeout for the ramp back up
SLEW = 6.0            # V/s  ramp rate of the commanded voltage (stands in for speed ramp)
MAX_ATTEMPTS = 3      # reverse attempts before FAULT
HEALTHY_RESET = 10.0  # s  of clean running that resets the attempt counter

V_TARGET = {"RUN": VNOM, "SLOW": 0.5 * VNOM, "STOP1": 0.0, "REVERSE": -0.25 * VNOM,
            "STOP2": 0.0, "RESUME": VNOM, "FAULT": 0.0}
STATES = list(V_TARGET)          # RUN=0, SLOW=1, STOP1=2, REVERSE=3, STOP2=4, RESUME=5, FAULT=6


def simulate(control=True, clear_rate=CLEAR_RATE):
    w, plug = 0.0, 0.0
    v_cmd = VNOM                              # start at full voltage, same as the baseline
    state, t_state = "RUN", 0.0
    over_t = still_t = 0.0
    attempts = 0
    out = {k: [] for k in ("i", "w", "plug", "state")}

    def go(new_state):
        nonlocal state, t_state, over_t, still_t
        state, t_state, over_t, still_t = new_state, 0.0, 0.0, 0.0

    for tk in t:
        # 1. ramp commanded voltage toward the target of the current state
        v_goal = V_TARGET[state] if control else VNOM
        v_cmd += np.clip(v_goal - v_cmd, -SLEW * dt, SLEW * dt)

        # 2. jam physics: grows with forward feed, cleared by reversing
        if 15 < tk < 30:
            plug += 0.03 * dt * max(v_cmd, 0) / VNOM
        if v_cmd < 0:
            plug = max(plug - clear_rate * (-v_cmd / (0.25 * VNOM)) * dt, 0)
        elif state == "SLOW":
            plug = max(plug - 0.005 * dt, 0)

        # 3. motor + load
        i = (v_cmd - ke * w) / R
        load = (T0 + plug) * np.tanh(w / 1.0)
        w += dt * (kt * i - b * w - load) / J

        # 4. state machine (uses only i and w)
        if control:
            t_state += dt
            if state == "RUN":
                bad = abs(i) > I_HIGH or w < W_LOW
                over_t = over_t + dt if bad else 0.0
                if t_state > RUN_GRACE and over_t > DEBOUNCE:
                    go("SLOW")
                elif t_state > HEALTHY_RESET:
                    attempts = 0
            elif state == "SLOW":
                if t_state > SLOW_DWELL:                 # judge only after things settle
                    if abs(i) < I_OK:
                        go("RUN")                        # false alarm / jam gone
                    elif attempts >= MAX_ATTEMPTS:
                        go("FAULT")
                    else:
                        attempts += 1
                        go("STOP1")
            elif state in ("STOP1", "STOP2"):
                still = abs(w) < 3.0 and abs(v_cmd) < 0.1
                still_t = still_t + dt if still else 0.0
                if still_t > STOP_HOLD or t_state > STOP_MAX:
                    go("REVERSE" if state == "STOP1" else "RESUME")
            elif state == "REVERSE":
                if t_state > REVERSE_TIME:
                    go("STOP2")
            elif state == "RESUME":                      # triggers ignored here
                if (v_cmd > VNOM - 0.01 and w > 0.9 * W_NOM) or t_state > RESUME_MAX:
                    go("RUN")

        out["i"].append(i); out["w"].append(w); out["plug"].append(plug)
        out["state"].append(STATES.index(state))
    return {k: np.array(v) for k, v in out.items()}


base, ctrl = simulate(False), simulate(True)

def report(name, r):
    m = t > 5                                 
    over = np.sum(np.abs(r["i"][m]) > I_HIGH) * dt
    fwd = np.mean(np.clip(r["w"][t > 15], 0, None)) / W_NOM
    print(f"{name:13s} peak |I| {np.abs(r['i'][m]).max():5.1f} A | "
          f"time above I_HIGH {over:5.1f} s | "
          f"mean fwd speed after 15 s {100 * fwd:4.0f}% | final speed {r['w'][-1]:5.0f} rad/s | "
          f"final plug {r['plug'][-1]:.2f}")

report("no control", base)
report("with control", ctrl)
print("ended in state:", STATES[ctrl["state"][-1]])


fig, ax = plt.subplots(4, 1, figsize=(9, 10), sharex=True)
for name, r in (("no control", base), ("with control", ctrl)):
    ax[0].plot(t, r["i"], label=name)
    ax[1].plot(t, r["w"], label=name)
    ax[2].plot(t, r["plug"], label=name)
ax[0].axhline(I_HIGH, color="grey", ls="--", lw=0.8, label="I_HIGH trigger")
ax[3].plot(t, ctrl["state"], drawstyle="steps-post")
ax[3].set_yticks(range(len(STATES))); ax[3].set_yticklabels(STATES)
ax[0].set_ylabel("Current (A)"); ax[1].set_ylabel("Speed (rad/s)")
ax[2].set_ylabel("Jam torque (N.m)"); ax[3].set_ylabel("State"); ax[3].set_xlabel("Time (s)")
ax[0].legend(); plt.tight_layout()
plt.savefig("jam_simulation.png", dpi=120)
plt.show()