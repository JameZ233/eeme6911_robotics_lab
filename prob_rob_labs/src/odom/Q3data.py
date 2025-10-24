import numpy as np
import matplotlib.pyplot as plt

filename = "twist_x_log.txt"

timestamps = []
values = []

with open(filename, 'r') as f:
    for line in f:
        parts = line.strip().split(',')
        if len(parts) != 2:
            continue
        t_str, v_str = parts
        if v_str == '---':   # skip missing samples
            continue
        try:
            timestamps.append(float(t_str))
            values.append(float(v_str))
        except ValueError:
            continue

# Convert to numpy arrays
t = np.array(timestamps)
v = np.array(values)

# Normalize time so it starts from 0
t = t - t[0]

print(f"Loaded {len(v)} samples from {filename}")
print(f"Time span: {t[-1]-t[0]:.3f} s")

# --- Compute tau (time constant = time to reach 90% of final) ---
v0 = v[0]
vf = np.median(v[-10:])      # steady-state value
dv = vf - v0
v90 = v0 + 0.9 * dv          # 90% level

# find first time that crosses 90%
idx = np.where(v >= v90)[0][0]
tau = t[idx]
# Calculate delta_t (average sample interval)
dt = np.mean(np.diff(t))

# Calculate a = 0.1^(Δt / τ)
a = 0.1 ** (dt / tau)

print(f"Initial v0 = {v0:.3f}")
print(f"Final vf = {vf:.3f}")
print(f"Tau (time-to-90%) = {tau:.3f} s")
print(f"Δt (average sample interval) = {dt:.4f} s")
print(f"a (forgetting factor) = {a:.6f}")

filename = "twist_log.txt"   # your .txt file name

timestamps = []
values = []

with open(filename, 'r') as f:
    for line in f:
        parts = line.strip().split(',')
        if len(parts) != 2:
            continue
        t_str, v_str = parts
        if v_str == '---':   # skip missing samples
            continue
        try:
            timestamps.append(float(t_str))
            values.append(float(v_str))
        except ValueError:
            continue

# Convert to numpy arrays
t = np.array(timestamps)
v = np.array(values)

# Normalize time so it starts from 0
t = t - t[0]

print(f"Loaded {len(v)} samples from {filename}")
print(f"Time span: {t[-1]-t[0]:.3f} s")

# --- Compute tau (time constant = time to reach 90% of final) ---
v0 = v[0]
vf = np.median(v[-10:])      # steady-state value
dv = vf - v0
v90 = v0 + 0.9 * dv          # 90% level

# find first time that crosses 90%
idx = np.where(v >= v90)[0][0]
tau = t[idx]
# Calculate delta_t (average sample interval)
dt = np.mean(np.diff(t))

# Calculate a = 0.1^(Δt / τ)
a = 0.1 ** (dt / tau)

print(f"Initial v0 = {v0:.3f}")
print(f"Final vf = {vf:.3f}")
print(f"Tau (time-to-90%) = {tau:.3f} s")
print(f"Δt (average sample interval) = {dt:.4f} s")
print(f"a (forgetting factor) = {a:.6f}")