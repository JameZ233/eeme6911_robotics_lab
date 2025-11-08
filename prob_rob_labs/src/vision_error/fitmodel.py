import re
import numpy as np
import matplotlib.pyplot as plt

# Load file
with open("output.txt") as f:
    lines = f.readlines()

pattern = re.compile(r"e_d=([-\d.]+), e_theta=([-\d.]+)")
errors_d, errors_theta = [], []

for line in lines:
    m = pattern.search(line)
    if m:
        errors_d.append(float(m.group(1)))
        errors_theta.append(float(m.group(2)))

errors_d = np.array(errors_d)
errors_theta = np.array(errors_theta)

# Variance calculation
var_d = np.var(errors_d)
var_theta = np.var(errors_theta)
print(f"Distance variance (m^2): {var_d:.6f}")
print(f"Angle variance (rad^2): {var_theta:.6e}")

# Define model
def var_model_d(d): return var_d         
def var_model_theta(theta): return var_theta

# Plot
plt.figure(figsize=(10,4))
plt.subplot(1,2,1)
plt.hist(errors_d, bins=30, color='blue', edgecolor='k')
plt.title("Distance Error Distribution")
plt.xlabel("e_d (m)")
plt.ylabel("Count")

plt.subplot(1,2,2)
plt.hist(errors_theta, bins=30, color='red', edgecolor='k')
plt.title("Bearing Error Distribution")
plt.xlabel("e_theta (rad)")
plt.tight_layout()
plt.show()
