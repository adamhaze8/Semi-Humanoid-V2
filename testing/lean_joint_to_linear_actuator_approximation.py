import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import linregress

# Fixed side lengths (in mm)
a = 271
b = 96

# Linear fit range in degrees
fit_min_angle = 60   # lower bound of angle range to fit
fit_max_angle = 120  # upper bound of angle range to fit

# Generate actuator lengths
c_min = abs(a - b) + 1
c_max = a + b - 1
c_values = np.linspace(c_min, c_max, 500)

# Calculate angles
angles_rad = np.arccos((a**2 + b**2 - c_values**2) / (2 * a * b))
angles_deg = np.degrees(angles_rad)

# Filter data within desired angle range
mask = (angles_deg >= fit_min_angle) & (angles_deg <= fit_max_angle)
c_fit = c_values[mask]
angle_fit = angles_deg[mask]

# Perform linear regression on filtered data
slope, intercept, r_value, p_value, std_err = linregress(c_fit, angle_fit)

# Print slope with units
print(f"Slope: {slope:.4f} degrees per mm (fit over {fit_min_angle}° to {fit_max_angle}°)")

# Plot
plt.figure(figsize=(8, 5))
plt.plot(c_values, angles_deg, label="Actual Angle", color='blue')
plt.plot(c_fit, slope * c_fit + intercept, '--', label="Linear Fit", color='red')
plt.axhline(fit_min_angle, color='gray', linestyle=':', linewidth=0.7)
plt.axhline(fit_max_angle, color='gray', linestyle=':', linewidth=0.7)
plt.xlabel("Actuator Length (mm)")
plt.ylabel("Lean Angle (degrees)")
plt.title("Lean Angle vs Actuator Length with Adjustable Linear Fit Range")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
