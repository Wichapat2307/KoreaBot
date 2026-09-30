"""KoreaBot colour settings - edit the numbers and save.

The window that owns this file sends a saved change to the robot at once.
Allowed range is in [brackets]; an out-of-range value is refused and the
previous one kept.
"""


# ===== WHITE LINE  (HSV, 0 - 255) ==============================================

s_max = 110                   # white: saturation max (colours have high S)  [0 - 255]
v_min = 246                   # white: brightness min (raise if it grabs reflections)  [0 - 255]


# ===== YELLOW LINE  (HSV) ======================================================

h_low = 24                    # yellow: hue min (OpenCV hue is 0 - 179)  [0 - 179]
h_high = 31                   # yellow: hue max  [0 - 179]
s_low = 114                   # yellow: saturation min (white has low S)  [0 - 255]
v_low = 197                   # yellow: brightness min  [0 - 255]


# ===== CEILING LIGHT FILTER  (white only) ======================================

ignore_round = True           # True = erase round white blobs (ceiling lights, reflections)
light_min_circularity = 0.82  # blobs this round (1 = circle) are lights and get erased  [0.1 - 1.0]
light_margin = 1.3            # erase this many times the light's radius, to remove its glow  [1.0 - 5.0]


# ===== DETECTION ===============================================================

col_ratio = 0.3               # share of a dot-strip column that must be line colour to count  [0.01 - 1.0]
min_width_px = 3              # narrower patches are ignored as noise  [1 - 400]
