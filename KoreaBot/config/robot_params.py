"""KoreaBot robot settings - edit the numbers and save.

The window that owns this file sends a saved change to the robot at once.
Allowed range is in [brackets]; an out-of-range value is refused and the
previous one kept.
"""


# ===== SPEED  (m/s, the robot's top speed is 0.22) =============================

forward_speed = 0.18          # m/s when the line sits exactly on target  [0.0 - 0.22]
min_speed = 0.04              # m/s floor while turning hard  [0.0 - 0.22]
slow_down = 0.8               # share of forward_speed dropped at a full turn (0 = never slow)  [0.0 - 1.0]
search_speed = 0.015          # m/s forward while searching (0 = spin on the spot)  [0.0 - 0.22]


# ===== STEERING  (rad/s = how fast it turns) ===================================

kp = 1.2                      # turn strength: rad/s per unit of error (1 = a quarter picture off target)  [0.0 - 10.0]
kd = 0.08                     # rad/s per unit of error change per second (damps wobble)  [0.0 - 5.0]
max_turn = 1.6                # rad/s cap on steering (turn speed)  [0.0 - 2.84]
search_rate = 0.45            # rad/s turn while searching  [0.0 - 2.84]
invert_steering = False       # True = flip turn direction (mirrored camera)


# ===== WHERE THE LINES SHOULD BE  (fractions of the picture: 0 = left, 1 = right) ===

target_x = 0.778              # white: where the line should be (ring), share of width  [0.0 - 1.0]
scan_from = 0.4               # white: the dot looks from here to the right edge  [0.0 - 0.95]
yellow_target_x = 0.039       # yellow: where the line should be (ring), share of width  [0.0 - 1.0]
yellow_scan_to = 0.6          # yellow: the dot looks from the left edge to here  [0.05 - 1.0]
center_x = 0.5                # both lines: where the middle of the lane should be, share of width  [0.0 - 1.0]
lane_width = 0.6              # both lines: gap between them to assume until both are seen  [0.05 - 1.0]


# ===== TRACKING DOTS ===========================================================

crop_top = 0.25               # line tracking ignores this share of the picture at the top  [0.0 - 0.9]
dot_y = 0.548                 # height of the dots and rings, share of the part below crop_top (0 = top)  [0.0 - 1.0]
dot_size = 12                 # dot radius in camera pixels - it looks at a strip this tall  [2 - 100]
dot_reach = 0.1               # how far a dot may slide per frame, share of width  [0.01 - 0.5]
dot_smooth = 0.5              # 0 = dot jumps straight to the line, higher = glides  [0.0 - 0.95]


# ===== LOST LINE / CAMERA  (seconds) ===========================================

search_enabled = True         # True = turn to look for the line when it is lost
lost_grace = 0.5              # s to keep the last command after the line vanishes  [0.0 - 10.0]
search_time = 4.0             # s spent turning each way; gives up after 3x this  [0.0 - 60.0]
image_timeout = 2.0           # s without a camera frame before the robot stops  [0.2 - 60.0]
