"""Rules for KoreaBot's settings, and the files that remember them.

Two separate groups, each with its own window and its own file:

    COLORS  colour of the lines   color_params window  -> config/colors.py
    ROBOT   speed, steering, ...  robot_params window  -> config/robot_params.py

The files are plain Python, one "name = value" line per setting with a
comment, so they can also be edited by hand; the window owning the file
notices the change and sends it on.  Every value is range-checked: a bad value
is refused and the previous one kept, so a typo can never make the robot bolt.
The defaults below are what the Defaults button puts back.

No ROS imports here - this is shared by all three programs.
"""

import ast
import os
import re
import tempfile


# ไฟล์ค่าอยู่ในโฟลเดอร์ source แก้ด้วยมือได้เลย ไม่ต้อง colcon build ใหม่
CONFIG_DIR = os.environ.get(
    "KOREABOT_CONFIG", os.path.expanduser("~/ros2_ws/src/KoreaBot/config"))


# name: (default, min, max, meaning)   ชนิดดูจาก default: bool / int / float
COLOR_SCHEMA = {
    "s_max": (60, 0, 255, "white: saturation max (colours have high S)"),
    "v_min": (180, 0, 255, "white: brightness min (raise if it grabs reflections)"),

    "h_low": (20, 0, 179, "yellow: hue min (OpenCV hue is 0 - 179)"),
    "h_high": (35, 0, 179, "yellow: hue max"),
    "s_low": (100, 0, 255, "yellow: saturation min (white has low S)"),
    "v_low": (100, 0, 255, "yellow: brightness min"),

    "ignore_round": (True, None, None,
                     "True = erase round white blobs (ceiling lights, reflections)"),
    "light_min_circularity": (0.8, 0.1, 1.0,
                              "blobs this round (1 = circle) are lights and get erased"),
    "light_margin": (1.3, 1.0, 5.0,
                     "erase this many times the light's radius, to remove its glow"),

    "col_ratio": (0.30, 0.01, 1.0,
                  "share of a dot-strip column that must be line colour to count"),
    "min_width_px": (3, 1, 400, "narrower patches are ignored as noise"),
}

COLOR_SECTIONS = [
    ("WHITE LINE  (HSV, 0 - 255)", ["s_max", "v_min"]),
    ("YELLOW LINE  (HSV)", ["h_low", "h_high", "s_low", "v_low"]),
    ("CEILING LIGHT FILTER  (white only)",
     ["ignore_round", "light_min_circularity", "light_margin"]),
    ("DETECTION", ["col_ratio", "min_width_px"]),
]

COLOR_PAIRS = [("h_low", "h_high")]


ROBOT_SCHEMA = {
    "forward_speed": (0.06, 0.0, 0.22, "m/s when the line sits exactly on target"),
    "min_speed": (0.02, 0.0, 0.22, "m/s floor while turning hard"),
    "slow_down": (0.6, 0.0, 1.0,
                  "share of forward_speed dropped at a full turn (0 = never slow)"),
    "search_speed": (0.03, 0.0, 0.22, "m/s forward while searching (0 = spin on the spot)"),

    "kp": (1.2, 0.0, 10.0,
           "turn strength: rad/s per unit of error (1 = a quarter picture off target)"),
    "kd": (0.05, 0.0, 5.0, "rad/s per unit of error change per second (damps wobble)"),
    "max_turn": (1.0, 0.0, 2.84, "rad/s cap on steering (turn speed)"),
    "search_rate": (0.3, 0.0, 2.84, "rad/s turn while searching"),
    "invert_steering": (False, None, None, "True = flip turn direction (mirrored camera)"),

    "target_x": (0.80, 0.0, 1.0, "white: where the line should be (ring), share of width"),
    "scan_from": (0.4, 0.0, 0.95, "white: the dot looks from here to the right edge"),
    "yellow_target_x": (0.20, 0.0, 1.0,
                        "yellow: where the line should be (ring), share of width"),
    "yellow_scan_to": (0.6, 0.05, 1.0, "yellow: the dot looks from the left edge to here"),
    "center_x": (0.5, 0.0, 1.0,
                 "both lines: where the middle of the lane should be, share of width"),
    "lane_width": (0.6, 0.05, 1.0,
                   "both lines: gap between them to assume until both are seen"),

    "crop_top": (0.25, 0.0, 0.9, "line tracking ignores this share of the picture at the top"),
    "dot_y": (0.70, 0.0, 1.0,
              "height of the dots and rings, share of the part below crop_top (0 = top)"),
    "dot_size": (12, 2, 100, "dot radius in camera pixels - it looks at a strip this tall"),
    "dot_reach": (0.10, 0.01, 0.5, "how far a dot may slide per frame, share of width"),
    "dot_smooth": (0.5, 0.0, 0.95, "0 = dot jumps straight to the line, higher = glides"),

    "search_enabled": (True, None, None, "True = turn to look for the line when it is lost"),
    "lost_grace": (0.5, 0.0, 10.0, "s to keep the last command after the line vanishes"),
    "search_time": (4.0, 0.0, 60.0, "s spent turning each way; gives up after 3x this"),
    "image_timeout": (2.0, 0.2, 60.0, "s without a camera frame before the robot stops"),
}

ROBOT_SECTIONS = [
    ("SPEED  (m/s, the robot's top speed is 0.22)",
     ["forward_speed", "min_speed", "slow_down", "search_speed"]),
    ("STEERING  (rad/s = how fast it turns)",
     ["kp", "kd", "max_turn", "search_rate", "invert_steering"]),
    ("WHERE THE LINES SHOULD BE  (fractions of the picture: 0 = left, 1 = right)",
     ["target_x", "scan_from", "yellow_target_x", "yellow_scan_to", "center_x", "lane_width"]),
    ("TRACKING DOTS", ["crop_top", "dot_y", "dot_size", "dot_reach", "dot_smooth"]),
    ("LOST LINE / CAMERA  (seconds)",
     ["search_enabled", "lost_grace", "search_time", "image_timeout"]),
]

ROBOT_PAIRS = [
    ("scan_from", "target_x"),            # วงอยู่นอกช่วงที่จุดหาเส้นได้ = ไม่มีวันเข้าเป้า
    ("yellow_target_x", "yellow_scan_to"),
]


def coerce(schema, name, value):
    """Turn value into the type and range schema wants, or raise ValueError."""

    if name not in schema:
        raise ValueError(f"unknown setting '{name}'")

    default, low, high, _ = schema[name]

    if isinstance(default, bool):
        if isinstance(value, bool):
            return value
        text = str(value).strip().lower()
        if text in ("true", "1", "yes", "on"):
            return True
        if text in ("false", "0", "no", "off"):
            return False
        raise ValueError(f"{name}: expected True / False, got {value!r}")

    # bool เป็น subclass ของ int ต้องกันไว้ ไม่งั้น True กลายเป็น 1
    if isinstance(value, bool):
        raise ValueError(f"{name}: expected a number, got {value!r}")

    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{name}: expected a number, got {value!r}") from None

    if number != number:  # NaN
        raise ValueError(f"{name}: NaN is not allowed")

    if isinstance(default, int):
        if number != int(number):
            raise ValueError(f"{name}: expected a whole number, got {value!r}")
        number = int(number)

    if not (low <= number <= high):
        raise ValueError(f"{name}: {number} is outside {low} .. {high}")

    return number


class Settings:
    """One group of settings (colours or robot) and the file that remembers it."""

    def __init__(self, filename, title, schema, sections, pairs):
        self.path = os.path.join(CONFIG_DIR, filename)
        self.title = title
        self.schema = schema
        self.sections = sections
        self.pairs = pairs

    def defaults(self):
        return {name: spec[0] for name, spec in self.schema.items()}

    def validate(self, raw, base=None):
        """Merge raw into base, keeping base's value for anything invalid.

        Returns (values, errors).
        """
        values = dict(base if base is not None else self.defaults())
        errors = []

        if not isinstance(raw, dict):
            return values, ["expected {name: value}"]

        for name, value in raw.items():
            try:
                values[name] = coerce(self.schema, name, value)
            except ValueError as error:
                errors.append(str(error))

        fallback = base if base is not None else self.defaults()
        for low, high in self.pairs:
            if values[low] > values[high]:
                errors.append(f"{low} ({values[low]}) must not exceed {high} ({values[high]})")
                values[low], values[high] = fallback[low], fallback[high]

        return values, errors

    # ---------------- file ----------------

    def mtime(self):
        try:
            return os.stat(self.path).st_mtime_ns
        except OSError:
            return None

    def load(self, base=None):
        """Read and validate the file.  Creates it with defaults if missing."""

        fallback = dict(base if base is not None else self.defaults())

        if not os.path.exists(self.path):
            self.save(fallback)
            return fallback, []

        try:
            with open(self.path) as handle:
                tree = ast.parse(handle.read(), filename=os.path.basename(self.path))
        except (OSError, SyntaxError) as error:
            return fallback, [f"could not read {os.path.basename(self.path)}: {error}"]

        # อ่านเฉพาะบรรทัด name = ค่า ไม่ได้รันไฟล์ โค้ดอื่นในไฟล์จึงไม่มีผลอะไร
        raw, errors = {}, []
        for node in tree.body:
            if not (isinstance(node, ast.Assign) and len(node.targets) == 1
                    and isinstance(node.targets[0], ast.Name)):
                continue
            name = node.targets[0].id
            try:
                raw[name] = ast.literal_eval(node.value)
            except ValueError:
                errors.append(f"line {node.lineno}: {name} must be a plain number or True / False")

        values, more = self.validate(raw, fallback)
        return values, errors + more

    def save(self, values):
        """Write values into the file, changing only the numbers (comments stay)."""

        if not os.path.exists(self.path):
            write_text_atomic(self.path, self._fresh_text(values))
            return

        with open(self.path) as handle:
            text = handle.read()

        missing = []
        for name in self.schema:
            pattern = re.compile(
                rf"^(?P<pre>{name}\s*=\s*)(?P<value>[^#\n]*?)(?P<gap>[ \t]*)(?P<comment>#.*)?$",
                re.M,
            )
            match = pattern.search(text)
            if match is None:
                missing.append(name)
                continue

            new = format_value(values[name])
            if match["value"].strip() == new:
                continue

            # คอมเมนต์อยู่คอลัมน์เดิม ไม่เลื่อนไปมาตามความยาวตัวเลข
            gap = match["gap"]
            if match["comment"]:
                gap = " " * max(1, len(match["value"]) + len(gap) - len(new))
            line = match["pre"] + new + gap + (match["comment"] or "")
            text = text[:match.start()] + line + text[match.end():]

        if missing:
            text = text.rstrip("\n") + "\n\n" + "\n".join(
                self._line(name, values[name]) for name in missing) + "\n"

        write_text_atomic(self.path, text)

    def _line(self, name, value):
        _, low, high, meaning = self.schema[name]
        span = "" if low is None else f"  [{low} - {high}]"
        code = f"{name} = {format_value(value)}"
        return f"{code:<29} # {meaning}{span}"

    def _fresh_text(self, values):
        lines = [
            f'"""{self.title} - edit the numbers and save.',
            "",
            "The window that owns this file sends a saved change to the robot at once.",
            "Allowed range is in [brackets]; an out-of-range value is refused and the",
            "previous one kept.",
            '"""',
        ]
        listed = [name for _, names in self.sections for name in names]
        rest = [name for name in self.schema if name not in listed]
        for title, names in self.sections + ([("OTHER", rest)] if rest else []):
            lines += ["", "", f"# ===== {title} " + "=" * max(3, 72 - len(title)), ""]
            lines += [self._line(name, values[name]) for name in names]
        return "\n".join(lines) + "\n"


def format_value(value):
    if isinstance(value, bool):
        return "True" if value else "False"
    return repr(value)


def write_text_atomic(path, text):
    """Write to a temp file and rename, so a reader never sees half a file."""

    folder = os.path.dirname(path) or "."
    os.makedirs(folder, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=folder, prefix=".tmp_")
    try:
        with os.fdopen(fd, "w") as handle:
            handle.write(text)
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


COLORS = Settings("colors.py", "KoreaBot colour settings",
                  COLOR_SCHEMA, COLOR_SECTIONS, COLOR_PAIRS)
ROBOT = Settings("robot_params.py", "KoreaBot robot settings",
                 ROBOT_SCHEMA, ROBOT_SECTIONS, ROBOT_PAIRS)
