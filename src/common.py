"""
Shared core for The Living Map simulation (Writer + Executor).
No pygame in this file: world, sensors, beacon codec, wall-follower.
"""
import math, random, struct

PX_PER_M = 20.0
FPS = 60
WINDOW_SIZE = (1000, 700)
ROBOT_R = 14
LIDAR_RAYS, LIDAR_MAX, LIDAR_FOV = 9, 120.0, math.radians(180)
BLE_RANGE_PX = 300.0        # ~15 m with line of sight (40% of that without)
MAX_HOP_PX = 240.0          # max distance between two consecutive beacons
START_POSE = (100.0, 100.0, 0.0)
FRESH_S, AGING_S = 300, 1800  # age thresholds in seconds
HOP_MARGIN = ROBOT_R + 5      # wall clearance a straight hop between two beacons must keep (px)

# walls as (x, y, w, h)
WALLS = [(40, 40, 920, 15), (40, 640, 920, 15), (40, 40, 15, 600), (945, 40, 15, 600),
         (280, 40, 15, 400), (40, 420, 160, 15), (540, 200, 15, 440),
         (540, 200, 260, 15), (750, 360, 200, 15)]

# ---- every first-aid problem the Writer can detect + what the Executor does about it.
# To add a new problem: add one entry here and one entry in make_sources().
EVENT_CATALOG = {
    1: {"name": "Victim - unresponsive", "priority": 7, "color": (231, 76, 60), "approach_px": 26,
        "steps": [("Assess vital signs (thermal + audio)", 2.0),
                  ("Deliver first-aid kit + oxygen", 3.0),
                  ("Alert command post: PRIORITY 1 victim", 1.0)], "neutralize": False},
    2: {"name": "Victim - responsive", "priority": 5, "color": (46, 204, 113), "approach_px": 26,
        "steps": [("Open two-way audio link", 2.0),
                  ("Deliver water + blanket", 3.0),
                  ("GUIDE: tell victim to follow the beacon trail to the exit", 2.0)], "neutralize": False},
    3: {"name": "Fire / high heat", "priority": 6, "color": (230, 126, 34), "approach_px": 42,
        "steps": [("Aim extinguisher", 1.0), ("Extinguish fire", 4.0),
                  ("Confirm temperature drop", 1.0)], "neutralize": True},
    4: {"name": "Toxic gas / smoke", "priority": 6, "color": (155, 89, 182), "approach_px": 42,
        "steps": [("Mark no-go zone", 1.0), ("Open vent / seal leak", 4.0),
                  ("Confirm air quality", 1.0)], "neutralize": True},
}


def make_sources():
    """Ground truth of the disaster scene (only the sensors may look at this)."""
    return [{"type": 1, "pos": (140, 530), "range": 90, "active": True},
            {"type": 2, "pos": (420, 100), "range": 90, "active": True},
            {"type": 3, "pos": (860, 110), "range": 110, "active": True},
            {"type": 4, "pos": (880, 540), "range": 100, "active": True}]


# ---------------- geometry ----------------
def circle_hits_rect(cx, cy, rad, rect):
    rx, ry, rw, rh = rect
    nx, ny = max(rx, min(cx, rx + rw)), max(ry, min(cy, ry + rh))
    return (cx - nx) ** 2 + (cy - ny) ** 2 < rad * rad


def collides(x, y, rad=ROBOT_R):
    return any(circle_hits_rect(x, y, rad, w) for w in WALLS)


def point_in_wall(x, y):
    return any(rx <= x <= rx + rw and ry <= y <= ry + rh for rx, ry, rw, rh in WALLS)


def los_clear(p, q, step=5.0):
    """Line of sight between two points (used by sensors and radio)."""
    n = max(1, int(math.hypot(q[0] - p[0], q[1] - p[1]) / step))
    return not any(point_in_wall(p[0] + (q[0] - p[0]) * i / n, p[1] + (q[1] - p[1]) * i / n)
                   for i in range(1, n))


def segment_clear(p, q, margin=None, step=4.0):
    """A robot can drive straight from p to q without touching a wall."""
    margin = HOP_MARGIN if margin is None else margin
    L = math.hypot(q[0] - p[0], q[1] - p[1])
    if L < 10: return True                      # tiny hop: both ends are valid robot poses
    n = max(1, int(L / step))
    return not any(collides(p[0] + (q[0] - p[0]) * i / n, p[1] + (q[1] - p[1]) * i / n, margin)
                   for i in range(n + 1))


def try_move(x, y, dx, dy):
    """Move with wall sliding. Returns (x, y, moved)."""
    if not collides(x + dx, y + dy): return x + dx, y + dy, True
    if not collides(x + dx, y): return x + dx, y, True
    if not collides(x, y + dy): return x, y + dy, True
    return x, y, False


def cast_lidar(x, y, heading):
    """9 rays, index 0 = -90 deg (left) ... 4 = ahead ... 8 = +90 deg (right)."""
    out, step = [], LIDAR_FOV / (LIDAR_RAYS - 1)
    for i in range(LIDAR_RAYS):
        a = heading - LIDAR_FOV / 2 + i * step
        ca, sa, d = math.cos(a), math.sin(a), LIDAR_MAX
        for s in range(8, int(LIDAR_MAX), 4):
            if point_in_wall(x + s * ca, y + s * sa):
                d = float(s); break
        out.append(d)
    return out


# ---------------- sensors ----------------
def sense(x, y, sources, noise=0.02):
    """{event_type: (reading 0..1, bearing_rad, estimated_distance_px)}.
    Needs line of sight (thermal/flame/gas plume modelled the same way)."""
    out = {t: (0.0, 0.0, 0.0) for t in EVENT_CATALOG}
    for s in sources:
        if not s["active"]: continue
        sx, sy = s["pos"]
        d = math.hypot(sx - x, sy - y)
        if d >= s["range"] or not los_clear((x, y), (sx, sy)): continue
        r = max(0.0, min(1.0, 1.0 - d / s["range"] + random.gauss(0, noise)))
        if r > out[s["type"]][0]:
            out[s["type"]] = (r, math.atan2(sy - y, sx - x), (1.0 - r) * s["range"])
    return out


# ---------------- beacon codec (11 bytes) ----------------
class BeaconCodec:
    """ID 2B | WHAT 1B (3b priority + 5b type) | HEADING 1B | DIST 1B (dm) | WHEN 4B | VER 1B | CRC 1B.
    HEADING/DIST = vector from THIS beacon back to the PREVIOUS beacon (type 0 = plain waypoint)."""
    @staticmethod
    def pack(beacon_id, event_type, priority, heading_deg, distance_m, timestamp, version=1):
        what = ((priority & 7) << 5) | (event_type & 0x1F)
        hb = int(round((heading_deg % 360.0) / 360.0 * 256)) % 256
        db = int(round(min(25.5, max(0.0, distance_m)) * 10))
        body = struct.pack(">HBBBIB", beacon_id & 0xFFFF, what, hb, db, timestamp & 0xFFFFFFFF, version & 0xFF)
        crc = 0
        for b in body: crc ^= b
        return body + bytes([crc])

    @staticmethod
    def unpack(payload):
        if len(payload) != 11: raise ValueError("bad length")
        crc = 0
        for b in payload: crc ^= b
        if crc: raise ValueError("CRC mismatch")
        bid, what, hb, db, ts, ver, _ = struct.unpack(">HBBBIBB", payload)
        return {"id": bid, "event_type": what & 0x1F, "priority": what >> 5,
                "heading_deg": hb * 360.0 / 256, "distance_m": db / 10.0,
                "timestamp": ts, "version": ver}


def age_state(age_s):
    return "FRESH" if age_s < FRESH_S else ("AGING" if age_s < AGING_S else "STALE")


# ---------------- reactive wall follower ----------------
class WallFollower:
    """side=+1: keep the wall on the right. side=-1: wall on the left (mirror image)."""
    def __init__(self, side=1, desired=45.0, front_min=40.0):
        self.side, self.desired, self.front_min, self.recover = side, desired, front_min, 0

    def trigger_recovery(self): self.recover = 18

    def step(self, ranges):
        """returns (linear speed px/tick, turn rate rad/tick; positive = clockwise on screen)"""
        l = ranges if self.side > 0 else ranges[::-1]
        if self.recover > 0:
            self.recover -= 1
            return -1.0, -0.06 * self.side          # back up, turning away from the wall
        front, right, fr = min(l[3], l[4], l[5]), l[8], l[6]
        if front < self.front_min:                   # concave corner: pivot away from wall
            return 0.3, -0.09 * self.side
        if right >= LIDAR_MAX - 1:                   # wall ended (convex corner): curve round it
            return 1.8, 0.045 * self.side
        # heading error w.r.t. the wall (ignore the diagonal beam when it already lost the wall: a corner is coming)
        a = 0.0 if fr >= LIDAR_MAX - 1 else math.atan2(fr * 0.7071 - right, fr * 0.7071)
        dist = right * math.cos(a)
        turn = max(-0.08, min(0.08, 0.0015 * (dist - self.desired) + 0.12 * a))
        return 2.0, turn * self.side
