"""
WRITER robot - explores alone, detects every first-aid problem, drops beacons, returns, exports writer_log.json
Run:      python writer.py            (window)      keys: G show/hide hidden sources, 1/2/3 speed x1/x4/x16, ESC quit
Headless: python writer.py --headless (no window, writes writer_log.json)
"""
import sys, json, math, time
from common import *

ENTER, CONFIRM, PEAK_DROP = 0.10, 0.25, 0.05   # detection thresholds (smoothed reading)
DEDUP_PX = 60.0                                # same problem seen twice -> record once


class Writer:
    def __init__(self, supply=40, battery=16000.0):   # supply = beacons the Writer carries
        self.x, self.y, self.heading = START_POSE
        self.sources = make_sources()
        self.nav = WallFollower(+1)
        self.frame, self.t0 = 0, int(time.time())
        self.mode = "EXPLORE"                    # EXPLORE -> RETURN -> DONE
        self.beacons, self.events = [], []
        self.supply, self.battery = supply, battery
        self.path, self.path_len, self.travelled = [(self.x, self.y)], 0.0, 0.0
        self.last_beacon = (self.x, self.y)      # the entrance acts as "beacon 0"
        self.last_clear = (self.x, self.y)
        self.tracks = {t: {"ema": 0.0, "max": 0.0, "best": None, "reported": False} for t in EVENT_CATALOG}
        self.msg = "exploring"

    # ---------- motion ----------
    def move(self, v, w):
        self.heading += w
        nx, ny, ok = try_move(self.x, self.y, v * math.cos(self.heading), v * math.sin(self.heading))
        if not ok: self.nav.trigger_recovery()
        d = math.hypot(nx - self.x, ny - self.y)
        self.x, self.y = nx, ny
        self.travelled += d; self.battery -= d
        if math.hypot(self.x - self.path[-1][0], self.y - self.path[-1][1]) >= 25:
            self.path_len += math.hypot(self.x - self.path[-1][0], self.y - self.path[-1][1])
            self.path.append((self.x, self.y))

    # ---------- beacons ----------
    def drop(self, kind, etype, pos):
        dx, dy = self.last_beacon[0] - pos[0], self.last_beacon[1] - pos[1]
        bid = len(self.beacons) + 1
        ts = self.t0 + int(self.frame / FPS)
        prio = EVENT_CATALOG[etype]["priority"] if etype else 0
        raw = BeaconCodec.pack(bid, etype, prio, math.degrees(math.atan2(dy, dx)), math.hypot(dx, dy) / PX_PER_M, ts)
        b = {"id": bid, "kind": kind, "event_type": etype, "x": round(pos[0], 1), "y": round(pos[1], 1),
             "timestamp": ts, "raw_hex": raw.hex()}
        self.beacons.append(b)
        self.last_beacon = pos
        self.last_clear = pos
        return b

    def ensure_chain(self):
        """Executor drives straight between beacons -> every hop must be wall-free."""
        pos = (self.x, self.y)
        if not segment_clear(self.last_beacon, pos): self.drop("nav", 0, self.last_clear)

    def waypoints(self):
        pos = (self.x, self.y)
        if segment_clear(self.last_beacon, pos) and math.hypot(pos[0] - self.last_beacon[0], pos[1] - self.last_beacon[1]) < MAX_HOP_PX:
            self.last_clear = pos
            return
        if len(self.beacons) < self.supply: self.drop("nav", 0, self.last_clear)   # last spot that still had a clear line back

    # ---------- event detection ("decide what matters") ----------
    def sensing(self):
        reads = sense(self.x, self.y, self.sources)
        for t, (r, bearing, dist) in reads.items():
            tr = self.tracks[t]
            tr["ema"] = 0.7 * tr["ema"] + 0.3 * r
            e = tr["ema"]
            if e >= ENTER:
                if e > tr["max"]:
                    tr["max"] = e
                    tr["best"] = (self.x + dist * math.cos(bearing), self.y + dist * math.sin(bearing), r)
                elif tr["max"] >= CONFIRM and e < tr["max"] - PEAK_DROP and not tr["reported"]:
                    self.report(t, tr)               # we just passed the closest approach
            else:
                if tr["max"] >= CONFIRM and not tr["reported"]: self.report(t, tr)
                tr["max"], tr["best"], tr["reported"] = 0.0, None, False

    def report(self, t, tr):
        tr["reported"] = True
        ex, ey, conf = tr["best"]
        if any(ev["event_type"] == t and math.hypot(ev["est_x"] - ex, ev["est_y"] - ey) < DEDUP_PX for ev in self.events):
            return                                   # already recorded this one
        if len(self.beacons) >= self.supply: return
        self.ensure_chain()
        b = self.drop("event", t, (self.x, self.y))
        cat = EVENT_CATALOG[t]
        self.events.append({"beacon_id": b["id"], "event_type": t, "name": cat["name"], "priority": cat["priority"],
                            "est_x": round(ex, 1), "est_y": round(ey, 1), "confidence": round(conf, 2),
                            "timestamp": b["timestamp"]})
        self.msg = f"BEACON #{b['id']}: {cat['name']} (priority {cat['priority']})"
        print(f"[t={self.frame / FPS:6.1f}s] {self.msg}  hex={b['raw_hex']}")

    # ---------- main loop ----------
    def step(self):
        if self.mode == "DONE": return
        self.frame += 1
        if self.mode == "EXPLORE":
            v, w = self.nav.step(cast_lidar(self.x, self.y, self.heading))
            self.move(v, w)
            self.sensing()
            self.waypoints()
            loop_closed = self.travelled > 2500 and math.hypot(self.x - START_POSE[0], self.y - START_POSE[1]) < 40
            low_battery = self.battery < self.path_len * 1.3 + 200
            if loop_closed or low_battery or len(self.beacons) >= self.supply:
                self.mode, self.msg = "RETURN", "returning to entrance"
                print(f"[t={self.frame / FPS:6.1f}s] returning ({'loop closed' if loop_closed else 'battery/supply'})")
        elif self.mode == "RETURN":
            start = START_POSE[:2]
            if math.hypot(self.x - start[0], self.y - start[1]) < 10:
                self.mode, self.msg = "DONE", "mission complete - log exported"
                self.export(); return
            if segment_clear((self.x, self.y), start): tgt = start
            else:
                while len(self.path) > 1 and segment_clear((self.x, self.y), self.path[-2]): self.path.pop()
                tgt = self.path[-1]
                if math.hypot(tgt[0] - self.x, tgt[1] - self.y) < 12 and len(self.path) > 1: self.path.pop(); tgt = self.path[-1]
            err = (math.atan2(tgt[1] - self.y, tgt[0] - self.x) - self.heading + math.pi) % (2 * math.pi) - math.pi
            self.move(2.0 if abs(err) < 0.6 else 0.4, max(-0.12, min(0.12, 0.25 * err)))

    def export(self, filename="writer_log.json"):
        log = {"t0_unix": self.t0, "t_end_unix": self.t0 + int(self.frame / FPS), "px_per_m": PX_PER_M,
               "entrance_pose": {"x": START_POSE[0], "y": START_POSE[1], "theta_deg": math.degrees(START_POSE[2])},
               "events": self.events, "beacons_dropped": self.beacons}
        with open(filename, "w") as f: json.dump(log, f, indent=2)
        print(f"Saved {filename}: {len(self.beacons)} beacons, {len(self.events)} events")


def run_window(speed=1):
    import pygame, viz
    screen, font, clock = viz.init("Writer - IEEE TSYP 14 (The Living Map)")
    w, show, running = Writer(), True, True
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT or (e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE): running = False
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_g: show = not show
                speed = {pygame.K_1: 1, pygame.K_2: 4, pygame.K_3: 16}.get(e.key, speed)
        for _ in range(speed): w.step()
        viz.draw_world(screen, w.sources, show)
        viz.draw_beacons(screen, w.beacons)
        viz.draw_robot(screen, w.x, w.y, w.heading, (41, 128, 185))
        found = [f"{EVENT_CATALOG[t]['name']}" for t in sorted({ev['event_type'] for ev in w.events})]
        viz.draw_lines(screen, font, [f"WRITER  mode={w.mode}  t={w.frame / FPS:.0f}s  beacons={len(w.beacons)}/{w.supply}  speed x{speed}",
                                      f"battery={max(0, w.battery):.0f}  events found: {len(w.events)}"], 15, 4)
        viz.draw_lines(screen, font, [w.msg, "found: " + (", ".join(found) or "-")], 15, 660)
        pygame.display.flip(); clock.tick(FPS)
    pygame.quit()


if __name__ == "__main__":
    if "--headless" in sys.argv:
        w = Writer()
        while w.mode != "DONE" and w.frame < 60000: w.step()
        if w.mode != "DONE": print("did not finish"); 
    else:
        run_window()
