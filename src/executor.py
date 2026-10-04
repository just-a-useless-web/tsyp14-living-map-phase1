"""
EXECUTOR robot - gets a briefing, follows the beacon chain (no GPS), trusts beacons by age,
and gives first aid / guidance at every problem the Writer recorded (highest priority first).
Run:      python executor.py [writer_log.json]
Keys:     A = fast-forward 40 min (beacons become STALE)   M = destroy a beacon ahead (missing beacon)
          X = corrupt next radio packet (CRC failure)      C = hazards disappear (beacons contradicted -> SUSPECT)
          G = show/hide hidden sources     1/2/3 = speed x1/x4/x16     ESC = quit
Headless: python executor.py --headless
"""
import sys, json, math, random
from common import *


def make_briefing(log):
    """Stand-in for the Outside Network Area + command post (Member B replaces this with the real ONA):
    it tells the Executor WHAT to do and in which order, never WHERE (the beacons are the map)."""
    targets = [{"beacon_id": e["beacon_id"], "event_type": e["event_type"], "priority": e["priority"]}
               for e in log["events"]]
    targets.sort(key=lambda t: -t["priority"])
    return {"mission": "first-aid support", "targets": targets}


class Executor:
    WAIT_LIMIT = 4 * FPS            # wait 4 s for a missing packet, then go blind

    def __init__(self, log_path="writer_log.json", entry_delay=60):
        log = json.load(open(log_path))
        self.beacons = {b["id"]: b for b in log["beacons_dropped"]}
        ep = log["entrance_pose"]
        self.x, self.y, self.heading = ep["x"], ep["y"], math.radians(ep["theta_deg"])
        self.sources = make_sources()
        self.clock0, self.time_skip, self.tick = log["t_end_unix"] + entry_delay, 0, 0
        self.briefing = make_briefing(log)
        self.todo = [dict(t, status="PENDING") for t in self.briefing["targets"]]
        self.pkt, self.close, self.dead, self.corrupt_next = {}, {}, set(), False
        self.cur, self.goal, self.goal_t, self.state = 0, None, None, "PLAN"
        self.wp, self.next_node, self.stuck, self.move_t, self.hop_wait = None, None, 0, 0, 0
        self.est = (self.x, self.y)       # dead-reckoned position of the current node (from packets only)
        self.local_path, self.act_i, self.act_t = [], 0, 0
        self.verify_t, self.verify_sum, self.lost_t = 0, 0.0, 0
        self.nav, self.blind_dir, self.label = None, 1, "-"
        self.logs, self.announced, self.crc_errors = [], None, 0

    # ---------- helpers ----------
    def now(self): return self.clock0 + self.tick / FPS + self.time_skip

    def say(self, msg):
        print(f"[t={self.tick / FPS:6.1f}s] {msg}")
        self.logs.append(msg)

    def drive(self, target, speed, tol):
        dx, dy = target[0] - self.x, target[1] - self.y
        dist = math.hypot(dx, dy)
        if dist <= tol: self.stuck = 0; return True
        self.heading = math.atan2(dy, dx)
        s = min(speed, dist)
        self.x, self.y, ok = try_move(self.x, self.y, s * math.cos(self.heading), s * math.sin(self.heading))
        self.stuck = 0 if ok else self.stuck + 1
        return False

    # ---------- radio (BLE model) ----------
    def listen(self):
        self.close = {}
        for bid, b in self.beacons.items():
            if bid in self.dead or (self.tick + bid * 7) % 30: continue     # advertises every 0.5 s
            d = math.hypot(b["x"] - self.x, b["y"] - self.y)
            rng = BLE_RANGE_PX if d < 60 or los_clear((self.x, self.y), (b["x"], b["y"])) else BLE_RANGE_PX * 0.4
            if d > rng or random.random() > 0.92: continue                   # range limit + 8% packet loss
            raw = bytearray.fromhex(b["raw_hex"])
            if self.corrupt_next: raw[3] ^= 0x10; self.corrupt_next = False
            try: p = BeaconCodec.unpack(bytes(raw))
            except ValueError as e:
                self.crc_errors += 1
                self.say(f"packet from beacon #{bid} REJECTED ({e}) - waiting for next advertisement")
                continue
            self.pkt[bid] = p
            self.close[bid] = d + random.gauss(0, 4)                          # RSSI-style distance hint

    def hop_vec(self, cur, n):
        """Vector from node cur to node n, computed ONLY from decoded packets."""
        if n > cur: p, ang = self.pkt.get(n), None          # n's packet points back to cur -> reverse it
        else: p, ang = self.pkt.get(cur), None              # cur's packet points back to n
        if not p: return None
        ang = math.radians(p["heading_deg"] + (180.0 if n > cur else 0.0))
        d = p["distance_m"] * PX_PER_M
        return d * math.cos(ang), d * math.sin(ang)

    # ---------- state machine ----------
    def step(self):
        if self.state == "DONE": return
        self.tick += 1
        self.listen()
        getattr(self, "st_" + self.state.lower())()

    def st_plan(self):
        pend = [t for t in self.todo if t["status"] == "PENDING"]
        pend.sort(key=lambda t: (-t["priority"], abs(t["beacon_id"] - self.cur)))     # triage
        self.goal_t = pend[0] if pend else None
        self.goal = self.goal_t["beacon_id"] if pend else 0
        if self.announced != self.goal:
            self.announced = self.goal
            self.say(f"NEXT: {EVENT_CATALOG[self.goal_t['event_type']]['name']} (beacon #{self.goal}, priority {self.goal_t['priority']})"
                     if pend else "all targets handled - heading back to the entrance")
        if self.cur == self.goal:
            if self.goal_t: self.begin_event()
            else:
                self.state = "DONE"
                self.say("MISSION COMPLETE. Summary: " + "; ".join(
                    f"#{t['beacon_id']} {EVENT_CATALOG[t['event_type']]['name']}={t['status']}" for t in self.todo))
        else:
            self.hop_wait, self.state = 0, "HOP"

    def st_hop(self):
        d = 1 if self.goal > self.cur else -1
        v = self.hop_vec(self.cur, self.cur + d)
        if v is None:
            self.hop_wait += 1
            if self.hop_wait > self.WAIT_LIMIT: self.enter_blind(d)
            return
        self.wp, self.next_node, self.move_t, self.state = (self.est[0] + v[0], self.est[1] + v[1]), self.cur + d, 0, "MOVE"

    def st_move(self):
        self.move_t += 1
        if self.drive(self.wp, 2.2, 1.5):
            self.cur, self.est = self.next_node, self.wp
            self.arrive()
            self.state = "PLAN"
        elif self.stuck > 60 or self.move_t > 900:
            self.say("path blocked - switching to blind wall-following")
            self.enter_blind(1 if self.goal > self.cur else -1)

    def arrive(self):
        p = self.pkt.get(self.cur)
        if p and p["event_type"] in EVENT_CATALOG:
            age = self.now() - p["timestamp"]
            tgt = self.goal_t and self.goal_t["beacon_id"] == self.cur
            self.say(f"at beacon #{self.cur}: {EVENT_CATALOG[p['event_type']]['name']}, age {age:.0f}s ({age_state(age)})"
                     + ("" if tgt else " - not the current target, passing by"))

    def begin_event(self):
        p = self.pkt.get(self.cur)
        if p is None and self.hop_wait < 120: self.hop_wait += 1; return
        age = self.now() - p["timestamp"] if p else AGING_S + 1
        self.label = age_state(age)
        self.say(f"BEACON #{self.cur} trust level: {self.label} (age {age:.0f}s) -> "
                 + ("trust it, quick sensor check" if self.label == "FRESH" else "verify with own sensors before committing"))
        self.verify_t, self.verify_sum, self.state = 0, 0.0, "VERIFY"

    def set_status(self, status, why):
        self.goal_t["status"] = status
        self.say(f"beacon #{self.goal_t['beacon_id']} -> {status}: {why}")

    def st_verify(self):
        et = self.goal_t["event_type"]
        self.verify_t += 1
        self.verify_sum += sense(self.x, self.y, self.sources)[et][0]
        if self.verify_t >= (20 if self.label == "FRESH" else 60):
            avg = self.verify_sum / self.verify_t
            if avg < (0.05 if self.label == "FRESH" else 0.12):
                self.set_status("SUSPECT", f"live sensor reads {avg:.2f}, contradicts the beacon - discarded")
                self.state = "PLAN"
            else:
                self.say(f"live sensor confirms ({avg:.2f}) - approaching")
                self.local_path, self.lost_t, self.state = [(self.x, self.y)], 0, "HOMING"

    def st_homing(self):
        et = self.goal_t["event_type"]
        r, bearing, d = sense(self.x, self.y, self.sources)[et]
        if r < 0.03:
            self.lost_t += 1
            if self.lost_t > 90:
                self.set_status("SUSPECT", "signal lost while approaching"); self.state = "BACK"
            return
        self.lost_t = 0
        if d <= EVENT_CATALOG[et]["approach_px"]:
            self.act_i, self.act_t, self.state = 0, 0, "ACTION"
            self.say("target reached - starting first-aid procedure")
            return
        self.heading = bearing
        self.x, self.y, ok = try_move(self.x, self.y, 1.6 * math.cos(bearing), 1.6 * math.sin(bearing))
        if math.hypot(self.x - self.local_path[-1][0], self.y - self.local_path[-1][1]) > 20:
            self.local_path.append((self.x, self.y))

    def st_action(self):
        cat = EVENT_CATALOG[self.goal_t["event_type"]]
        steps = cat["steps"]
        if self.act_t == 0: self.say(f"   >> {steps[self.act_i][0]}")
        self.act_t += 1
        if self.act_t >= steps[self.act_i][1] * FPS:
            self.act_i, self.act_t = self.act_i + 1, 0
            if self.act_i >= len(steps):
                if cat["neutralize"]:
                    cand = [s for s in self.sources if s["type"] == self.goal_t["event_type"] and s["active"]]
                    if cand: min(cand, key=lambda s: math.hypot(s["pos"][0] - self.x, s["pos"][1] - self.y))["active"] = False
                self.set_status("DONE", "procedure finished" + (" - hazard neutralised" if cat["neutralize"] else ""))
                self.state = "BACK"

    def st_back(self):
        if not self.local_path: self.state = "PLAN"; return
        if self.drive(self.local_path[-1], 2.0, 3): self.local_path.pop()

    # ---------- missing-beacon recovery ----------
    def enter_blind(self, d):
        self.say(f"no usable packet toward beacon #{self.cur + d}: BLIND SEARCH (wall-following until another beacon is heard)")
        self.nav, self.blind_dir, self.state = WallFollower(+1 if d > 0 else -1), d, "BLIND"

    def st_blind(self):
        d = self.blind_dir
        v, w = self.nav.step(cast_lidar(self.x, self.y, self.heading))
        self.heading += w
        self.x, self.y, ok = try_move(self.x, self.y, v * math.cos(self.heading), v * math.sin(self.heading))
        if not ok: self.nav.trigger_recovery()
        cands = [i for i, dd in self.close.items() if 1 <= (i - self.cur) * d <= 4 and dd < 30]   # only nodes near where we should be
        if cands:
            new = min(cands, key=lambda i: (i - self.cur) * d)
            for t in self.todo:
                if t["status"] == "PENDING" and (t["beacon_id"] - self.cur) * d >= 1 and (t["beacon_id"] - new) * d < 0:
                    t["tries"] = t.get("tries", 0) + 1
                    if t["tries"] >= 2: t["status"] = "MISSED"; self.say(f"beacon #{t['beacon_id']} unreachable -> MISSED")
                    else: self.say(f"passed beacon #{t['beacon_id']} without hearing it - will go back and check")
            self.say(f"re-synchronised on beacon #{new}")
            self.cur, self.est, self.state = new, (self.x, self.y), "PLAN"

    # ---------- failure injection ----------
    def kill_ahead(self):
        d = 1 if (self.goal or 0) > self.cur else -1
        n = self.cur + 2 * d if 0 < self.cur + 2 * d <= max(self.beacons) else self.cur + d
        if n in self.beacons:
            self.dead.add(n); self.pkt.pop(n, None)
            self.say(f"[FAILURE] beacon #{n} destroyed")

    def contradict(self):
        for s in self.sources: s["active"] = False
        self.say("[FAILURE] all hazards/victims gone - live sensors now contradict the beacons")


def run_window(path, speed=1):
    import pygame, viz
    screen, font, clock = viz.init("Executor - IEEE TSYP 14 (The Living Map)")
    ex, show, running = Executor(path), True, True
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT or (e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE): running = False
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_a: ex.time_skip += 2400; ex.say("[FAILURE] +40 min: beacons are now STALE")
                elif e.key == pygame.K_m: ex.kill_ahead()
                elif e.key == pygame.K_x: ex.corrupt_next = True; ex.say("[FAILURE] next packet will be corrupted")
                elif e.key == pygame.K_c: ex.contradict()
                elif e.key == pygame.K_g: show = not show
                speed = {pygame.K_1: 1, pygame.K_2: 4, pygame.K_3: 16}.get(e.key, speed)
        for _ in range(speed): ex.step()
        viz.draw_world(screen, ex.sources, show)
        viz.draw_beacons(screen, list(ex.beacons.values()), ex.dead, ex.goal)
        viz.draw_robot(screen, ex.x, ex.y, ex.heading, (39, 174, 96))
        status = "  ".join(f"#{t['beacon_id']}:{t['status']}" for t in ex.todo)
        viz.draw_lines(screen, font, [f"EXECUTOR state={ex.state} node={ex.cur} goal={ex.goal} trust={ex.label} speed x{speed}",
                                      "A:+40min  M:kill beacon  X:corrupt packet  C:contradict  G:sources  1/2/3:speed", ], 15, 4)
        viz.draw_lines(screen, font, ["targets: " + status] + ex.logs[-1:], 15, 660)
        if ex.state == "ACTION":
            st = EVENT_CATALOG[ex.goal_t["event_type"]]["steps"][ex.act_i]
            pygame.draw.rect(screen, (60, 60, 60), (int(ex.x) - 40, int(ex.y) - 34, 80, 8))
            pygame.draw.rect(screen, (46, 204, 113), (int(ex.x) - 40, int(ex.y) - 34, int(80 * ex.act_t / (st[1] * FPS)), 8))
        pygame.display.flip(); clock.tick(FPS)
    pygame.quit()


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    path = args[0] if args else "writer_log.json"
    if "--headless" in sys.argv:
        ex = Executor(path)
        while ex.state != "DONE" and ex.tick < 80000: ex.step()
        print("finished" if ex.state == "DONE" else "DID NOT FINISH", "ticks", ex.tick)
    else:
        run_window(path)
