"""
Crouch -> crawl -> stand-up sequencer for run_sim2sim_keyboard.py.

This file is plain Python with NO MuJoCo / torch dependency, so its logic can be
unit-tested on any laptop:   python3 -m unittest sim2sim_mj/test_crawl_sequencer.py

How it is used (see run_sim2sim_keyboard.py):
  * Press X  -> start_crawl():  load crawl_start policy, wait until the robot has
                settled low, then load the crawl policy.
  * Press Y  -> start_stand():  EXPERIMENTAL. Slowly ramp joints back to the stand
                pose. No trained crawl->stand policy ships with this repo, so this
                may fall. Use it to collect data, then train a real one (see
                docs/FUTURE_INTEGRATIONS.md).
  * Each loop iteration the main script calls update(...) and executes the returned
    command string.

Legend used in the comments below:
  [TUNE]        safe and expected to change while testing in simulation
  [CAUTION]     changes behaviour a lot; change one thing at a time and re-test
  [DO NOT EDIT] logic that other code depends on
"""

# ---------------------------------------------------------------------------
# SETTINGS
# ---------------------------------------------------------------------------
SEQ_CONFIG = {
    # Policy files (relative to sim2sim_mj/). [CAUTION] must match CONFIG["policy_cycle"].
    "crawl_start_policy": "policies/policy_crawl_start.pt",
    "crawl_policy": "policies/policy_crawl.pt",

    # [TUNE] Always wait at least this long (s) in the crawl_start policy before
    # checking whether the robot has settled.
    "min_start_s": 4.0,

    # [TUNE] Give up waiting and go to the crawl policy anyway after this long (s).
    "max_start_s": 12.0,

    # [TUNE] Robot counts as "settled low" when base height (m) is below this ...
    # (Crawl pose has the base at about 0.25 m; standing is about 0.75 m.)
    "settle_height_m": 0.40,

    # [TUNE] ... AND the average |joint velocity| (rad/s) is below this.
    "settle_joint_vel": 0.5,

    # [TUNE] Stand-up ramp speed (rad/s). Lower = gentler and slower. EXPERIMENTAL.
    "stand_ramp_rate": 0.3,

    # [TUNE] Seconds the stand-up ramp runs before the sequencer reports done.
    "stand_duration_s": 12.0,
}

IDLE, CRAWL_START, CRAWL, STAND = "idle", "crawl_start", "crawl", "stand"  # [DO NOT EDIT]


class CrawlSequencer:
    """Tiny state machine. update() returns a command string or None.

    Commands (strings the main loop understands):
        "policy:<relative path>"   load that policy and stay in policy mode
        "mode:default_pos"         ramp joints to the stand pose
        "ramp_rate:<float>"        set the I/K ramp speed (rad/s)
        "done"                     sequence finished
    """

    def __init__(self, config=None):
        self.cfg = dict(SEQ_CONFIG if config is None else config)
        self.state = IDLE
        self._t0 = 0.0
        self._announced = False

    # -- starting a sequence -------------------------------------------------
    def start_crawl(self, now):
        self.state, self._t0, self._announced = CRAWL_START, now, False
        return "policy:" + self.cfg["crawl_start_policy"]

    def start_stand(self, now):
        self.state, self._t0 = STAND, now
        return "mode:default_pos"

    def cancel(self):
        self.state = IDLE

    @property
    def active(self):
        return self.state in (CRAWL_START, STAND)

    # -- called every loop ---------------------------------------------------
    def update(self, now, base_height, mean_abs_joint_vel):
        c = self.cfg
        elapsed = now - self._t0
        if self.state == CRAWL_START:
            if elapsed < c["min_start_s"]:
                return None
            settled = (base_height < c["settle_height_m"]
                       and mean_abs_joint_vel < c["settle_joint_vel"])
            if settled or elapsed >= c["max_start_s"]:
                self.state = CRAWL
                return "policy:" + c["crawl_policy"]
        elif self.state == STAND:
            if not self._announced:
                self._announced = True
                return "ramp_rate:%.3f" % c["stand_ramp_rate"]
            if elapsed >= c["stand_duration_s"]:
                self.state = IDLE
                return "done"
        return None
