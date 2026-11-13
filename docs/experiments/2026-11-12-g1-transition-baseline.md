# G1 crawl transition — experiment record (2026-11-12)

**Status: work in progress; simulation only.** This record distinguishes successful execution from reliable control and physical-robot validation. Test work and measurements were reported by [@Pj2481](https://github.com/Pj2481) using this team repository and its pre-existing policies. Credit for original project code and policies remains with the upstream contributors.

## Proposed behavior

Stand → widen legs and bend knees → reach arms forward / lower torso → confirm **both hands and feet** support weight for a sustained period → switch to crawling policy.

A normal crawl-to-stand transition and a recovery maneuver from a fall are separate objectives. An uncontrolled fall must **not** be used as a routine method of entering crawl or triggering real G1 firmware recovery.

**Target (not achieved):** at least 90 successful transitions out of 100 randomized simulated starting configurations, with speed, contact, pose and failures quantified.

## Software and models

- Windows laptop: RTX 4060 GPU, Isaac Sim 4.5, Isaac Lab 2.2, a separate Python 3.11 MuJoCo environment.
- Isaac Lab `g1-crawl-start` exposes 23 actions, 75 policy observations and 81 critic observations; physics timestep is 0.005 s, control step is 0.020 s.
- MuJoCo `sim2sim_mj/scene.xml` has **21 actuated joints**, while the Isaac Lab USD configuration references a **23-joint** G1 including left and right wrist-roll. MuJoCo has no actuated fingers and uses simple hand-contact spheres. **The exact physical G1 EDU body-DOF/wrist variant still requires verification.**
- The existing MuJoCo `policy_crawl_start.pt` is **not** the trained Isaac Lab checkpoint `model_99.pt`. Their observation/action correspondence must be verified before comparing or transferring results.

## Training: headless PPO runs

| Experiment | Environment | Total timesteps | Report |
|---|---|---:|---|
| Crawl baseline | `g1-crawl`, 4 envs × 50 iterations | 4,800 | Saved `model_49.pt` |
| Transition smoke test | `g1-crawl-start`, 1 env × 2 iterations | 48 | Initialization and learning successful |
| Transition baseline | `g1-crawl-start`, 4 envs × 100 iterations | 9,600 | Completed in ~70 seconds |

Transition run command: `isaaclab.bat -p scripts/rsl_rl/train.py --task g1-crawl-start --num_envs 4 --max_iterations 100 --headless`.

Baseline checkpoint: `logs/rsl_rl/g1_crawl/2026-11-12_16-26-16/model_99.pt`.

Final reported training metrics at iteration 99:
- Mean reward **−68.97**; mean episode length **364.46** steps.
- Undesired contact reward **−5.1675**, gravity-alignment reward **−5.5466**, target-pose deviation reward **−0.4059**.
- `Episode_Termination/time_out = 1.0000`. The current transition environment only defines timeout termination; **timeout is not a success criterion**.

## Eight-environment headless policy evaluation

A separate diagnostic evaluator is included as [`scripts/rsl_rl/evaluate_g1_crawl_start.py`](../../scripts/rsl_rl/evaluate_g1_crawl_start.py). The local script was successfully run against the 100-iteration checkpoint for seven simulated seconds in eight environments, without opening Isaac Sim's graphics window.

| Measure | Observed | Provisional check |
|---|---:|---:|
| Final mean absolute joint-pose error | 1.100 rad | <0.35 rad |
| Final projected gravity +X | 0.748 | >0.85 |
| Final joint velocity RMS | 0.473 rad/s | <1.0 rad/s |
| Final base height | 0.103 m | informational only |
| Final snapshot passes | **0/8** | all three checks |
| Sustained-hold passes | **0/8** | ≥80% passing hold measurements |
| Reset/terminated during test | 0/8 | informational |

**Result:** no test instance passed the *preliminary diagnostic*. This does not prove any particular fall configuration, and the evaluator still lacks hand/foot force, clearance, slip, or full success checks. Do not describe the transition as learned or reliable.

## MuJoCo: unexpected joint-target spikes at the policy switch

User-recorded local diagnostics file: `outputs/diagnostics/sim2sim_20261112_171024.csv`. It contains 343 samples: 175 under the pre-existing shamble policy and 168 under the pre-existing crawl-start policy. No successful automatic crawl-policy handoff appears in this capture.

| Statistic | Shamble | Crawl-start |
|---|---:|---:|
| Peak recorded joint speed | 2.363 rad/s | **36.456 rad/s** |
| Samples with any joint speed >25 rad/s | 0 | **10** |
| Maximum adjacent joint-target step | 0.136 rad | **3.987 rad** |

At the transition, both hip-pitch target angles changed by about **2.237 rad** each, and the left shoulder-roll target by about **2.059 rad**. Later target oscillations reached nearly 4 rad/sample. Sudden *joint-target* changes are a serious issue independently of keyboard walking speed.

Safety-log *position-jump* checks currently use wall-clock rather than simulated time; that comparison must be corrected before treating those warnings as calibrated physical limits. The separately logged joint-velocity spikes are still meaningful simulation diagnostics.

## Contacts and rate-limiting work

A local contact-handoff prototype was installed and produced this diagnostic sample:
- Minimum registered hand force: **0.0 N**.
- Minimum registered foot force: **113.2 N**.
- Projected gravity +X: **−0.23**; root height **0.29 m**.
- Joint-pose mean error **0.29 rad**; stable contact hold **0.00 s**.
- Automatic switch to `policy_crawl.pt` **not observed**.

The user restored keyboard command caps to **2.0 m/s** forward, **1.0 m/s** lateral, **1.0 rad/s** turning and proposed smoothing rates `[2.0, 0.75, 0.75]` for commanded motion. **Velocity-command smoothing is not a joint-speed limiter.** A separate **4 rad/s joint-position-target slew** patch was prepared but not verified on the user’s local viewer; with MuJoCo's 0.008-s policy interval this would cap the requested target change to 0.032 rad/update. No claim of actual motor safety follows from that limit.

Local manual edits to `run_sim2sim_keyboard.py` later caused an `IndentationError` after inserting smoothing into the wrong scope. A repair script was prepared, but repaired execution was **not yet confirmed**. We intentionally did **not** push this broken local controller or any unverified runtime patches.

## Model and behavior work remaining

1. Restore working MuJoCo controller syntax locally, inspect `git diff`, preserve backup, then retest with separate velocity-command and joint-target ramps.
2. Confirm the physical G1's 23-vs-29-DOF model, wrist motion, and fixed-hand geometry; match MJCF and USD joint order, collision surfaces and inertias.
3. Define intermediate *wide stance* and *arms-reaching* target configurations, initially with constrained simulation tests rather than arbitrary new hardware commands.
4. Train/evaluate a phased stand-to-crawl controller; measure bilateral hand and foot load, posture, base orientation, clearance, motor state and joint speeds before switching policies.
5. Evaluate on ≥100 randomized seeds and compare before/after success rates, falls, velocity peaks and steady-contact hold.
6. Handle crawl-to-stand and fallen-state recovery separately; no unsupervised tests on the physical G1.

**No physical G1 has been validated with the experimental control changes.** Real deployment needs hardware-specific limits, a robust independent safety system, and qualified lab supervision.

**Excluded from this PR:** local checkpoints and large outputs, user-machine backups, unverified policy/motor modifications, and the diagnostic CSV. This PR includes only the experiment record and the tested-in-session preliminary headless evaluator.
