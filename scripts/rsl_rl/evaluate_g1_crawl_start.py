"""Headless diagnostic evaluation for the g1-crawl-start RSL-RL policy.

Designed for Isaac Lab 2.2 with the lez666-g1-sim2crawl project.
This does not assess real-robot safety or certify a successful recovery behavior.
"""

import argparse
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Evaluate G1 stand-to-crawl policy headlessly.")
parser.add_argument("--task", type=str, default="g1-crawl-start")
parser.add_argument("--checkpoint", type=str, required=True)
parser.add_argument("--num_envs", type=int, default=8)
parser.add_argument("--seconds", type=float, default=7.0)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
args_cli.headless = True  # Never start the Vulkan-dependent viewer.
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch
from rsl_rl.runners import OnPolicyRunner
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
from isaaclab_tasks.utils.hydra import hydra_task_config
import isaaclab_tasks  # noqa: F401 -- registers standard tasks
import g1_crawl.tasks  # noqa: F401 -- registers project tasks
from g1_crawl.tasks.manager_based.g1_crawl.mdp.rewards import _get_full_target_vector


@hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")
def main(env_cfg, agent_cfg):
    if args_cli.num_envs < 1:
        raise ValueError("--num_envs must be >= 1")
    if not (0.5 < args_cli.seconds < 7.9):
        raise ValueError("Use --seconds between 0.5 and 7.9 to avoid the 8s timeout reset")
    path = os.path.abspath(args_cli.checkpoint)
    if not os.path.isfile(path):
        raise FileNotFoundError(f"Checkpoint not found: {path}")

    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = agent_cfg.seed
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device

    raw_env = gym.make(args_cli.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(raw_env, clip_actions=agent_cfg.clip_actions)
    try:
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
        runner.load(path)
        policy = runner.get_inference_policy(device=env.unwrapped.device)

        robot = env.unwrapped.scene["robot"]
        target = _get_full_target_vector("assets/crawl-pose.json", robot)
        target = target.to(device=robot.data.joint_pos.device, dtype=robot.data.joint_pos.dtype)

        step_dt = float(env.unwrapped.step_dt)
        steps = int(args_cli.seconds / step_dt)
        hold_begin_s = min(5.0, args_cli.seconds * 0.70)
        hold_begin_step = int(hold_begin_s / step_dt)

        # Provisional diagnostic thresholds. These are not deployment criteria.
        max_pose_mae = 0.35   # rad, average absolute error across robot joints
        min_gravity_x = 0.85  # projected gravity aligned with base +X (crawl convention)
        max_joint_rms = 1.0   # rad/s, RMS joint speed for settling

        obs, _ = env.get_observations()
        hits = torch.zeros(args_cli.num_envs, device=target.device, dtype=torch.float32)
        n_hold = 0
        any_done = torch.zeros(args_cli.num_envs, device=target.device, dtype=torch.bool)
        last_pose = last_gravity = last_joint_speed = last_height = last_ok = None

        print(f"\n[TEST] Task: {args_cli.task}")
        print(f"[TEST] Checkpoint: {path}")
        print(f"[TEST] Environments: {args_cli.num_envs}, duration: {steps * step_dt:.2f} simulated seconds")
        print(f"[TEST] Hold evaluation starts at {hold_begin_s:.2f} s")
        print("[TEST] Diagnostic thresholds: pose MAE < 0.35 rad, gravity X > 0.85, joint RMS < 1.0 rad/s")

        for i in range(steps):
            with torch.inference_mode():
                actions = policy(obs)
                obs, _, dones, _ = env.step(actions)

                joint_pos = robot.data.joint_pos
                joint_vel = robot.data.joint_vel
                pose_mae = (joint_pos - target.unsqueeze(0)).abs().mean(dim=1)
                grav_x = robot.data.projected_gravity_b[:, 0]
                joint_rms = torch.sqrt(torch.mean(joint_vel.square(), dim=1))
                height = robot.data.root_pos_w[:, 2]
                ok = (pose_mae < max_pose_mae) & (grav_x > min_gravity_x) & (joint_rms < max_joint_rms)

                any_done |= dones.view(-1).to(device=any_done.device, dtype=torch.bool)
                if i >= hold_begin_step:
                    hits += ok.float()
                    n_hold += 1

                last_pose = pose_mae.detach().clone()
                last_gravity = grav_x.detach().clone()
                last_joint_speed = joint_rms.detach().clone()
                last_height = height.detach().clone()
                last_ok = ok.detach().clone()

        success_fraction = hits / max(1, n_hold)
        # Requires both a stable last observation and >=80% passing hold samples.
        success = (success_fraction >= 0.8) & last_ok & (~any_done)

        def avg(t):
            return float(t.mean().item())

        print("\n========== G1 CRAWL-START EVALUATION ==========")
        print(f"Checkpoint: {path}")
        print(f"Final pose mean absolute joint error: {avg(last_pose):.3f} rad (lower is better)")
        print(f"Final gravity +X alignment:         {avg(last_gravity):.3f} (closer to 1 is better)")
        print(f"Final joint velocity RMS:           {avg(last_joint_speed):.3f} rad/s (lower means more settled)")
        print(f"Final base height:                  {avg(last_height):.3f} m")
        print(f"Environments reset/terminated:      {int(any_done.sum().item())}/{args_cli.num_envs}")
        print(f"Passing final snapshot:             {int(last_ok.sum().item())}/{args_cli.num_envs}")
        print(f"Passing sustained-hold criterion:   {int(success.sum().item())}/{args_cli.num_envs}")
        print(f"Mean hold-pass fraction:            {avg(success_fraction):.1%}")
        print("NOTE: Preliminary diagnostic only; contact stability, falls, and real-hardware safety are NOT verified.")
        print("================================================\n")
    finally:
        env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()