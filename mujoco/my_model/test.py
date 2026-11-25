"""Run the custom mecanum model, with a viewer or headless wheel-speed control.

Examples (activate the environment containing mujoco first):
    python my_model/test.py
    python my_model/test.py --camera tracking
    python my_model/test.py --scene lidar --show-lidar
    python my_model/test.py --headless --duration 4 --vy 0.04
    python my_model/test.py --vx 0.04 --wz 0.2
"""

import argparse
from pathlib import Path
import time

import mujoco
import numpy as np


MODEL_PATH = Path(__file__).resolve().parent / "robots" / "mecanum.xml"
LIDAR_SCENE_PATH = MODEL_PATH.with_name("lidar_scene.xml")
WHEELS = ("front_left", "front_right", "back_left", "back_right")


class PlanarLidar:
    """Read the native rangefinder array; angles increase from +X toward +Y."""

    def __init__(self, model):
        sensor_ids = [
            index for index in range(model.nsensor)
            if model.sensor(index).name.startswith("lidar_range")
        ]
        self.indices = model.sensor_adr[sensor_ids].copy()
        self.range_min = float(model.numeric("lidar_min_range").data[0])
        self.range_max = float(model.sensor_cutoff[sensor_ids[0]])
        self.scan_period = float(model.sensor_interval[sensor_ids[0], 0])
        angles = []
        for sensor_id in sensor_ids:
            rotation = np.empty(9)
            site_id = model.sensor_objid[sensor_id]
            mujoco.mju_quat2Mat(rotation, model.site_quat[site_id])
            direction = rotation.reshape(3, 3)[:, 2]
            angles.append(np.arctan2(direction[1], direction[0]) % (2 * np.pi))
        self.angles = np.asarray(angles)

    def ranges(self, data):
        """Return metres; infinity means no valid return within the usable range."""
        ranges = data.sensordata[self.indices].copy()
        invalid = (~np.isfinite(ranges)) | (ranges < self.range_min) | (ranges >= self.range_max)
        ranges[invalid] = np.inf
        return ranges


class MecanumDrive:
    """Body-frame twist to wheel torque. Wheel arrays are always FL, FR, BL, BR."""

    def __init__(self, model):
        self.model = model
        self.velocity_indices = np.array(
            [model.jnt_dofadr[model.joint(f"{name}_wheel_rolling_joint").id]
             for name in WHEELS]
        )
        self.actuator_indices = np.array(
            [model.actuator(f"{name}_motor").id for name in WHEELS]
        )
        radius = float(model.numeric("wheel_radius").data[0])
        rows = []
        # X-layout: passive rolling releases motion perpendicular to each roller axis.
        for name, lateral_sign in zip(WHEELS, (-1, 1, 1, -1)):
            x, y, _ = model.body(f"{name}_wheel_intermediate_link").pos
            rows.append([1, lateral_sign, -y + lateral_sign * x])
        self.inverse_kinematics = np.asarray(rows) / radius
        self.torque_limits = model.actuator_ctrlrange[self.actuator_indices]
        self.gain = 0.05

    def apply(self, data, twist):
        """twist = (vx m/s forward, vy m/s left, wz rad/s counterclockwise)."""
        target = self.inverse_kinematics @ np.asarray(twist, dtype=float)
        # Preserve the twist direction when a command exceeds the wheel speed limit.
        largest = np.max(np.abs(target))
        if largest > 20:
            target *= 20 / largest
        measured = data.qvel[self.velocity_indices]
        torque = self.gain * (target - measured)
        data.ctrl[self.actuator_indices] = np.clip(
            torque, self.torque_limits[:, 0], self.torque_limits[:, 1]
        )
        return target


def command_at(sim_time, fixed_twist=None):
    """One-second settling period, then 0.5 s ramps between commands."""
    if sim_time < 1:
        return np.zeros(3)
    elapsed = sim_time - 1
    if fixed_twist is not None:
        return np.asarray(fixed_twist) * min(elapsed / 0.5, 1.0)
    stages = np.array([
        [0.04, 0, 0],
        [0, 0.04, 0],
        [-0.04, 0, 0],
        [0, -0.04, 0],
        [0, 0, 0.3],
        [0, 0, -0.3],
        [0, 0, 0],
    ])
    stage = int(elapsed // 3)
    if stage >= len(stages):
        return np.zeros(3)
    previous = stages[stage - 1] if stage else np.zeros(3)
    blend = min((elapsed - stage * 3) / 0.5, 1.0)
    return previous + blend * (stages[stage] - previous)


def configure_camera(model, camera, camera_name=None):
    """Select an XML camera by name, or initialize an interactive free camera."""
    if camera_name is not None:
        camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
        camera.fixedcamid = model.camera(camera_name).id
    else:
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.fixedcamid = -1
        camera.lookat[:] = model.stat.center
        camera.distance = 0.60
        camera.azimuth = 135
        camera.elevation = -25


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="run without a window")
    parser.add_argument("--duration", type=float, default=23.0, help="simulation seconds")
    parser.add_argument("--camera", help="named XML camera to display, e.g. tracking")
    parser.add_argument("--scene", choices=("robot", "lidar"), default="robot", help="empty floor or lidar obstacle arena")
    parser.add_argument("--show-lidar", action="store_true", help="draw native rangefinder rays in the viewer")
    parser.add_argument("--scan-output", type=Path, help="save the final scan as an NPZ file")
    parser.add_argument("--vx", type=float, help="forward velocity in m/s")
    parser.add_argument("--vy", type=float, help="leftward velocity in m/s")
    parser.add_argument("--wz", type=float, help="counterclockwise yaw rate in rad/s")
    args = parser.parse_args()
    values = (args.vx, args.vy, args.wz)
    if not np.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration must be finite and positive")
    if any(v is not None and not np.isfinite(v) for v in values):
        parser.error("velocity commands must be finite")
    fixed = None if all(v is None for v in values) else [0 if v is None else v for v in values]

    model_path = LIDAR_SCENE_PATH if args.scene == "lidar" else MODEL_PATH
    model = mujoco.MjModel.from_xml_path(str(model_path))
    if args.camera is not None:
        try:
            model.camera(args.camera)
        except KeyError:
            parser.error(f"unknown camera: {args.camera}")
    data = mujoco.MjData(model)
    drive = MecanumDrive(model)
    lidar = PlanarLidar(model)
    mujoco.mj_forward(model, data)
    steps = int(np.ceil(args.duration / model.opt.timestep))
    print(f"MuJoCo {mujoco.__version__}; total mass {model.body_mass.sum():.3f} kg")
    print("Wheel order: FL FR BL BR; body axes: +X forward, +Y left, +Z up")

    def step():
        drive.apply(data, command_at(data.time, fixed))
        mujoco.mj_step(model, data)

    if args.headless:
        for _ in range(steps):
            step()
    else:
        from mujoco import viewer as mj_viewer

        with mj_viewer.launch_passive(model, data) as viewer:
            with viewer.lock():
                configure_camera(model, viewer.cam, args.camera)
                if args.scene == "lidar" and args.camera is None:
                    viewer.cam.distance = 3.2
                    viewer.cam.elevation = -55
                viewer.opt.geomgroup[3] = 0
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = args.show_lidar
            deadline = time.perf_counter()
            for _ in range(steps):
                if not viewer.is_running():
                    break
                step()
                viewer.sync()
                deadline += model.opt.timestep
                remaining = deadline - time.perf_counter()
                if remaining > 0:
                    time.sleep(remaining)
                elif remaining < -0.1:
                    deadline = time.perf_counter()

    print(f"Simulation time: {data.time:.3f} s")
    print("base_link position:", np.round(data.qpos[:3], 6))
    print("Wheel speeds (rad/s):", np.round(data.qvel[drive.velocity_indices], 4))
    ranges = lidar.ranges(data)
    valid = np.isfinite(ranges)
    nearest = f"{ranges[valid].min():.3f} m" if valid.any() else "no return"
    print(f"Lidar: {valid.sum()}/{ranges.size} valid returns; nearest: {nearest}")
    if args.scan_output is not None:
        np.savez(
            args.scan_output,
            angles=lidar.angles,
            ranges=ranges,
            range_min=lidar.range_min,
            range_max=lidar.range_max,
            scan_period=lidar.scan_period,
        )
        print(f"Scan saved: {args.scan_output}")
    warnings = sum(int(w.number) for w in data.warning)
    if warnings or not np.isfinite(data.qpos).all():
        raise RuntimeError(f"Simulation failed: {warnings} MuJoCo warnings")


if __name__ == "__main__":
    main()
