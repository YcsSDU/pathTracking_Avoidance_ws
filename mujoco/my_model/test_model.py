"""Physical regression checks: python -m unittest my_model.test_model -v."""

import unittest

import mujoco
import numpy as np

from my_model.test import (
    LIDAR_SCENE_PATH, MODEL_PATH, WHEELS, MecanumDrive, PlanarLidar, configure_camera,
)


class ModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        cls.drive = MecanumDrive(cls.model)

    def assert_healthy(self, data):
        self.assertTrue(np.isfinite(data.qpos).all())
        self.assertTrue(np.isfinite(data.qvel).all())
        self.assertEqual(sum(int(w.number) for w in data.warning), 0)
        self.assertGreater(data.qpos[2], 0.034)
        self.assertLess(data.qpos[2], 0.036)
        self.assertLess(np.linalg.norm(data.qpos[4:6]), 0.01)
        chassis = self.model.geom("base_link_geom")
        vertical_extent = np.abs(data.geom_xmat[chassis.id].reshape(3, 3)[2]) @ chassis.size
        clearance = data.geom_xpos[chassis.id, 2] - vertical_extent
        self.assertGreater(clearance, 0.019)
        self.assertLess(clearance, 0.021)

    def settle(self):
        data = mujoco.MjData(self.model)
        for _ in range(1000):
            mujoco.mj_step(self.model, data)
        return data

    def test_user_dimensions_and_mass(self):
        model = self.model
        fl = model.body("front_left_wheel_intermediate_link").pos
        fr = model.body("front_right_wheel_intermediate_link").pos
        bl = model.body("back_left_wheel_intermediate_link").pos
        self.assertAlmostEqual(fl[0] - bl[0], 0.20)
        self.assertAlmostEqual(fl[1] - fr[1], 0.25)
        np.testing.assert_allclose(model.geom("base_link_geom").size, [0.15, 0.10, 0.015])
        np.testing.assert_allclose(model.geom("base_link_geom").pos, [0, 0, 0])
        self.assertAlmostEqual(float(model.numeric("wheel_radius").data[0]), 0.025)
        self.assertAlmostEqual(model.body_mass[model.body("base_link").id], 6.0)
        for name in WHEELS:
            wheel = model.body(f"{name}_wheel_intermediate_link")
            self.assertAlmostEqual(wheel.pos[2], -0.01)
            self.assertAlmostEqual(model.body_subtreemass[wheel.id], 0.12)
        self.assertAlmostEqual(model.body_mass[model.body("lidar_link").id], 0.15)
        self.assertAlmostEqual(model.body_mass.sum(), 6.63)
        self.assertEqual(model.nu, 4)

    def test_named_and_free_cameras(self):
        camera = mujoco.MjvCamera()
        configure_camera(self.model, camera, "tracking")
        self.assertEqual(camera.type, mujoco.mjtCamera.mjCAMERA_FIXED)
        self.assertEqual(camera.fixedcamid, self.model.camera("tracking").id)
        data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, data)
        initial_position = data.cam_xpos[camera.fixedcamid].copy()
        data.qpos[0] += 0.2
        mujoco.mj_forward(self.model, data)
        np.testing.assert_allclose(
            data.cam_xpos[camera.fixedcamid] - initial_position, [0.2, 0, 0], atol=1e-8
        )
        configure_camera(self.model, camera)
        self.assertEqual(camera.type, mujoco.mjtCamera.mjCAMERA_FREE)
        self.assertEqual(camera.fixedcamid, -1)

    def test_no_initial_penetration_and_stable_rest(self):
        data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, data)
        self.assertGreaterEqual(min((c.dist for c in data.contact), default=0), 0)
        for _ in range(3000):
            self.drive.apply(data, [0, 0, 0])
            mujoco.mj_step(self.model, data)
        self.assert_healthy(data)
        self.assertLess(np.linalg.norm(data.qpos[:2]), 1e-4)
        self.assertLess(np.linalg.norm(data.qvel[:6]), 1e-4)
        self.assertGreater(data.ncon, 0)
        self.assertGreater(min(c.dist for c in data.contact), -0.0002)

    def test_body_frame_motion_and_yaw(self):
        for label, command, axis, sign in (
            ("forward", [0.04, 0, 0], 0, 1),
            ("backward", [-0.04, 0, 0], 0, -1),
            ("left", [0, 0.04, 0], 1, 1),
            ("right", [0, -0.04, 0], 1, -1),
            ("counterclockwise", [0, 0, 0.3], 2, 1),
            ("clockwise", [0, 0, -0.3], 2, -1),
        ):
            with self.subTest(motion=label):
                data = self.settle()
                initial = data.qpos[:3].copy()
                for step in range(3000):
                    self.drive.apply(data, np.asarray(command) * min(step / 500, 1))
                    mujoco.mj_step(self.model, data)
                self.assert_healthy(data)
                delta = data.qpos[:3] - initial
                q = data.qpos[3:7]
                yaw = 2 * np.arctan2(q[3], q[0])
                if axis == 2:
                    self.assertGreater(sign * yaw, 0.5)
                    self.assertLess(sign * yaw, 1.0)
                    self.assertLess(np.linalg.norm(delta[:2]), 0.004)
                else:
                    self.assertGreater(sign * delta[axis], 0.07)
                    self.assertLess(sign * delta[axis], 0.14)
                    self.assertLess(abs(delta[1 - axis]), 0.004)
                    self.assertLess(abs(yaw), 0.02)


class LidarTests(unittest.TestCase):
    def test_empty_floor_has_no_self_returns(self):
        model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        lidar = PlanarLidar(model)
        self.assertEqual(lidar.ranges(data).size, 360)
        self.assertTrue(np.isinf(lidar.ranges(data)).all())
        np.testing.assert_allclose(data.sensordata[lidar.indices], -1)
        np.testing.assert_allclose(lidar.angles, np.deg2rad(np.arange(360)), atol=1e-12)
        self.assertAlmostEqual(data.site_xpos[model.site("lidar_ray000").id, 2], 0.081)

    def test_known_distances_and_body_frame_rotation(self):
        model = mujoco.MjModel.from_xml_path(str(LIDAR_SCENE_PATH))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        lidar = PlanarLidar(model)
        ranges = lidar.ranges(data)
        np.testing.assert_allclose(ranges[[0, 90, 180, 270]], 1.175, atol=1e-10)
        self.assertAlmostEqual(ranges[30], 0.45 / np.cos(np.deg2rad(30)))
        data = mujoco.MjData(model)
        data.qpos[:2] = [0.2, -0.1]
        data.qpos[3:7] = [np.sqrt(0.5), 0, 0, np.sqrt(0.5)]
        mujoco.mj_forward(model, data)
        ranges = lidar.ranges(data)
        self.assertAlmostEqual(ranges[0], 1.275)
        self.assertAlmostEqual(ranges[270], 0.975)

    def test_blind_zone_and_maximum_range(self):
        for target_x, half_width, raw_expected in ((0.04, 0.005, 0.035), (9, 0.1, 8)):
            with self.subTest(target_x=target_x):
                spec = mujoco.MjSpec.from_file(str(MODEL_PATH))
                spec.worldbody.add_geom(
                    name="range_target", type=mujoco.mjtGeom.mjGEOM_BOX,
                    pos=[target_x, 0, 0.081], size=[half_width, 0.005, 0.005],
                )
                model = spec.compile()
                data = mujoco.MjData(model)
                mujoco.mj_forward(model, data)
                self.assertAlmostEqual(data.sensor("lidar_range000").data[0], raw_expected)
                self.assertTrue(np.isinf(PlanarLidar(model).ranges(data)[0]))

    def test_scan_updates_at_ten_hertz(self):
        spec = mujoco.MjSpec.from_file(str(MODEL_PATH))
        target = spec.worldbody.add_body(name="moving_target", mocap=True, pos=[1, 0, 0.1])
        target.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.1, 0.1, 0.1])
        model = spec.compile()
        data = mujoco.MjData(model)
        lidar = PlanarLidar(model)
        mujoco.mj_forward(model, data)
        for _ in range(20):
            mujoco.mj_step(model, data)
        data.mocap_pos[0, 0] = 2
        for _ in range(79):
            mujoco.mj_step(model, data)
        self.assertAlmostEqual(lidar.ranges(data)[0], 0.9, places=4)
        for _ in range(3):
            mujoco.mj_step(model, data)
        self.assertAlmostEqual(lidar.ranges(data)[0], 1.9, places=4)


if __name__ == "__main__":
    unittest.main()
