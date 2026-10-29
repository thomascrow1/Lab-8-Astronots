"""Headless checks: python -m unittest discover -s sim -p 'test_*.py' -v.

Fan torques in these tests are SYNTHETIC inputs, not LD3007MS measurements.
The dynamics tests include motor lag, stall handling, and a separate fan plant.
"""

from dataclasses import replace
import math
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

import numpy as np

from sun_tracker_sim import (
    DEFAULT_KD, DEFAULT_KP, DT_CONTROL, FanDesaturationConfig,
    FanDesaturationState, I_WHEEL, POINTING_DEADBAND_DEG, RPM_PER_PWM,
    RPM_PER_RAD_S, WHEEL_MAX_ACCEL_RPM_S, fan_config_valid,
    fan_desaturation_step, fan_equivalent_torque_nm, simulate, tracker_step,
)


def synthetic_config(**changes):
    return replace(FanDesaturationConfig(enabled=True, positive_torque_nm=0.002,
                                        negative_torque_nm=0.002), **changes)


class SupervisorTests(unittest.TestCase):
    def step(self, rpm=330, error=0, rate=0, dt=DT_CONTROL, permit=True, command=0):
        return fan_desaturation_step(rpm, error, rate, dt, permit, self.config, self.state, command)

    def setUp(self):
        self.config, self.state = synthetic_config(), FanDesaturationState()

    def test_disabled_defaults_never_command_a_fan(self):
        self.config = FanDesaturationConfig()
        self.state.bank, self.state.fault = 1, True
        self.assertEqual(self.step(rpm=math.nan), (0, 0, False))
        self.assertEqual(self.state, FanDesaturationState())

    def test_both_momentum_signs_and_hysteresis(self):
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                self.state = FanDesaturationState()
                self.assertEqual(self.step(rpm=sign * 199)[0], 0)
                bank, torque, fault = self.step(rpm=sign * 200)
                self.assertEqual(bank, -sign)
                self.assertLess(torque * sign, 0)
                self.assertFalse(fault)
                self.assertEqual(self.step(rpm=sign * 150)[0], -sign)
                self.assertEqual(self.step(rpm=sign * 100)[0], 0)
                self.assertEqual(self.step(rpm=sign * 150)[0], 0)

    def test_crossing_target_stops_before_reversing(self):
        self.step(rpm=330)
        self.assertEqual(self.step(rpm=-330)[0], 0)
        self.assertEqual(self.step(rpm=-330)[0], 1)

    def test_nonzero_target_uses_distance_from_target(self):
        self.config.target_rpm = 20
        self.assertEqual(self.step(rpm=219)[0], 0)
        self.assertEqual(self.step(rpm=220)[0], -1)
        self.assertEqual(self.step(rpm=120)[0], 0)

    def test_pointing_start_stop_bands_and_rate_gate(self):
        self.config.unload_during_slews = False
        self.assertEqual(self.step(error=6)[0], 0)
        self.assertEqual(self.step(error=5)[0], -1)
        self.assertEqual(self.step(error=9)[0], -1)
        self.assertEqual(self.step(error=11)[0], 0)
        self.assertEqual(self.step(error=6)[0], 0)
        self.assertEqual(self.step(rate=3.01)[0], 0)
        self.assertEqual(self.step(rate=3)[0], -1)
        self.assertEqual(self.step(rate=-3.01)[0], 0)

    def test_slew_unloading_bypasses_settled_pointing_and_rate_gates(self):
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                self.state = FanDesaturationState()
                self.assertEqual(self.step(rpm=sign * 220, error=90, rate=75)[0], -sign)
                self.assertEqual(self.step(rpm=sign * 160, error=20, rate=-60)[0], -sign)
                self.assertEqual(self.step(rpm=sign * 160, error=9, rate=60)[0], 0)
                self.assertEqual(self.step(rpm=sign * 220, error=9, rate=0)[0], 0)
                self.assertEqual(self.step(rpm=sign * 220, error=5, rate=0)[0], -sign)

    def test_command_trigger_requires_real_same_direction_momentum(self):
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                for rpm, command, expected in [(150 * sign, 299 * sign, 0),
                        (150 * sign, -300 * sign, 0), (0, 300 * sign, 0),
                        (100 * sign, 300 * sign, 0), (149 * sign, 300 * sign, 0),
                        (150 * sign, 300 * sign, -sign)]:
                    self.state = FanDesaturationState()
                    self.assertEqual(self.step(rpm=rpm, command=command)[0], expected)
                self.state = FanDesaturationState()
                self.assertEqual(self.step(rpm=150 * sign, command=math.nan)[0], 0)
                self.assertEqual(self.step(rpm=220 * sign, command=math.nan)[0], -sign)

    def test_command_trigger_hysteresis_does_not_restart_just_above_stop(self):
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                self.state = FanDesaturationState()
                self.assertEqual(self.step(rpm=150 * sign, command=400 * sign)[0], -sign)
                self.assertEqual(self.step(rpm=101 * sign, command=400 * sign)[0], -sign)
                self.assertEqual(self.step(rpm=100 * sign, command=400 * sign)[0], 0)
                self.assertEqual(self.step(rpm=101 * sign, command=400 * sign)[0], 0)
                self.assertEqual(self.step(rpm=149 * sign, command=400 * sign)[0], 0)
                self.assertEqual(self.step(rpm=150 * sign, command=400 * sign)[0], -sign)

    def test_loss_of_permission_switches_off_and_models_coast_down(self):
        _, before, _ = self.step()
        bank, coast, fault = self.step(permit=False)
        self.assertEqual(bank, 0)
        self.assertFalse(fault)
        self.assertLess(coast, 0)
        self.assertLess(abs(coast), abs(before))
        for _ in range(100):
            _, coast, _ = self.step(permit=False)
        self.assertLess(abs(coast), 1e-8)

    def test_runtime_limit_latches_until_state_is_reset(self):
        self.config.max_on_s = 0.12
        self.step()
        self.step()
        self.assertTrue(self.step()[2])
        self.assertEqual(self.step(rpm=-330)[0], 0)
        self.state = FanDesaturationState()
        self.assertEqual(self.step(rpm=-330)[0], 1)

    def test_invalid_configuration_cannot_start(self):
        invalid = [dict(positive_torque_nm=0), dict(negative_torque_nm=-1),
                   dict(stop_rpm=200), dict(start_error_deg=11),
                   dict(max_on_s=0), dict(torque_time_constant_s=-1),
                   dict(max_dt_s=math.inf), dict(target_rpm=math.nan),
                   dict(command_start_rpm=100), dict(command_start_rpm=math.nan)]
        for changes in invalid:
            with self.subTest(changes=changes):
                self.config, self.state = synthetic_config(**changes), FanDesaturationState()
                self.assertFalse(fan_config_valid(self.config))
                self.assertEqual(self.step(), (0, 0, True))

    def test_invalid_feedback_or_delayed_update_latches_off(self):
        for changes in [dict(rpm=math.nan), dict(error=math.inf), dict(rate=math.nan),
                        dict(dt=0), dict(dt=-0.1), dict(dt=0.21), dict(dt=math.nan)]:
            with self.subTest(changes=changes):
                self.state = FanDesaturationState()
                _, before, _ = self.step()
                expected = before
                if changes.get('dt') == 0.21:
                    expected = -0.002 + (before + 0.002) * math.exp(-0.21 / 0.5)
                bank, residual, fault = self.step(**changes)
                self.assertEqual(bank, 0)
                self.assertTrue(fault)
                self.assertAlmostEqual(residual, expected)
                bank, coast, fault = self.step()
                self.assertEqual(bank, 0)
                self.assertTrue(fault)
                self.assertLess(abs(coast), abs(residual))
                self.assertLess(coast, 0)

    def test_fully_spun_up_fans_coast_after_timing_fault_or_timeout(self):
        for cause in ('timing', 'timeout', 'invalid_config'):
            with self.subTest(cause=cause):
                self.config, self.state = synthetic_config(), FanDesaturationState()
                for _ in range(100):
                    _, before, _ = self.step()
                self.assertGreater(abs(before), 0.00199)
                if cause == 'timing':
                    bank, coast, fault = self.step(dt=0.21)
                    self.assertAlmostEqual(coast, -0.002 + (before + 0.002) * math.exp(-0.21 / 0.5))
                elif cause == 'invalid_config':
                    self.config.positive_torque_nm = 0
                    bank, coast, fault = self.step()
                    self.assertEqual(coast, before)
                    self.config = synthetic_config()
                else:
                    self.config.max_on_s = 5.01
                    bank, coast, fault = self.step()
                    self.assertAlmostEqual(coast, before * math.exp(-DT_CONTROL / 0.5))
                self.assertEqual(bank, 0)
                self.assertTrue(fault)
                self.assertGreater(abs(coast), 0.0018)
                bank, decayed, fault = self.step()
                self.assertEqual(bank, 0)
                self.assertTrue(fault)
                self.assertLess(abs(decayed), abs(coast))

    def test_delayed_startup_accounts_for_thrust_built_during_energized_loop_gap(self):
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                self.state = FanDesaturationState()
                _, initial, _ = self.step(rpm=sign * 330)
                self.assertAlmostEqual(abs(initial) / 0.002, 1 - math.exp(-0.05 / 0.5))
                bank, current, fault = self.step(rpm=sign * 330, dt=1.0)
                expected = -sign * 0.002 * (1 - math.exp(-1.05 / 0.5))
                self.assertEqual(bank, 0)
                self.assertTrue(fault)
                self.assertAlmostEqual(current, expected)
                self.assertGreater(abs(current), abs(initial) * 9)
                bank, coast, fault = self.step(rpm=sign * 330)
                self.assertEqual(bank, 0)
                self.assertTrue(fault)
                self.assertAlmostEqual(coast, current * math.exp(-0.05 / 0.5))


class TorqueAndDynamicsTests(unittest.TestCase):
    def test_empirical_rpm_calibration_unloads_with_incorrect_configured_inertia(self):
        # An ideal speed actuator isolates the calibration's unit cancellation.
        # The real inertia differs from every configured value below. Empirical
        # full-ON wheel response is known from ON-minus-OFF measured RPM slopes.
        real_wheel_inertia = 0.002
        effect_rpm_s = 4.0
        actual_bank_torque = real_wheel_inertia * effect_rpm_s / RPM_PER_RAD_S
        for configured_inertia in (0.001, 0.0035, 0.009):
            for sign in (-1, 1):
                with self.subTest(configured_inertia=configured_inertia, sign=sign):
                    equivalent = fan_equivalent_torque_nm(effect_rpm_s, configured_inertia)
                    config = synthetic_config(positive_torque_nm=equivalent,
                                              negative_torque_nm=equivalent,
                                              torque_time_constant_s=0)
                    state = FanDesaturationState()
                    wheel, rate, angle = sign * 250., 0., 0.
                    for _ in range(900):
                        bank, estimate, fault = fan_desaturation_step(
                            wheel, -angle, math.degrees(rate), DT_CONTROL, True,
                            config, state, wheel)
                        _, commanded, saturated = tracker_step(
                            -angle, math.degrees(rate), DT_CONTROL,
                            DEFAULT_KP, DEFAULT_KD, POINTING_DEADBAND_DEG,
                            wheel, external_torque_nm=estimate, I_wheel=configured_inertia)
                        actual_wheel_accel = (commanded - wheel) / DT_CONTROL / RPM_PER_RAD_S
                        body_torque = bank * actual_bank_torque - real_wheel_inertia * actual_wheel_accel
                        rate += body_torque / 0.014 * DT_CONTROL
                        angle += math.degrees(rate) * DT_CONTROL
                        wheel = commanded
                        self.assertFalse(fault)
                        self.assertFalse(saturated)
                    self.assertGreater(abs(wheel), 99)
                    self.assertLess(abs(wheel), 101)
                    self.assertLess(abs(angle), 1e-5)

    def test_tangential_pairs_cancel_force_and_pitch_radial_fans_have_no_yaw(self):
        for z in (-0.02, 0.02):
            for direction in (-1, 1):
                force_py = np.array([-direction * 0.01, 0, 0])
                force_my = -force_py
                torque = (np.cross([0, 0.32, z], force_py) +
                          np.cross([0, -0.32, z], force_my))
                np.testing.assert_allclose(force_py + force_my, 0)
                np.testing.assert_allclose(torque, [0, 0, direction * 0.0064], atol=1e-15)
            radial = np.cross([0, 0.32, z], [0, 0.01, 0])
            self.assertEqual(radial[2], 0)

    def test_feedforward_preserves_requested_body_torque(self):
        for error, rate in [(0, 0), (5, 1), (-5, -1)]:
            for fan_torque in [-0.002, 0, 0.002]:
                tau, w, saturated = tracker_step(error, rate, DT_CONTROL, DEFAULT_KP,
                    DEFAULT_KD, POINTING_DEADBAND_DEG, 200, external_torque_nm=fan_torque)
                wheel_accel = (w - 200) / DT_CONTROL / RPM_PER_RAD_S
                self.assertAlmostEqual(fan_torque - I_WHEEL * wheel_accel, tau, places=12)
                self.assertFalse(saturated)

    def test_combined_fan_and_pointing_acceleration_is_limited(self):
        _, w, _ = tracker_step(0, 0, DT_CONTROL, DEFAULT_KP, DEFAULT_KD,
                              POINTING_DEADBAND_DEG, 0, external_torque_nm=1)
        self.assertAlmostEqual(w, WHEEL_MAX_ACCEL_RPM_S * DT_CONTROL)
        _, w, saturated = tracker_step(0, 0, DT_CONTROL, DEFAULT_KP, DEFAULT_KD,
                              POINTING_DEADBAND_DEG, 399, external_torque_nm=1)
        self.assertEqual(w, 400)
        self.assertTrue(saturated)

    def test_unloading_both_signs_with_disturbance_and_calibration_error(self):
        # Synthetic 2 mN*m banks, with true thrust 20% lower and slower than the
        # controller estimate. A persistent 0.2 mN*m disturbance adds momentum.
        config = synthetic_config()
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                args = dict(bias_pwm=sign * 250 / RPM_PER_PWM, sun_start_deg=180,
                            duration_s=65, gyro_noise_dps=0, sun_noise=0,
                            disturbance_torque_nm=sign * 0.0002)
                log = simulate(**args, fan_config=config,
                               fan_torque_scale=0.8, fan_actual_tau_s=0.8)
                baseline = simulate(**args)
                tracking = log['mode'] == 'TRACK'
                on = log['fan_bank'] != 0
                self.assertTrue(on.any())
                self.assertTrue(np.all(log['fan_bank'][on] == -sign))
                self.assertGreater(abs(log['w_meas'][-1]), 95)
                self.assertLess(abs(log['w_meas'][-1]), 120)
                self.assertLess(abs(log['w_meas'][-1]), abs(baseline['w_meas'][-1]) - 150)
                self.assertLess(np.max(abs(log['err'][tracking])), 5)
                self.assertFalse(log['fan_fault'].any())
                self.assertFalse(np.any(on[log['t'] < 10]))

    def test_sun_loss_cuts_bank_commands(self):
        log = simulate(bias_pwm=330 / RPM_PER_PWM, sun_start_deg=180, duration_s=25,
                       gyro_noise_dps=0, sun_noise=0, sun_off_at_s=15,
                       fan_config=synthetic_config())
        self.assertTrue(np.any(log['fan_bank'][log['t'] < 15] != 0))
        self.assertTrue(np.all(log['fan_bank'][log['t'] >= 15] == 0))
        self.assertEqual(log['mode'][-1], 'STOPPED')

    def test_unloading_remains_active_during_a_fast_ninety_degree_slew(self):
        # Same synthetic, deliberately mismatched fan plant as the hold test.
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                log = simulate(bias_pwm=sign * 250 / RPM_PER_PWM,
                               sun_start_deg=180 + sign * 90, duration_s=65,
                               gyro_noise_dps=0, sun_noise=0,
                               fan_config=synthetic_config(),
                               disturbance_torque_nm=sign * 0.0002,
                               fan_torque_scale=0.8, fan_actual_tau_s=0.8)
                on = log['fan_bank'] != 0
                self.assertGreater(np.count_nonzero(on & (abs(log['err']) > 10)), 20)
                self.assertGreater(np.count_nonzero(on & (abs(log['rate']) > 3)), 20)
                self.assertTrue(np.all(log['fan_bank'][on] == -sign))
                self.assertFalse(log['fan_fault'].any())
                self.assertGreater(abs(log['w_meas'][-1]), 95)
                self.assertLess(abs(log['w_meas'][-1]), 115)
                self.assertLess(abs(log['err'][-1]), 5)

    def test_sun_loss_grace_counters_coasting_without_using_dark_pointing_error(self):
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                log = simulate(bias_pwm=sign * 330 / RPM_PER_PWM, sun_start_deg=180,
                               duration_s=25, gyro_noise_dps=0, sun_noise=0,
                               sun_off_at_s=15, fan_config=synthetic_config())
                grace = np.flatnonzero(log['mode'] == 'NOSUN')
                self.assertGreater(len(grace), 5)
                self.assertTrue(np.all(log['fan_bank'][grace] == 0))
                self.assertTrue(np.all(log['fan_torque'][grace] * sign < 0))
                self.assertTrue(np.all(log['tau'][grace] == 0))
                # Dark sensors report a meaningless -180 deg error. The only
                # command change must be the known residual torque compensation.
                np.testing.assert_allclose(log['err'][grace], -180)
                expected_change = log['fan_torque'][grace] / I_WHEEL * RPM_PER_RAD_S * DT_CONTROL
                np.testing.assert_allclose(log['w_cmd'][grace] - log['w_cmd'][grace - 1],
                                           expected_change, rtol=1e-9, atol=1e-12)
                self.assertLess(np.max(abs(log['rate'][grace])), 0.2)


class CppParityTests(unittest.TestCase):
    def test_actual_cpp_matches_python_when_host_compiler_is_available(self):
        compiler = shutil.which('g++') or shutil.which('clang++')
        if compiler is None:
            self.skipTest('No host C++ compiler available; Xtensa firmware cannot run on this PC')
        folder = Path(__file__).resolve().parent
        firmware = folder.parent / 'KestrelSAT_SunTracker'
        with tempfile.TemporaryDirectory(prefix='fan-parity-') as temp:
            executable = str(Path(temp) / 'fan_parity.exe')
            subprocess.run([compiler, '-std=c++11', '-I', str(firmware / 'include'),
                            str(folder / 'fan_parity.cpp'),
                            str(firmware / 'src/fan_desaturation.cpp'),
                            str(firmware / 'src/sun_tracker.cpp'), '-o', executable], check=True)
            samples = [(330, 0, 0, .05, 1), (250, 8, 0, .05, 1), (140, 0, 0, .05, 1),
                       (-330, 0, 0, .05, 1), (-250, 0, 0, .05, 0), (-330, 0, 4, .05, 1),
                       (-330, 0, 0, .21, 1), (330, 0, 0, .05, 1)]
            result = subprocess.run([executable], input=''.join(' '.join(map(str, s)) + '\n'
                                     for s in samples), text=True, capture_output=True, check=True)
            self.assertEqual(len(result.stdout.splitlines()), len(samples))
            state, config, w = FanDesaturationState(), synthetic_config(), 200.
            for line, sample in zip(result.stdout.splitlines(), samples):
                rpm, error, rate, dt, permit = sample
                bank, torque, fault = fan_desaturation_step(rpm, error, rate, dt, bool(permit), config, state, w)
                tau, w, sat = tracker_step(error, rate, dt, DEFAULT_KP, DEFAULT_KD,
                                          POINTING_DEADBAND_DEG, w, external_torque_nm=torque)
                expected = [bank, torque, fault, tau, w, sat]
                np.testing.assert_allclose(list(map(float, line.split())), expected, rtol=1e-4, atol=1e-6)


if __name__ == '__main__':
    unittest.main()
