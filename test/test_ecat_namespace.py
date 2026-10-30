from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import os
from advrf_ros2_motors_controller.launch_config import resolve_ecat_namespace, ecat_remappings, resolve_motor_profile, robot_reference, ros2_folder

class EcatNamespaceTest(unittest.TestCase):
    def test_separate_namespaces_and_explicit_override(self):
        with tempfile.TemporaryDirectory() as folder:
            config = Path(folder) / 'ec_cfg.yaml'
            config.write_text('ros2: {namespace: /advrf/bench}\necat_dds: {namespace: /advrf/bench/ecat}\n')
            self.assertEqual(resolve_ecat_namespace(ec_cfg=str(config)), '/advrf/bench/ecat')
            self.assertEqual(resolve_ecat_namespace('/custom/ecat', str(config)), '/custom/ecat')
            mappings = dict(ecat_remappings(resolve_ecat_namespace(ec_cfg=str(config))))
            self.assertEqual(mappings['rx/joint_states'], '/advrf/bench/ecat/rx/joint_states')
            self.assertEqual(mappings['tx/motors'], '/advrf/bench/ecat/tx/motors')
            self.assertNotIn('joint_commands', mappings)
            self.assertNotIn('go_to_joint_positions', mappings)

    def test_legacy_and_root_namespace(self):
        with tempfile.TemporaryDirectory() as folder:
            config = Path(folder) / 'ec_cfg.yaml'
            config.write_text('ros2: {namespace: /legacy}\n')
            self.assertEqual(resolve_ecat_namespace(ec_cfg=str(config)), '/legacy')
        self.assertEqual(resolve_ecat_namespace(namespace='/explicit'), '/explicit')
        self.assertEqual(dict(ecat_remappings('/'))['tx/motors'], '/tx/motors')

    def test_legacy_section_and_new_precedence(self):
        with tempfile.TemporaryDirectory() as folder:
            config = Path(folder) / 'ec_cfg.yaml'
            config.write_text('dds_bridge: {namespace: /legacy/ecat}\n')
            self.assertEqual(resolve_ecat_namespace(ec_cfg=str(config)), '/legacy/ecat')
            config.write_text('dds_bridge: {namespace: /legacy/ecat}\n'
                              'ecat_dds: {namespace: /advrf/bench/ecat}\n')
            self.assertEqual(resolve_ecat_namespace(ec_cfg=str(config)), '/advrf/bench/ecat')

    def test_dds_motor_profile_never_changes_power_profile(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            profile = root / 'tx.yaml'
            profile.write_text('motor_1: {control_mode: 59, gains: [1, 2, 0, 0, 3]}')
            config = root / 'ec_cfg.yaml'
            config.write_text('ros2: {motor_tx_profile: "${PWD}/tx.yaml"}\n')
            self.assertEqual(resolve_motor_profile(ec_cfg=str(config)), str(profile))
            self.assertEqual(resolve_motor_profile(ec_cfg=str(config), motor_power=True), '')
            self.assertEqual(resolve_motor_profile('/explicit.yaml', ec_cfg=str(config)), '/explicit.yaml')
            self.assertEqual(resolve_motor_profile(directory='/custom', ec_cfg=str(config)), '')

    def test_native_profile_namespace_and_environment_paths(self):
        with tempfile.TemporaryDirectory() as folder, patch.dict(os.environ, {'ADVRF_CONFIG_ROOT': '/tmp/native_robot'}):
            config = Path(folder) / 'ec_cfg.yaml'
            config.write_text('ros2: {namespace: /advrf/miranda, folder: "${ADVRF_CONFIG_ROOT}/ros2"}\n'
                              'robot: {robot_id_map_path: "${ADVRF_CONFIG_ROOT}/map.yaml"}\n'
                              'advrf_ros2_ecat: {}\n')
            self.assertEqual(resolve_ecat_namespace(ec_cfg=str(config)), '/advrf/miranda/ecat')
            self.assertEqual(ros2_folder(config), Path('/tmp/native_robot/ros2'))
            self.assertEqual(robot_reference(config, 'robot_id_map_path'), Path('/tmp/native_robot/map.yaml'))
            config.write_text('ros2: {namespace: /advrf/miranda}\n'
                              'advrf_ros2_ecat: {namespace: /custom/transport}\n')
            self.assertEqual(resolve_ecat_namespace(ec_cfg=str(config)), '/custom/transport')

    def test_invalid_namespace(self):
        with self.assertRaises(Exception):
            resolve_ecat_namespace('/bad namespace')

if __name__ == '__main__':
    unittest.main()
