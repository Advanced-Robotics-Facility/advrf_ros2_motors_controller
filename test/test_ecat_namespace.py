import unittest

from advrf_ros2_motors_controller.ros_names import (
    resolve_namespace,
    resolve_ecat_namespace,
)


class EcatNamespaceTest(unittest.TestCase):
    def test_defaults_and_explicit_namespaces(self):
        self.assertEqual(resolve_namespace(), '/advrf/robot')
        self.assertEqual(resolve_namespace('/'), '/')
        self.assertEqual(resolve_ecat_namespace(namespace='/robot'), '/robot/ecat')
        self.assertEqual(resolve_ecat_namespace('/custom/'), '/custom/ecat')
        self.assertEqual(resolve_ecat_namespace(namespace='/'), '/ecat')

    def test_invalid_namespace(self):
        with self.assertRaises(Exception):
            resolve_ecat_namespace('/bad namespace')


if __name__ == '__main__':
    unittest.main()
