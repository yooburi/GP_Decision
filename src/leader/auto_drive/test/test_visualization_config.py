import re
import unittest
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = PACKAGE_DIR.parents[1]


def parameter_value(path, name):
    text = path.read_text(encoding='utf-8')
    match = re.search(
        rf'^\s*{re.escape(name)}:\s*["\']?([^"\'\s]+)',
        text,
        re.MULTILINE,
    )
    if match is None:
        raise AssertionError(f'Missing {name} in {path}')
    return match.group(1)


class VisualizationConfigTest(unittest.TestCase):
    def test_vehicle_markers_use_localization_vehicle_frame(self):
        mission_config = PACKAGE_DIR / 'config' / 'mission_supervisor.yaml'
        localization_config = (
            SRC_DIR / 'gps' / 'gps_to_utm' / 'config' / 'tf_gps_csv_single.yaml'
        )
        vehicle_frame = parameter_value(localization_config, 'vehicle_frame_id')

        self.assertEqual(
            parameter_value(mission_config, 'mission_marker_frame_id'),
            vehicle_frame,
        )
        self.assertEqual(
            parameter_value(mission_config, 'safety_marker_frame_id'),
            vehicle_frame,
        )

    def test_mission_zones_use_localization_csv_frame(self):
        zone_config = PACKAGE_DIR / 'config' / 'mission_zones.yaml'
        localization_config = (
            SRC_DIR / 'gps' / 'gps_to_utm' / 'config' / 'tf_gps_csv_single.yaml'
        )

        self.assertEqual(
            parameter_value(zone_config, 'csv_frame_id'),
            parameter_value(localization_config, 'csv_frame_id'),
        )

    def test_complex_rviz_follows_localization_vehicle_frame(self):
        rviz_config = (PACKAGE_DIR / 'config' / 'complex.rviz').read_text(
            encoding='utf-8'
        )
        localization_config = (
            SRC_DIR / 'gps' / 'gps_to_utm' / 'config' / 'tf_gps_csv_single.yaml'
        )
        vehicle_frame = parameter_value(localization_config, 'vehicle_frame_id')

        self.assertIn(f'Fixed Frame: {vehicle_frame}', rviz_config)
        self.assertIn(f'Target Frame: {vehicle_frame}', rviz_config)
        self.assertNotIn('Fixed Frame: vehicle_ref', rviz_config)
        self.assertNotIn('Reference Frame: vehicle_ref', rviz_config)
        self.assertNotIn('Target Frame: vehicle_ref', rviz_config)


if __name__ == '__main__':
    unittest.main()
