"""순찰 지점 기록 파일 형식 시험 (ROS 불필요, pyyaml 이 있을 때만)."""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'security_patrol'))
from route_file import route_yaml  # noqa: E402

try:
    import yaml
except ImportError:      # 맥 기본 python 에는 pyyaml 이 없다 (VM 의 ROS 환경에는 있음)
    yaml = None


@unittest.skipIf(yaml is None, 'pyyaml 없음')
class RouteFileTest(unittest.TestCase):

    def test_patrol_node_format(self):
        text = route_yaml([('출발점', 0.0, 0.0), ('문 앞', 1.234, -0.5)])
        cfg = yaml.safe_load(text)['patrol']
        self.assertEqual(cfg['dwell_time'], 2.0)
        self.assertEqual(cfg['checkpoints'][1], {'name': '문 앞', 'x': 1.23, 'y': -0.5})

    def test_quote_in_name_does_not_break_file(self):
        cfg = yaml.safe_load(route_yaml([('a"b\\c', 1.0, 2.0)]))['patrol']
        self.assertEqual(cfg['checkpoints'][0]['name'], 'abc')

    def test_numeric_looking_name_stays_text(self):
        cfg = yaml.safe_load(route_yaml([('1', 1.0, 2.0)]))['patrol']
        self.assertEqual(cfg['checkpoints'][0]['name'], '1')


if __name__ == '__main__':
    unittest.main()
