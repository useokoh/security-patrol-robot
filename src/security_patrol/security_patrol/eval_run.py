"""7단계 통합 시험 노드: 시나리오대로 물체를 만들고 움직이며, 감지·알림을 기록해 결과표를 만든다.

nav.launch.py, detect.launch.py, patrol_node 가 켜진 상태에서 실행한다.
순찰 바퀴 번호(/patrol/status)에 맞춰 config/eval_scenario.yaml 의 물체를 Gazebo 에 만들고,
정해진 바퀴 수가 끝나면 결과표를 터미널에 출력하고 ~/security_patrol_ws/eval_results/ 에 저장한 뒤 종료한다.
채점 규칙은 eval_metrics.py 에 있다.

  ros2 run security_patrol eval_run --ros-args -p use_sim_time:=true
"""

import json
import math
import os
import subprocess
import threading
import time
from datetime import datetime

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import String

from security_patrol.eval_metrics import TruthObject, report_lines, summarize
from security_patrol.intruder_walker import position_at, set_pose


class EvalRun(Node):

    def __init__(self):
        super().__init__('eval_run')
        self.share = get_package_share_directory('security_patrol')
        scenario_file = self.declare_parameter(
            'scenario_file', os.path.join(self.share, 'config', 'eval_scenario.yaml')).value
        self.output_dir = os.path.expanduser(self.declare_parameter(
            'output_dir', '~/security_patrol_ws/eval_results').value)
        with open(scenario_file, encoding='utf-8') as f:
            cfg = yaml.safe_load(f)['eval']
        self.laps = int(cfg['laps'])
        self.world = cfg['world']
        self.spawns = cfg.get('objects', [])
        self.walks = cfg.get('walks', [])
        self.objects = {o['name']: TruthObject(o['name'], o['x'], o['y'], o.get('person', False))
                        for o in self.spawns}

        self.detections, self.alerts = [], []
        self.robot = None
        self.lap = None
        self.lap_started = None
        self.lap_times = []
        self.finished = False
        self.lock = threading.Lock()

        self.create_subscription(String, 'patrol/status', self.on_status, 10)
        self.create_subscription(String, 'anomaly/detections', self.on_detection, 10)
        self.create_subscription(String, 'anomaly/alerts', self.on_alert, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_pose, 10)
        self.get_logger().info(
            f'통합 시험: {self.laps}바퀴, 물체 {len(self.spawns)}개, 걷기 {len(self.walks)}개. 순찰 시작을 기다리는 중...')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ---------- 기록 ----------

    def on_pose(self, msg):
        p = msg.pose.pose.position
        self.robot = (p.x, p.y)

    def on_detection(self, msg):
        data = json.loads(msg.data)
        data['robot'] = self.robot
        with self.lock:
            self.detections.append(data)

    def on_alert(self, msg):
        data = json.loads(msg.data)
        data['stamp'] = self.now()
        with self.lock:
            self.alerts.append(data)

    # ---------- 시나리오 진행 ----------

    def on_status(self, msg):
        lap = json.loads(msg.data)['lap']
        if lap == self.lap or self.finished:
            return
        now = self.now()
        if self.lap is not None:
            self.lap_times.append(now - self.lap_started)
            self.get_logger().info(f'===== {self.lap}바퀴 끝 ({self.lap_times[-1]:.0f}초) =====')
        self.lap, self.lap_started = lap, now
        if lap > self.laps:
            self.finish()
            return
        self.get_logger().info(f'===== {lap}바퀴 시작 =====')
        for s in self.spawns:
            if s['lap'] == lap:
                threading.Thread(target=self.spawn, args=(s,), daemon=True).start()
        for w in self.walks:
            if w['lap'] == lap:
                threading.Thread(target=self.walk, args=(w,), daemon=True).start()

    def spawn(self, s):
        path = os.path.join(self.share, 'models', s['model'])
        result = subprocess.run(
            ['ros2', 'run', 'ros_gz_sim', 'create', '-world', self.world, '-name', s['name'],
             '-x', str(s['x']), '-y', str(s['y']), '-file', path],
            capture_output=True, text=True)
        if result.returncode == 0 and 'OK creation' in (result.stdout + result.stderr):
            self.objects[s['name']].spawned_at = self.now()
            self.get_logger().info(f'생성: {s["name"]} ({s["x"]:+.2f}, {s["y"]:+.2f})')
        else:
            self.get_logger().error(f'생성 실패: {s["name"]}')

    def walk(self, w):
        obj = self.objects[w['name']]
        start, end = tuple(w['start']), tuple(w['end'])
        obj.walk = (self.now(), start, end)
        self.get_logger().info(f'걷기 시작: {w["name"]} {start} ↔ {end}, {w["speed"]} m/s, {w["duration"]}초')
        begin = time.monotonic()
        slowest = 0.0
        while time.monotonic() - begin < w['duration'] and not self.finished:
            call = time.monotonic()
            x, y = position_at(call - begin, start, end, w['speed'])
            set_pose(self.world, w['name'], x, y)
            slowest = max(slowest, time.monotonic() - call)
            time.sleep(max(0.0, 0.2 - (time.monotonic() - call)))
        self.get_logger().info(f'걷기 끝: {w["name"]} (가장 느린 위치 변경 {slowest:.2f}초)')

    # ---------- 결과 ----------

    def finish(self):
        self.finished = True
        objects = list(self.objects.values())
        with self.lock:
            summary = summarize(objects, self.detections, self.alerts)
            raw = {'detections': self.detections, 'alerts': self.alerts}
        lines = report_lines(objects, summary, self.lap_times)
        for line in lines:
            self.get_logger().info(line)
        os.makedirs(self.output_dir, exist_ok=True)
        base = os.path.join(self.output_dir, datetime.now().strftime('%Y%m%d_%H%M%S'))
        with open(base + '.md', 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines) + '\n')
        with open(base + '.json', 'w', encoding='utf-8') as f:
            json.dump(raw, f, ensure_ascii=False, indent=1, default=lambda v: None if v is math.inf else str(v))
        self.get_logger().info(f'저장: {base}.md / .json  (순찰과 다른 터미널은 Ctrl+C 로 끄면 됩니다)')
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = EvalRun()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
