"""
supervisor - runs the WHOLE project unattended (Phase 9).

    ros2 launch my_bot_bringup full_mission.launch.py            # <- the single command

The top-level launch file starts the simulator (+ odometry, evaluator, RViz) and THIS
node. The supervisor then runs the two STAGES one after the other, each as its own
`ros2 launch` process (so each stage starts clean and is also runnable on its own):

   t=0  ros2 launch ... full_mission.launch.py
    |
    +-- STAGE 1  stage_explore.launch.py   SLAM + Nav2 + explorer
    |              explorer finishes -> returns home -> saves map -> exits -> stage shuts down
    +-- check the saved map against the true world (compare_maps)
    +-- TELEPORT the robot to the mission start (kidnapped-robot test for AMCL)
    +-- STAGE 2  stage_mission.launch.py   map_server + AMCL + Nav2 + mission + scenario
    |              rough initial pose -> localize -> 5 goals -> surprise obstacle ->
    |              re-route -> hard block -> recovery -> retry -> mission_runner exits
    +-- write report (summary.md) -> exit -> the launch file shuts everything down
   t=end   "all goals complete" -> TOTAL WALL-CLOCK TIME = the success metric

Why subprocesses instead of one giant launch file? Switching from SLAM to AMCL means
STOPPING some nodes and STARTING others. A supervisor that runs stages is the simplest
robust way (an event-driven, pure-launch alternative is full_mission_events.launch.py).
"""
import json
import os
import shlex
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from ..common.ros_utils import (ignore_sigint, lookup_pose_2d, make_run_dir, spin_in_background,
                                write_json)
from ..common.sim_services import SimWorldClient
from ..evaluation.report import make_report
from .mission_io import load_mission


class Supervisor(Node):
    def __init__(self):
        super().__init__('supervisor')
        p = self.declare_parameter
        self.run_dir = make_run_dir(p('run_dir', '').value)
        self.explore_args = p('explore_args', '').value
        self.mission_args = p('mission_args', '').value
        self.mission_file = p('mission_file', 'demo_mission.yaml').value
        self.skip_exploration = p('skip_exploration', False).value
        self.map_yaml = p('map', '').value
        self.world_file = p('world_file', '').value
        self.world_name = p('world_name', 'nav_room').value
        self.robot_name = p('robot_name', 'my_bot').value
        self.teleport = p('teleport_to_mission_start', True).value
        self.explore_timeout = p('explore_timeout', 1200.0).value
        self.mission_timeout = p('mission_timeout', 1500.0).value
        # 'all' = copy every line of the stage output to this terminal; 'important' = only
        # explorer/mission/scenario events, warnings and errors (the full output is ALWAYS in
        # <run_dir>/stageN_*.log). 'important' keeps the terminal readable and saves CPU.
        self.stage_output = p('stage_output', 'important').value
        self.t_launch = time.time()
        self.tf_buffer = None
        self.sim = SimWorldClient(self, self.world_name)
        self.child = None
        self.timeline = []

    # ------------------------------------------------------------ helpers
    def mark(self, what, **extra):
        t = round(time.time() - self.t_launch, 1)
        self.timeline.append(dict(t=t, what=what, **extra))
        self.get_logger().info(f'[supervisor t={t:7.1f}s] {what} {extra if extra else ""}')

    def run_stage(self, name, launch_file, args, timeout):
        cmd = ['ros2', 'launch', 'my_bot_bringup', launch_file, f'run_dir:={self.run_dir}'] + shlex.split(args)
        self.mark(f'{name} START', cmd=' '.join(cmd))
        log_path = os.path.join(self.run_dir, f'{name}.log')
        # start_new_session -> the stage gets its own process GROUP, so we can stop the
        # stage and every node it started with one signal.
        self.child = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                      start_new_session=True, text=True, bufsize=1)

        important = ('[explore]', '[mission]', '[scenario]', 'ERROR', 'WARN', 'FATAL',
                     'process has died', 'Traceback', 'Error')

        def pump():   # copy the stage output to a log file AND (all or part of it) to the terminal
            with open(log_path, 'w') as f:
                for line in self.child.stdout:
                    f.write(line)
                    if self.stage_output == 'all' or any(k in line for k in important):
                        sys.stdout.write(f'[{name}] {line}')
        threading.Thread(target=pump, daemon=True).start()
        try:
            rc = self.child.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.mark(f'{name} TIMEOUT -> stopping it', seconds=timeout)
            self.stop_child()
            rc = -1
        self.child = None
        self.mark(f'{name} END', returncode=rc)
        return rc

    def stop_child(self):
        if self.child is None or self.child.poll() is not None:
            return
        pgid = os.getpgid(self.child.pid)
        for sig, wait in ((signal.SIGINT, 20), (signal.SIGTERM, 10), (signal.SIGKILL, 5)):
            try:
                os.killpg(pgid, sig)
                self.child.wait(timeout=wait)
                return
            except subprocess.TimeoutExpired:
                continue
            except ProcessLookupError:
                return

    def wait_for_robot(self, timeout=180.0):
        # A TF listener subscribes to /tf (hundreds of messages per second). We only need it
        # here, so it is created now and destroyed afterwards - Python TF listeners cost CPU.
        self.tf_buffer = Buffer()
        listener = TransformListener(self.tf_buffer, self)
        try:
            t0 = time.time()
            while rclpy.ok() and time.time() - t0 < timeout:
                if lookup_pose_2d(self.tf_buffer, 'odom', 'base_footprint') is not None:
                    return True
                time.sleep(0.5)
            return False
        finally:
            listener.unregister()

    # ------------------------------------------------------------ the pipeline
    def run(self):
        result = {'run_dir': self.run_dir, 'success': False}
        self.mark('waiting for the simulated robot (TF odom -> base_footprint)')
        if not self.wait_for_robot():
            raise RuntimeError('robot never appeared (simulator problem?) - see the launch output')
        self.mark('robot is alive')

        # ---------------- STAGE 1: explore + map
        map_yaml = os.path.expanduser(self.map_yaml) if self.map_yaml else ''
        if not self.skip_exploration:
            self.run_stage('stage1_explore', 'stage_explore.launch.py', self.explore_args, self.explore_timeout)
            res_path = os.path.join(self.run_dir, 'exploration_result.json')
            if os.path.exists(res_path):
                with open(res_path) as f:
                    explo = json.load(f)
                result['exploration'] = explo
                map_yaml = explo.get('map_yaml') or ''
            if not map_yaml or not os.path.exists(map_yaml):
                raise RuntimeError('exploration did not produce a map')
            self.mark('map saved', map=map_yaml)
        result['map_yaml'] = map_yaml

        # ---------------- map quality (needs the world file)
        if self.world_file and os.path.exists(self.world_file):
            try:
                from ..common.map_io import read_map
                from ..common.world_model import World2D
                from ..mapping.compare_maps import compare
                from ..mapping.reference_map import build_reference_grid
                g, r, o = read_map(map_yaml)
                rg, rr, ro = build_reference_grid(World2D.from_sdf_file(self.world_file))
                result['map_quality'] = compare(g, r, o, rg, rr, ro)
                write_json(os.path.join(self.run_dir, 'map_quality.json'), result['map_quality'])
                self.mark('map quality', **{k: round(v, 3) if isinstance(v, float) else v
                                            for k, v in result['map_quality'].items()})
            except Exception as e:
                self.get_logger().warn(f'map comparison failed: {e!r}')

        # ---------------- teleport (kidnap) to the mission start
        mission = load_mission(self.mission_file)
        if self.teleport:
            x, y, yaw = mission['start_pose']
            ok = self.sim.set_pose(self.robot_name, x, y, yaw)
            self.mark('teleported robot to mission start', ok=ok, pose=[x, y, yaw])
            time.sleep(2.0)

        # ---------------- STAGE 2: localize + mission
        args = f'map:={map_yaml} mission_file:={self.mission_file} {self.mission_args}'
        self.run_stage('stage2_mission', 'stage_mission.launch.py', args, self.mission_timeout)
        res_path = os.path.join(self.run_dir, 'mission_results.json')
        if os.path.exists(res_path):
            with open(res_path) as f:
                result['mission'] = json.load(f).get('summary')
        total = time.time() - self.t_launch
        result['total_wall_clock_s'] = round(total, 1)
        result['success'] = bool(result.get('mission', {}) and result['mission'].get('all_goals_complete'))
        result['timeline'] = self.timeline
        write_json(os.path.join(self.run_dir, 'run_summary.json'), result)
        report = make_report(self.run_dir)
        self.mark('ALL DONE', success=result['success'], total_wall_clock_s=result['total_wall_clock_s'])
        print('\n' + '=' * 78 + '\n' + report + '\n' + '=' * 78 + '\n', flush=True)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = Supervisor()
    executor = spin_in_background(node)
    code = 1
    try:
        res = node.run()
        code = 0 if res['success'] else 2
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'supervisor failed: {e!r}')
        write_json(os.path.join(node.run_dir, 'run_summary.json'),
                   {'success': False, 'error': repr(e), 'timeline': node.timeline})
    finally:
        ignore_sigint()
        node.stop_child()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
