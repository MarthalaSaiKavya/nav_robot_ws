"""
Thin, blocking wrappers around Nav2's ACTION servers, usable from a worker thread.

ROS 2 ACTIONS in one paragraph:
  An action is a long-running request with three channels:
    goal     -> "drive to (x, y)"            (the server can accept or reject it)
    feedback <- "distance remaining 3.2 m"   (streamed while it runs)
    result   <- "SUCCEEDED / ABORTED / CANCELED"
  and it can be CANCELED midway. Nav2 exposes its features as actions:
    navigate_to_pose, navigate_through_poses, follow_waypoints,
    compute_path_to_pose, follow_path, spin, backup, wait, ...

Nav2's own Python helper (nav2_simple_commander.BasicNavigator) wraps the same thing;
mission_runner shows both styles. This class is the "raw rclpy" style.
"""
import threading
import time

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient

from .ros_utils import make_pose_stamped, wait_for_future

STATUS_NAMES = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
                GoalStatus.STATUS_CANCELED: 'CANCELED', GoalStatus.STATUS_UNKNOWN: 'UNKNOWN'}


class ActionHandle:
    """One running goal: lets the caller poll, read feedback, cancel and wait."""

    def __init__(self, goal_handle):
        self.goal_handle = goal_handle
        self.feedback = None
        self.status = None
        self.result = None
        self._done = threading.Event()
        if goal_handle is not None and goal_handle.accepted:
            fut = goal_handle.get_result_async()
            fut.add_done_callback(self._on_result)
        else:
            self.status = GoalStatus.STATUS_ABORTED
            self._done.set()

    @property
    def accepted(self):
        return self.goal_handle is not None and self.goal_handle.accepted

    def _on_result(self, fut):
        res = fut.result()
        self.status = res.status
        self.result = res.result
        self._done.set()

    def done(self):
        return self._done.is_set()

    def wait(self, timeout=None):
        return self._done.wait(timeout)

    def cancel(self, timeout=5.0):
        if self.goal_handle is not None and not self.done():
            wait_for_future(self.goal_handle.cancel_goal_async(), timeout)
            self._done.wait(timeout)

    @property
    def status_name(self):
        return STATUS_NAMES.get(self.status, str(self.status))

    @property
    def succeeded(self):
        return self.status == GoalStatus.STATUS_SUCCEEDED


class Nav2Actions:
    def __init__(self, node, callback_group=None):
        from nav2_msgs.action import ComputePathToPose, NavigateToPose, Spin
        self.node = node
        self._NavigateToPose, self._ComputePathToPose, self._Spin = NavigateToPose, ComputePathToPose, Spin
        self.nav = ActionClient(node, NavigateToPose, 'navigate_to_pose', callback_group=callback_group)
        self.plan = ActionClient(node, ComputePathToPose, 'compute_path_to_pose', callback_group=callback_group)
        self.spin = ActionClient(node, Spin, 'spin', callback_group=callback_group)

    def wait_for_servers(self, timeout=60.0, need_spin=False):
        """Waits until the action servers EXIST. Careful: a Nav2 server exists as soon as its
        process starts, but it REJECTS goals until its lifecycle state is 'active'.
        Use wait_until_active() for that."""
        t0 = time.time()
        for client in [self.nav, self.plan] + ([self.spin] if need_spin else []):
            remaining = max(timeout - (time.time() - t0), 0.1)
            if not client.wait_for_server(timeout_sec=remaining):
                return False
        return True

    def wait_until_active(self, nodes=('bt_navigator', 'behavior_server'), timeout=180.0):
        """Waits until every listed Nav2 LIFECYCLE node reports state 'active'.

        LIFECYCLE NODES in one paragraph: Nav2 servers start 'unconfigured', then the
        lifecycle_manager walks them through configure -> activate in a fixed order. Only an
        'active' server accepts goals. Every lifecycle node offers a service
        /<node>/get_state, which is what `ros2 lifecycle get /bt_navigator` calls too.
        (nav2_simple_commander's waitUntilNav2Active() does exactly this.)"""
        from lifecycle_msgs.srv import GetState

        from .ros_utils import call_service
        t0 = time.time()
        for name in nodes:
            client = self.node.create_client(GetState, f'/{name}/get_state')
            try:
                while time.time() - t0 < timeout:
                    res = call_service(client, GetState.Request(), timeout_sec=2.0, wait_service_sec=2.0)
                    if res is not None and res.current_state.label == 'active':
                        break
                    time.sleep(1.0)
                else:
                    return False
            finally:
                self.node.destroy_client(client)
        return self.wait_for_servers(timeout=max(timeout - (time.time() - t0), 5.0), need_spin=True)

    def navigate(self, x, y, yaw, frame='map', behavior_tree=''):
        goal = self._NavigateToPose.Goal()
        goal.pose = make_pose_stamped(x, y, yaw, frame, self.node.get_clock().now().to_msg())
        goal.behavior_tree = behavior_tree
        handle_holder = {}

        def on_feedback(msg):
            if 'h' in handle_holder:
                handle_holder['h'].feedback = msg.feedback
        gh = wait_for_future(self.nav.send_goal_async(goal, feedback_callback=on_feedback), 10.0)
        h = ActionHandle(gh)
        handle_holder['h'] = h
        return h

    def path_length(self, x, y, yaw=0.0, frame='map', timeout=3.0):
        """Asks the planner for a path WITHOUT driving it. Returns length [m] or None."""
        goal = self._ComputePathToPose.Goal()
        goal.goal = make_pose_stamped(x, y, yaw, frame, self.node.get_clock().now().to_msg())
        goal.use_start = False
        gh = wait_for_future(self.plan.send_goal_async(goal), timeout)
        if gh is None or not gh.accepted:
            return None
        res = wait_for_future(gh.get_result_async(), timeout)
        if res is None or res.status != GoalStatus.STATUS_SUCCEEDED:
            return None
        poses = res.result.path.poses
        if len(poses) < 2:
            return 0.0
        total = 0.0
        for a, b in zip(poses[:-1], poses[1:]):
            dx = b.pose.position.x - a.pose.position.x
            dy = b.pose.position.y - a.pose.position.y
            total += (dx * dx + dy * dy) ** 0.5
        return total

    def spin_in_place(self, angle=6.28, time_allowance=30):
        goal = self._Spin.Goal()
        goal.target_yaw = float(angle)
        goal.time_allowance.sec = int(time_allowance)
        gh = wait_for_future(self.spin.send_goal_async(goal), 10.0)
        h = ActionHandle(gh)
        h.wait(time_allowance + 5)
        return h.succeeded
