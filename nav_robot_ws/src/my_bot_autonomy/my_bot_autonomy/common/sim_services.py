"""
Change the simulated world from ROS: spawn / remove obstacles, teleport the robot.

Two interchangeable BACKENDS (a nice example of "same job, different technique"):

  backend='ros'    (default)
      Calls ROS services /world/nav_room/{create,remove,set_pose}
      (ros_gz_interfaces/srv/SpawnEntity, DeleteEntity, SetEntityPose).
      With Gazebo they are provided by ros_gz_bridge (see bridge_*.yaml);
      with sim:=lite the lite simulator provides the very same services.

  backend='gz_cli'
      Shells out to the Gazebo command line tool, e.g.
        gz service -s /world/nav_room/create --reqtype gz.msgs.EntityFactory \
                   --reptype gz.msgs.Boolean --timeout 3000 --req 'sdf: "..." ...'
      No bridge needed, but Gazebo-only. Good to know for quick experiments.

All methods are BLOCKING and must be called from a worker thread while the node is
spun elsewhere (see ros_utils module docstring).
"""
import math
import subprocess

from .ros_utils import call_service, quaternion_msg
from .world_model import box_model_sdf


class SimWorldClient:
    def __init__(self, node, world_name='nav_room', backend='ros'):
        self.node = node
        self.world = world_name
        self.backend = backend
        self._clients = {}
        if backend == 'ros':
            from ros_gz_interfaces.srv import DeleteEntity, SetEntityPose, SpawnEntity
            self._SpawnEntity, self._DeleteEntity, self._SetEntityPose = SpawnEntity, DeleteEntity, SetEntityPose
            base = f'/world/{world_name}'
            self._clients['create'] = node.create_client(SpawnEntity, f'{base}/create')
            self._clients['remove'] = node.create_client(DeleteEntity, f'{base}/remove')
            self._clients['set_pose'] = node.create_client(SetEntityPose, f'{base}/set_pose')

    # ------------------------------------------------------------------ public API
    def spawn_box(self, name, x, y, yaw=0.0, sx=0.5, sy=0.5, sz=0.6, color=(0.95, 0.45, 0.1)):
        sdf = box_model_sdf(name, sx, sy, sz, color)
        if self.backend == 'ros':
            req = self._SpawnEntity.Request()
            req.entity_factory.name = name
            req.entity_factory.allow_renaming = False
            req.entity_factory.sdf = sdf
            req.entity_factory.pose.position.x = float(x)
            req.entity_factory.pose.position.y = float(y)
            req.entity_factory.pose.position.z = 0.0
            req.entity_factory.pose.orientation = quaternion_msg(yaw)
            req.entity_factory.relative_to = 'world'
            return self._ok(call_service(self._clients['create'], req), 'create', name)
        txt = sdf.replace('"', '\\"')
        req = (f'sdf: "{txt}" name: "{name}" allow_renaming: false '
               f'pose: {{position: {{x: {x}, y: {y}, z: 0}}, '
               f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}}}')
        return self._gz('create', 'gz.msgs.EntityFactory', req, name)

    def remove(self, name):
        if self.backend == 'ros':
            req = self._DeleteEntity.Request()
            req.entity.name = name
            req.entity.type = 2  # MODEL
            return self._ok(call_service(self._clients['remove'], req), 'remove', name)
        return self._gz('remove', 'gz.msgs.Entity', f'name: "{name}" type: MODEL', name)

    def set_pose(self, name, x, y, yaw=0.0, z=0.02):
        if self.backend == 'ros':
            req = self._SetEntityPose.Request()
            req.entity.name = name
            req.entity.type = 2
            req.pose.position.x, req.pose.position.y, req.pose.position.z = float(x), float(y), float(z)
            req.pose.orientation = quaternion_msg(yaw)
            return self._ok(call_service(self._clients['set_pose'], req), 'set_pose', name)
        req = (f'name: "{name}" position: {{x: {x}, y: {y}, z: {z}}} '
               f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
        return self._gz('set_pose', 'gz.msgs.Pose', req, name)

    def available(self, timeout_sec=2.0):
        if self.backend != 'ros':
            return True
        return self._clients['create'].wait_for_service(timeout_sec=timeout_sec)

    # ------------------------------------------------------------------ internals
    def _ok(self, response, what, name):
        ok = bool(response is not None and response.success)
        log = self.node.get_logger()
        (log.info if ok else log.warn)(f'[sim] {what} "{name}" -> {"OK" if ok else "FAILED"}')
        return ok

    def _gz(self, service, reqtype, req, name):
        cmd = ['gz', 'service', '-s', f'/world/{self.world}/{service}', '--reqtype', reqtype,
               '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req]
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            ok = 'data: true' in out.stdout
        except Exception as e:  # gz not installed, timeout ...
            self.node.get_logger().error(f'[sim] gz CLI failed: {e}')
            ok = False
        (self.node.get_logger().info if ok else self.node.get_logger().warn)(
            f'[sim/gz_cli] {service} "{name}" -> {"OK" if ok else "FAILED"}')
        return ok
