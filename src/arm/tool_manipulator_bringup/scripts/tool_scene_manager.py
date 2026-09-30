#!/usr/bin/env python3
"""Yaw-trajectory-and-retreat-gated attachment collision manager."""
from __future__ import annotations

import json
import threading
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Pose, TransformStamped
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from tool_ids import NO_TOOL_ID, UNKNOWN_TOOL_ID


class ToolSceneManager(Node):
    """Scene changes follow confirmed physical state; restart begins UNKNOWN."""
    NONE = NO_TOOL_ID
    UNKNOWN = UNKNOWN_TOOL_ID

    def __init__(self) -> None:
        super().__init__('tool_scene_manager')
        for name, default in (
            ('tools_config_file', ''), ('tool_id_topic', '/selected_tool_id'),
            ('docking_complete_topic', '/docking_complete'),
            ('motion_status_topic', '/docking_motion_status'),
            ('attach_link', 'ee_output_link'),
            ('apply_planning_scene_service', '/apply_planning_scene'),
        ):
            self.declare_parameter(name, default)
        self.tools = self._load_tools(self.get_parameter('tools_config_file').value)
        self.fixtures = self._load_fixtures(self.get_parameter('tools_config_file').value)
        self.pending_tool_id = self.NONE
        # A process restart cannot infer whether a tool is mechanically retained.
        self.active_tool_id = self.UNKNOWN
        self.physical_engagement_started = False
        self.lock = threading.Lock()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.active_pub = self.create_publisher(Int32, '/active_tool_id', latched)
        self.status_pub = self.create_publisher(String, '/tool_attachment_status', latched)
        self.tcp_pub = self.create_publisher(String, '/active_tool_tcp', latched)
        self.payload_pub = self.create_publisher(String, '/active_tool_payload', latched)
        self.tf_pub = TransformBroadcaster(self)
        self.apply_client = self.create_client(ApplyPlanningScene, self.get_parameter('apply_planning_scene_service').value)
        self._register_fixtures()
        self.create_subscription(Int32, self.get_parameter('tool_id_topic').value, self._selected_cb, 10)
        self.create_subscription(Bool, self.get_parameter('docking_complete_topic').value, self._docked_cb, 10)
        self.create_subscription(String, self.get_parameter('motion_status_topic').value, self._motion_cb, 10)
        self.create_service(Trigger, '~/attach_pending', self._attach_service)
        self.create_service(Trigger, '~/detach_active', self._detach_service)
        self.create_service(Trigger, '~/operator_confirm_empty', self._confirm_empty_service)
        self.create_timer(0.2, self._publish_active_tcp_tf)
        self._publish('unknown:restart_requires_recovery')

    @staticmethod
    def _load_tools(filename: str) -> dict[int, dict]:
        if not filename:
            raise RuntimeError('tools_config_file is required')
        try:
            with Path(filename).open(encoding='utf-8') as stream:
                return {int(key): value for key, value in (yaml.safe_load(stream) or {}).get('tools', {}).items()}
        except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
            raise RuntimeError(f'cannot read tools_config_file: {exc}') from exc

    @staticmethod
    def _load_fixtures(filename: str) -> dict[str, dict]:
        try:
            with Path(filename).open(encoding='utf-8') as stream:
                fixtures = (yaml.safe_load(stream) or {}).get('fixtures', {})
            return {str(key): value for key, value in fixtures.items() if isinstance(value, dict)}
        except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
            raise RuntimeError(f'cannot read fixture collision registry: {exc}') from exc

    def _selected_cb(self, msg: Int32) -> None:
        tool_id = int(msg.data)
        if self.active_tool_id == self.UNKNOWN:
            self._publish('rejected:unknown_tool_state_requires_recovery')
        elif tool_id == self.NONE:
            self.pending_tool_id = self.NONE
            self._publish('selection_cleared')
        elif tool_id in self.tools:
            self.pending_tool_id = tool_id
            self._publish(f'pending_physical_dock:{tool_id}')
        else:
            self._publish(f'rejected:unknown_tool:{tool_id}')

    def _motion_cb(self, msg: String) -> None:
        state = msg.data
        if state.startswith(('descending', 'lock_rotating', 'retreat')):
            self.physical_engagement_started = True
        if state.startswith('failed:') and self.physical_engagement_started:
            self._mark_unknown(f'unknown:physical_failure:{state[7:]}')

    def _docked_cb(self, msg: Bool) -> None:
        # The executor emits this only after yaw trajectory success and the
        # configured retreat completes. It is never a yaw-complete event.
        if msg.data:
            self._attach_pending()

    def _attach_service(self, _request, response):
        response.success, response.message = self._attach_pending()
        return response

    def _detach_service(self, _request, response):
        response.success, response.message = False, 'physical detach is not implemented; scene detach is blocked'
        self._publish('rejected:physical_detach_not_implemented')
        return response

    def _confirm_empty_service(self, _request, response):
        if self.active_tool_id != self.UNKNOWN:
            response.success, response.message = False, 'operator recovery is only valid from UNKNOWN'
            return response
        if not self.apply_client.service_is_ready():
            response.success, response.message = False, 'apply_planning_scene unavailable'
            return response
        self._apply(self._clear_all_tool_scene(), self.NONE, 'recover_empty')
        response.success, response.message = True, 'operator-confirmed empty recovery requested'
        return response

    def _mark_unknown(self, reason: str) -> None:
        self.pending_tool_id = self.NONE
        self.active_tool_id = self.UNKNOWN
        self.physical_engagement_started = False
        self._publish(reason)

    def _attach_pending(self) -> tuple[bool, str]:
        tool_id, spec = self.pending_tool_id, self.tools.get(self.pending_tool_id, {})
        if self.active_tool_id == self.UNKNOWN:
            self._publish('rejected:unknown_tool_state_requires_recovery')
            return False, 'unknown attachment state'
        if tool_id == self.NONE:
            self._publish('rejected:no_pending_tool')
            return False, 'no pending tool'
        if self.active_tool_id != self.NONE:
            self._publish(f'rejected:tool_{self.active_tool_id}_still_attached')
            return False, 'another tool is active'
        if not spec.get('enabled', False):
            self._publish(f'rejected:tool_{tool_id}_disabled')
            return False, 'tool disabled'
        if not spec.get('collision', {}).get('primitives'):
            self._publish(f'rejected:tool_{tool_id}_has_no_primitive_collision')
            return False, 'collision geometry unavailable'
        self._apply(self._attach_scene(tool_id, spec), tool_id, 'attach')
        return True, 'attach requested after physical retreat'

    def _register_fixtures(self) -> None:
        if not self.fixtures:
            return
        scene = PlanningScene(is_diff=True)
        for fixture_id, spec in self.fixtures.items():
            if not isinstance(spec.get('frame'), str) or not spec.get('primitives'):
                self.get_logger().error(f'Fixture {fixture_id} is incomplete; it will not enter the planning scene')
                continue
            obj = CollisionObject(id=f'fixture_{fixture_id}', operation=CollisionObject.ADD)
            obj.header.frame_id = spec['frame']
            self._append_primitives(obj, spec['primitives'])
            scene.world.collision_objects.append(obj)
        if scene.world.collision_objects:
            self._apply(scene, 0, 'fixture_register')

    def _attached_id(self, tool_id: int) -> str:
        return f'tool_{tool_id}_attached_collision'

    def _slot_id(self, tool_id: int) -> str:
        return f'tool_{tool_id}_slot_collision'

    @staticmethod
    def _pose(spec: dict) -> Pose:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = [float(value) for value in spec.get('xyz', [0, 0, 0])]
        q = spec.get('quat_xyzw', [0, 0, 0, 1])
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = [float(value) for value in q]
        return pose

    @staticmethod
    def _append_primitives(obj: CollisionObject, entries: list[dict]) -> None:
        for entry in entries:
            primitive = SolidPrimitive()
            if entry['type'] == 'box':
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions = [float(value) for value in entry['size']]
            elif entry['type'] == 'cylinder':
                primitive.type = SolidPrimitive.CYLINDER
                primitive.dimensions = [float(entry['length']), float(entry['radius'])]
            else:
                raise ValueError(f"unsupported collision primitive: {entry['type']}")
            obj.primitives.append(primitive)
            obj.primitive_poses.append(ToolSceneManager._pose(entry))

    def _attach_scene(self, tool_id: int, spec: dict) -> PlanningScene:
        obj = CollisionObject(id=self._attached_id(tool_id), operation=CollisionObject.ADD)
        obj.header.frame_id = self.get_parameter('attach_link').value
        self._append_primitives(obj, spec['collision']['primitives'])
        attached = AttachedCollisionObject(link_name=self.get_parameter('attach_link').value, object=obj)
        # Remove any stale slot representation atomically with attach. Physical
        # detach is not implemented; this only recovers an old scene artifact.
        remove_slot = CollisionObject(id=self._slot_id(tool_id), operation=CollisionObject.REMOVE)
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        scene.world.collision_objects.append(remove_slot)
        return scene

    def _clear_all_tool_scene(self) -> PlanningScene:
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        for tool_id in self.tools:
            attached = AttachedCollisionObject(link_name=self.get_parameter('attach_link').value)
            attached.object.id, attached.object.operation = self._attached_id(tool_id), CollisionObject.REMOVE
            scene.robot_state.attached_collision_objects.append(attached)
            scene.world.collision_objects.append(CollisionObject(id=self._slot_id(tool_id), operation=CollisionObject.REMOVE))
        return scene

    def _apply(self, scene: PlanningScene, tool_id: int, operation: str) -> None:
        if not self.apply_client.service_is_ready():
            self._publish('rejected:apply_planning_scene_unavailable')
            return
        with self.lock:
            future = self.apply_client.call_async(ApplyPlanningScene.Request(scene=scene))
            future.add_done_callback(lambda future: self._done(future, tool_id, operation))
            self._publish(f'{operation}:pending:{tool_id}')

    def _done(self, future, tool_id: int, operation: str) -> None:
        try:
            success = bool(future.result().success)
        except Exception as exc:
            self.get_logger().error(f'planning-scene update failed: {exc}')
            success = False
        if not success:
            if operation == 'attach':
                self._mark_unknown(f'unknown:planning_scene_{operation}_failed')
            else:
                self._publish(f'{operation}:failed:{tool_id}')
            return
        if operation == 'attach':
            self.active_tool_id, self.pending_tool_id = tool_id, self.NONE
            self.physical_engagement_started = False
            self._publish(f'attached:{tool_id}:after_retreat')
        elif operation == 'recover_empty':
            self.active_tool_id, self.pending_tool_id = self.NONE, self.NONE
            self._publish('recovered:operator_confirmed_empty')
        elif operation == 'fixture_register':
            self._publish('fixtures_registered')

    def _publish(self, state: str) -> None:
        self.active_pub.publish(Int32(data=self.active_tool_id))
        self.status_pub.publish(String(data=state))
        if self.active_tool_id == self.UNKNOWN:
            self.tcp_pub.publish(String(data=json.dumps({'tool_id': self.UNKNOWN, 'state': 'UNKNOWN'})))
            self.payload_pub.publish(String(data=json.dumps({'tool_id': self.UNKNOWN, 'state': 'UNKNOWN'})))
            return
        if self.active_tool_id == self.NONE:
            self.tcp_pub.publish(String(data=json.dumps({'tool_id': self.NONE, 'frame': 'ee_output_link'})))
            self.payload_pub.publish(String(data=json.dumps({'tool_id': self.NONE, 'mass_kg': 0.0})))
            return
        spec = self.tools[self.active_tool_id]
        tcp = dict(spec.get('tcp', {}))
        self.tcp_pub.publish(String(data=json.dumps({'tool_id': self.active_tool_id, **tcp})))
        self.payload_pub.publish(String(data=json.dumps({'tool_id': self.active_tool_id, 'mass_kg': float(spec.get('mass_kg', 0.0))})))
        self._publish_active_tcp_tf()

    def _publish_active_tcp_tf(self) -> None:
        if self.active_tool_id in (self.NONE, self.UNKNOWN):
            return
        spec = self.tools.get(self.active_tool_id, {})
        tcp = dict(spec.get('tcp', {}))
        if tcp.get('frame') and tcp.get('parent') and len(tcp.get('xyz', [])) == 3 and len(tcp.get('quat_xyzw', [])) == 4:
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id, tf.child_frame_id = tcp['parent'], tcp['frame']
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = [float(value) for value in tcp['xyz']]
            q = [float(value) for value in tcp['quat_xyzw']]
            tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = q
            self.tf_pub.sendTransform(tf)


def main() -> None:
    rclpy.init()
    node = ToolSceneManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
