#!/usr/bin/env python3
"""Apply physically confirmed tool changes to the MoveIt planning scene."""
import threading

import rclpy
import yaml
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, Int32, String


class ToolAttachmentManager(Node):
    """The UI selects a tool; only physical confirmation changes collisions."""

    def __init__(self):
        super().__init__('tool_attachment_manager')
        for name, default in (
            ('tools_config_file', ''), ('tool_id_topic', '/selected_tool_id'),
            ('docking_complete_topic', '/docking_complete'),
            ('release_complete_topic', '/tool_release_complete'),
            ('attach_link', 'ee_output_link'),
            ('apply_planning_scene_service', '/apply_planning_scene'),
        ):
            self.declare_parameter(name, default)
        self.tools = self._read_registry(self.get_parameter('tools_config_file').value)
        self.pending_tool_id = self.active_tool_id = 0
        self.request_lock = threading.Lock()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.active_pub = self.create_publisher(Int32, '/active_tool_id', latched)
        self.status_pub = self.create_publisher(String, '/tool_attachment_status', latched)
        self.create_subscription(Int32, self.get_parameter('tool_id_topic').value, self._selected_cb, 10)
        self.create_subscription(Bool, self.get_parameter('docking_complete_topic').value, self._docked_cb, 10)
        self.create_subscription(Bool, self.get_parameter('release_complete_topic').value, self._released_cb, 10)
        self.apply_client = self.create_client(
            ApplyPlanningScene, self.get_parameter('apply_planning_scene_service').value)
        self._publish('no_tool')

    @staticmethod
    def _read_registry(filename):
        if not filename:
            raise RuntimeError('tools_config_file is required')
        try:
            with open(filename, encoding='utf-8') as stream:
                return {int(k): v for k, v in (yaml.safe_load(stream) or {}).get('tools', {}).items()}
        except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
            raise RuntimeError('cannot read tools registry') from exc

    def _selected_cb(self, msg):
        tool_id = int(msg.data)
        if tool_id == 0:
            self.pending_tool_id = 0
            self._publish('selection_cleared')
        elif tool_id in self.tools:
            self.pending_tool_id = tool_id
            self._publish(f'pending_physical_dock:{tool_id}')
        else:
            self._publish(f'rejected:unknown_tool:{tool_id}')

    def _docked_cb(self, msg):
        if not msg.data:
            return
        tool_id = self.pending_tool_id
        spec = self.tools.get(tool_id, {})
        if not tool_id:
            self._publish('rejected:no_pending_tool')
        elif self.active_tool_id:
            self._publish(f'rejected:tool_{self.active_tool_id}_still_attached')
        elif not spec.get('enabled', False):
            self._publish(f'rejected:tool_{tool_id}_disabled')
        elif not spec.get('collision', {}).get('primitives'):
            self._publish(f'rejected:tool_{tool_id}_has_no_collision_geometry')
        else:
            self._apply(self._attach_scene(tool_id, spec), tool_id, True)

    def _released_cb(self, msg):
        if msg.data and self.active_tool_id:
            self._apply(self._detach_scene(self.active_tool_id), self.active_tool_id, False)
        elif msg.data:
            self._publish('rejected:no_attached_tool')

    def _object_id(self, tool_id):
        return f'tool_{tool_id}_attached_collision'

    @staticmethod
    def _pose(spec):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = [float(v) for v in spec.get('xyz', [0, 0, 0])]
        q = spec.get('quat_xyzw', [0, 0, 0, 1])
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = [float(v) for v in q]
        return pose

    def _attach_scene(self, tool_id, spec):
        obj = CollisionObject(id=self._object_id(tool_id), operation=CollisionObject.ADD)
        obj.header.frame_id = self.get_parameter('attach_link').value
        for entry in spec['collision']['primitives']:
            primitive = SolidPrimitive()
            if entry['type'] == 'box':
                primitive.type, primitive.dimensions = SolidPrimitive.BOX, [float(v) for v in entry['size']]
            elif entry['type'] == 'cylinder':
                primitive.type = SolidPrimitive.CYLINDER
                primitive.dimensions = [float(entry['length']), float(entry['radius'])]
            else:
                raise ValueError(f"unsupported collision primitive: {entry['type']}")
            obj.primitives.append(primitive)
            obj.primitive_poses.append(self._pose(entry))
        attached = AttachedCollisionObject(link_name=self.get_parameter('attach_link').value, object=obj)
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        return scene

    def _detach_scene(self, tool_id):
        attached = AttachedCollisionObject(link_name=self.get_parameter('attach_link').value)
        attached.object.id, attached.object.operation = self._object_id(tool_id), CollisionObject.REMOVE
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        return scene

    def _apply(self, scene, tool_id, attaching):
        if not self.apply_client.service_is_ready():
            self._publish('rejected:apply_planning_scene_unavailable')
            return
        with self.request_lock:
            future = self.apply_client.call_async(ApplyPlanningScene.Request(scene=scene))
            future.add_done_callback(lambda future: self._done(future, tool_id, attaching))
            self._publish(('attaching:' if attaching else 'detaching:') + str(tool_id))

    def _done(self, future, tool_id, attaching):
        try:
            success = bool(future.result().success)
        except Exception as exc:
            self.get_logger().error(f'Planning-scene update failed: {exc}')
            success = False
        if not success:
            self._publish(('attach_failed:' if attaching else 'detach_failed:') + str(tool_id))
            return
        self.active_tool_id = tool_id if attaching else 0
        if attaching:
            self.pending_tool_id = 0
        self._publish(('attached:' if attaching else 'detached:') + str(tool_id))

    def _publish(self, state):
        self.active_pub.publish(Int32(data=self.active_tool_id))
        self.status_pub.publish(String(data=state))


def main():
    rclpy.init()
    node = ToolAttachmentManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
