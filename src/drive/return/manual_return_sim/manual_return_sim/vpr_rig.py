"""SIMULATION ONLY: independent truth/odom, virtual RGB-D, production VPR/controller.
No CAN sockets or motor hardware. Lateral slip is deliberately absent from odom.
"""
import json
import math
import time
from pathlib import Path as FilePath
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String, Bool, Float32MultiArray
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from manual_return_sim.vpr_scene import Scene, K, camera_pose


class Rig(Node):
    def __init__(self):
        super().__init__('virtual_vpr_rig')
        cv2.setNumThreads(1)
        self.declare_parameter('resume_apply_correction',True)
        self.declare_parameter('auto_start',True)
        self.armed=bool(self.get_parameter('auto_start').value)
        self.odom=np.zeros(3); self.truth=np.zeros(3); self.cmd=(0.,0.); self.cmd_at=0.
        self.last_tick=time.monotonic()
        self.state=''; self.phase='waiting'; self.started=time.monotonic(); self.phase_at=self.started
        self.reference_ready=False
        self.observations=[];self.report_written=False
        self.slip=0.; self.result='No VPR result yet'; self.correction=np.eye(4); self.validated=False
        self.scene=Scene(); self.trace={k:[] for k in ('truth','odom','corrected')}
        self.pub_odom=self.create_publisher(Odometry,'/odometry/filtered',10)
        self.manual=self.create_publisher(Float32MultiArray,'/motor_speed_cmd_virtual',10)
        self.create_subscription(Bool,'/sim/vpr/start',lambda m:setattr(self,'armed',True) if m.data and self.phase=='waiting' else None,10)
        self.record=self.create_publisher(Bool,'/path/record',10);self.ret=self.create_publisher(Bool,'/path/return',10)
        self.mode=self.create_publisher(String,'/mission/return/control_mode',10)
        self.check=self.create_publisher(Bool,'/visual_landmark/debug_check',10)
        self.metrics=self.create_publisher(String,'/sim/vpr/metrics',10)
        self.markers=self.create_publisher(MarkerArray,'/sim/vpr/markers',10)
        self.paths={k:self.create_publisher(Path,'/sim/vpr/'+k,10) for k in self.trace}
        self.tf=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self)
        self.create_subscription(Twist,'/cmd_vel',self.on_cmd,10)
        self.create_subscription(String,'/mission/return/state',lambda m:setattr(self,'state',m.data),10)
        self.create_subscription(String,'/visual_landmark/localization_event',self.on_event,10)
        self.create_subscription(String,'/visual_landmark/scheduler_event',lambda m:self.get_logger().info('SCHEDULER '+m.data),10)
        self.create_subscription(String,'/visual_landmark/event',lambda m:self.get_logger().info('CAPTURE '+m.data),10)
        self.create_subscription(PoseStamped,'/visual_landmark/correction',self.on_correction,10)
        self.client=self.create_client(SetParameters,'/visual_localization_node/set_parameters');self.future=None
        self.cameras={}
        for key,frame in [('rear','rear_camera_optical_frame'),('arm','arm_camera_color_optical_frame')]:
            prefix='/rear/camera' if key=='rear' else '/arm/camera/color'
            self.cameras[key]=(frame,self.create_publisher(CompressedImage,prefix+'/image_raw/compressed',1),
                              self.create_publisher(CameraInfo,prefix+'/camera_info',1),
                              self.create_publisher(Image,'/sim/vpr/'+key+'/image',1))
        self.depth=self.create_publisher(Image,'/arm/camera/aligned_depth_to_color/image_raw',1)
        camera_transforms=[]
        for rear,frame in [(True,'rear_camera_optical_frame'),(False,'arm_camera_color_optical_frame')]:
            m=camera_pose([0,0,0],rear);t=TransformStamped();t.header.frame_id='base_link';t.child_frame_id=frame
            t.header.stamp=self.get_clock().now().to_msg();t.transform.translation.x=float(m[0,3]);t.transform.translation.z=.65
            # rear optical quaternion Rz(pi) * Ropt; front Ropt.
            q=np.array([-.5,.5,-.5,.5]) if not rear else np.array([-.5,-.5,.5,.5])
            t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=map(float,q)
            camera_transforms.append(t)
        self.static.sendTransform(camera_transforms)
        self.create_timer(.05,self.tick);self.create_timer(.2,self.images);self.create_timer(.2,self.draw)

    def on_cmd(self,msg):
        self.cmd=(msg.linear.x,msg.angular.z);self.cmd_at=time.monotonic()

    def on_event(self,msg):
        data=json.loads(msg.data)
        if data.get('event') in ('reference_loaded','reference_rejected','validated','localized','no_match','rejected','debug_check_rejected'):
            self.result=json.dumps({k:data[k] for k in ('event','trigger','reason','inliers','correction_applied','planar_correction') if k in data})
            self.get_logger().info(self.result)
        if data.get('event')=='reference_loaded':self.reference_ready=True
        if data.get('event') in ('validated','localized'):
            self.observations.append({k:data[k] for k in ('event','trigger','inliers','reprojection_rmse','planar_correction','correction_applied') if k in data})
        if data.get('event')=='validated' and data.get('trigger')=='stationary_debug':self.validated=True

    def on_correction(self,msg):
        a=2*math.atan2(msg.pose.orientation.z,msg.pose.orientation.w)
        self.correction=np.array([[math.cos(a),-math.sin(a),0,msg.pose.position.x],[math.sin(a),math.cos(a),0,msg.pose.position.y],[0,0,1,0],[0,0,0,1]])

    def stage(self,phase):self.phase=phase;self.phase_at=time.monotonic();self.get_logger().info('SIM PHASE: '+phase)

    def tick(self):
        now=time.monotonic();dt=min(.2,now-self.last_tick);self.last_tick=now
        v,w=self.cmd if now-self.cmd_at<.3 else (0.,0.)
        for p in (self.odom,self.truth):
            p[0]+=v*math.cos(p[2])*dt;p[1]+=v*math.sin(p[2])*dt;p[2]=math.atan2(math.sin(p[2]+w*dt),math.cos(p[2]+w*dt))
        if self.phase=='slip' and self.state=='FOLLOW_RETURN_PATH':
            step=min(.08*dt,.35-self.slip);self.truth[1]+=step;self.slip+=step
        stamp=self.get_clock().now().to_msg();self.image_stamp=stamp
        od=Odometry();od.header.stamp=stamp;od.header.frame_id='odom';od.child_frame_id='base_link'
        od.pose.pose.position.x,od.pose.pose.position.y=map(float,self.odom[:2]);od.pose.pose.orientation.z=math.sin(self.odom[2]/2);od.pose.pose.orientation.w=math.cos(self.odom[2]/2)
        od.twist.twist.linear.x=float(v);od.twist.twist.angular.z=float(w);self.pub_odom.publish(od)
        t=TransformStamped();t.header=od.header;t.child_frame_id='base_link';t.transform.translation.x=od.pose.pose.position.x;t.transform.translation.y=od.pose.pose.position.y;t.transform.rotation=od.pose.pose.orientation;self.tf.sendTransform(t)
        if self.state=='FINISHED' and self.phase in ('corrected_return','uncorrected_return'):
            self.stage('complete')
        manual=0.
        if self.phase=='waiting' and self.armed and now-self.started>4:
            self.record.publish(Bool(data=True));self.stage('record_settle')
        elif self.phase=='record_settle' and now-self.phase_at>1.5 and self.reference_ready:self.stage('outbound')
        elif self.phase=='record_settle' and now-self.phase_at>12:self.stage('failed_reference_capture')
        elif self.phase=='outbound':
            if self.odom[0]<4:manual=150.
            else:self.stage('stop_record')
        elif self.phase=='stop_record' and now-self.phase_at>1:
            self.ret.publish(Bool(data=True));self.stage('turn')
        elif self.phase=='turn' and self.state=='FOLLOW_RETURN_PATH':self.stage('slip')
        elif self.phase=='slip' and self.slip>=.35-1e-6:
            self.mode.publish(String(data='MANUAL'));self.stage('stationary_wait')
        elif self.phase=='stationary_wait' and now-self.phase_at>1 and self.state=='RETURN_MANUAL':
            self.check.publish(Bool(data=True));self.stage('stationary_vpr')
        elif self.phase=='stationary_vpr' and now-self.phase_at>7:
            if self.validated:
                if self.client.service_is_ready():
                    request=SetParameters.Request();request.parameters=[Parameter('apply_correction',value=bool(self.get_parameter('resume_apply_correction').value)).to_parameter_msg()]
                    self.future=self.client.call_async(request);self.stage('configure_resume')
            elif now-self.phase_at>15:self.stage('failed_stationary_validation')
        elif self.phase=='configure_resume' and self.future.done():
            response=self.future.result()
            if response and all(r.successful for r in response.results):
                self.mode.publish(String(data='AUTO'));self.stage('corrected_return' if self.get_parameter('resume_apply_correction').value else 'uncorrected_return')
            else:self.stage('failed_parameter_update')
        self.manual.publish(Float32MultiArray(data=[manual,manual]))

    def images(self):
        if not hasattr(self,'image_stamp'):return
        stamp=self.image_stamp
        for key,(frame,rgb,info,raw) in self.cameras.items():
            image,z=self.scene.render(self.truth,key=='rear')
            ci=CameraInfo();ci.header.stamp=stamp;ci.header.frame_id=frame;ci.width=image.shape[1];ci.height=image.shape[0];ci.distortion_model='plumb_bob';ci.k=K.ravel().tolist();ci.d=[0.]*5;info.publish(ci)
            msg=CompressedImage();msg.header=ci.header;msg.format='jpeg';msg.data=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,90])[1].tobytes();rgb.publish(msg)
            im=Image();im.header=ci.header;im.height=ci.height;im.width=ci.width;im.encoding='bgr8';im.step=ci.width*3;im.data=image.tobytes();raw.publish(im)
            if key=='arm':
                dep=Image();dep.header=ci.header;dep.height=ci.height;dep.width=ci.width;dep.encoding='16UC1';dep.step=ci.width*2;dep.data=np.round(z*1000).astype('<u2').tobytes();self.depth.publish(dep)

    def draw(self):
        stamp=self.get_clock().now().to_msg();c=self.correction@np.array([*self.odom[:2],0.,1.])
        for key,p in [('truth',self.truth),('odom',self.odom),('corrected',c)]:
            pose=PoseStamped();pose.header.frame_id='odom';pose.header.stamp=stamp;pose.pose.position.x=float(p[0]);pose.pose.position.y=float(p[1]);pose.pose.orientation.w=1.
            self.trace[key].append(pose);self.trace[key]=self.trace[key][-3000:]
            path=Path();path.header=pose.header;path.poses=self.trace[key];self.paths[key].publish(path)
        metrics=dict(synthetic=True,phase=self.phase,state=self.state,slip_m=float(self.slip),
                     truth=self.truth.tolist(),odom=self.odom.tolist(),corrected=c[:2].tolist(),
                     truth_odom_error_m=float(np.linalg.norm(self.truth[:2]-self.odom[:2])),
                     truth_corrected_error_m=float(np.linalg.norm(self.truth[:2]-c[:2])),
                     observations=self.observations,apply_after_debug=bool(self.get_parameter('resume_apply_correction').value))
        self.metrics.publish(String(data=json.dumps(metrics)))
        if self.phase in ('complete','failed_stationary_validation','failed_parameter_update','failed_reference_capture') and not self.report_written:
            name='/tmp/resku_virtual_vpr_'+('on' if metrics['apply_after_debug'] else 'off')+'.json'
            metrics['trajectories']={k:[[p.pose.position.x,p.pose.position.y] for p in v] for k,v in self.trace.items()}
            FilePath(name).write_text(json.dumps(metrics,indent=2)+'\n');self.report_written=True
            self.get_logger().info('SIM REPORT '+name)
        arr=MarkerArray()
        for i,(p,color) in enumerate([(self.truth,(1.,0.,.3)),(self.odom,(.2,.6,1.)),(c,(1.,1.,0.))]):
            m=Marker();m.header.frame_id='odom';m.header.stamp=stamp;m.ns='poses';m.id=i;m.type=Marker.SPHERE;m.action=Marker.ADD;m.pose.position.x=float(p[0]);m.pose.position.y=float(p[1]);m.pose.position.z=.15+i*.08;m.pose.orientation.w=1.;m.scale.x=m.scale.y=m.scale.z=.15;m.color.r,m.color.g,m.color.b=color;m.color.a=1.;arr.markers.append(m)
        m=Marker();m.header.frame_id='odom';m.header.stamp=stamp;m.ns='status';m.id=10;m.type=Marker.TEXT_VIEW_FACING;m.pose.position.x=1.5;m.pose.position.z=2.;m.pose.orientation.w=1.;m.scale.z=.12;m.color.r=m.color.g=m.color.b=m.color.a=1.
        m.text=f'SYNTHETIC RGB-D / virtual odometry / no CAN\n{self.phase} | {self.state}\nInjected lateral slip={self.slip:.3f}m\ntruth-odom={np.linalg.norm(self.truth[:2]-self.odom[:2]):.3f}m | truth-corrected={np.linalg.norm(self.truth[:2]-c[:2]):.3f}m\n{self.result}'
        arr.markers.append(m);self.markers.publish(arr)


def main(args=None):
    rclpy.init(args=args);node=Rig()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
