"""Rear checkerboard preview/capture and offline surveyed-board extrinsic candidates."""
import argparse
import json
from pathlib import Path
import time
import cv2
import numpy as np
import yaml
from visual_landmark.calibration import load_calibration
from visual_landmark.extrinsics import optical_candidates, surveyed_board


def solve(args):
    observation = json.loads(Path(args.observation).read_text())
    survey = yaml.safe_load(Path(args.survey).read_text())
    columns, rows = observation['board_inner_corners']
    square = observation['square_m']
    base_board = surveyed_board(survey['p0_base_m'],survey['px_base_m'],survey['py_base_m'], columns,rows,square)
    candidates = optical_candidates(observation['corners'],np.array(observation['k']),np.array(observation['d']),base_board,columns,rows,square)
    result = dict(status='candidate_only_requires_independent_validation',parent_frame='base_link',
        child_frame=observation['camera_frame'],base_from_board=base_board.tolist(),
        candidates=candidates, survey=survey, observation=str(Path(args.observation).resolve()),
        note='Planar pose has multiple solutions; none is automatically certified or published. Intrinsic is trial quality.')
    Path(args.output).write_text(yaml.safe_dump(result,sort_keys=False))
    print(yaml.safe_dump(result,sort_keys=False))


def preview(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    data,k,d = load_calibration(args.calibration)
    rclpy.init()
    node=Node('rear_extrinsic_preview')
    latest={}
    sub=node.create_subscription(CompressedImage,args.topic,lambda msg:latest.update(msg=msg,arrival=time.monotonic()),qos_profile_sensor_data)
    output=Path(args.output_dir);output.mkdir(parents=True,exist_ok=True)
    frozen=False; image=None; msg=None; corners=None; frame=None; last=0.; count=0
    try:
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.02)
            if not frozen and 'msg' in latest and time.monotonic()-last > .2:
                last=time.monotonic();msg=latest['msg']
                image=cv2.imdecode(np.frombuffer(msg.data,np.uint8),cv2.IMREAD_COLOR)
                corners=None
                if image is None or image.shape[:2] != (data['image_height'],data['image_width']):
                    continue
                gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
                ok,detected=cv2.findChessboardCornersSB(gray,(9,6))
                frame=image.copy()
                if ok:
                    corners=detected.reshape(-1,2)
                    # Select image-upper endpoint; user measures the frozen labelled points,
                    # never assumes an unmarked symmetric board has an intrinsic origin.
                    if sum(corners[0]) > sum(corners[-1]): corners=corners[::-1].copy()
                    cv2.drawChessboardCorners(frame,(9,6),corners.reshape(-1,1,2),True)
                    for index,label,color in [(0,'P0',(0,0,255)),(8,'PX',(255,0,0)),(45,'PY',(0,255,0))]:
                        point=tuple(np.round(corners[index]).astype(int))
                        cv2.circle(frame,point,8,color,2)
                        cv2.putText(frame,label,point,cv2.FONT_HERSHEY_SIMPLEX,.8,color,2)
                    solutions=optical_candidates(corners,k,d)
                    text='BOARD OK | rms '+(f"{solutions[0]['reprojection_rms_px']:.3f}px" if solutions else 'pose failed')
                else: text='Place whole 10x7-square board in view (9x6 inner corners)'
                cv2.putText(frame,text,(15,30),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),2)
            if frame is not None:
                view=frame.copy()
                cv2.putText(view,'SPACE freeze/resume | S save+freeze | Q close',(15,60),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),2)
                if frozen:cv2.putText(view,'FROZEN: measure the EXACT P0/PX/PY corners',(15,90),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,255),2)
                cv2.imshow('Rear extrinsic - 9x6 - 15mm',view)
            key=cv2.waitKey(1)&255
            if key==ord('q'):break
            if key==32:frozen=not frozen
            if key==ord('s') and corners is not None:
                # Saving a frozen frame is allowed for surveying; no claim that it is live.
                if not frozen and time.monotonic()-latest.get('arrival',0) > .5:
                    print('No fresh image; save rejected',flush=True);continue
                frozen=True;count+=1
                prefix=output/f'sample_{time.time_ns()}_{count:03}'
                cv2.imwrite(str(prefix)+'.png',image)
                cv2.imwrite(str(prefix)+'_labels.png',frame)
                record=dict(camera_frame=msg.header.frame_id,image_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                    board_inner_corners=[9,6],square_m=.015,corners=corners.tolist(),k=k.tolist(),d=d.tolist(),
                    calibration_source=str(Path(args.calibration).resolve()),
                    image_file=str(prefix)+'.png',camera_from_board_candidates=optical_candidates(corners,k,d),
                    note='Base extrinsic requires independent survey of the exact labelled physical corners.')
                Path(str(prefix)+'.json').write_text(json.dumps(record,indent=2)+'\n')
                print('SAVED '+str(prefix)+'.json',flush=True)
    finally:
        node.destroy_node();rclpy.try_shutdown();cv2.destroyAllWindows()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='mode',required=True)
    live=commands.add_parser('preview')
    live.add_argument('--calibration',required=True)
    live.add_argument('--topic',default='/rear/camera/image_raw/compressed')
    live.add_argument('--output-dir',required=True)
    offline=commands.add_parser('solve')
    offline.add_argument('--observation',required=True)
    offline.add_argument('--survey',required=True)
    offline.add_argument('--output',required=True)
    args=parser.parse_args()
    if args.mode=='preview':preview(args)
    else:solve(args)


if __name__=='__main__':main()
