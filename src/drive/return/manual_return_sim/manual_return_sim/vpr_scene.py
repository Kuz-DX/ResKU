"""Deterministic textured wall renderer; actual RGB/depth, never a fake PnP result."""
import cv2
import numpy as np

WIDTH, HEIGHT = 320, 240
K = np.array([[210.,0,160],[0,210.,120],[0,0,1.]])
OPTICAL = np.array([[0,0,1],[-1,0,0],[0,-1,0.]])


def camera_pose(pose, rear=False):
    x,y,a = pose
    c,s = np.cos(a),np.sin(a)
    base = np.eye(4); base[:3,:3] = [[c,-s,0],[s,c,0],[0,0,1]]; base[:3,3]=[x,y,0]
    mount=np.eye(4); mount[:3,3]=[-.34 if rear else .15,0,.65]
    mount[:3,:3]=np.diag([-1.,-1.,1.])@OPTICAL if rear else OPTICAL
    return base@mount


class Scene:
    def __init__(self):
        rng=np.random.default_rng(421)
        self.texture=np.full((800,1600,3),210,np.uint8)
        for i in range(1500):
            u,v=rng.integers([0,0],[1600,800]); radius=int(rng.integers(3,15))
            color=tuple(map(int,rng.integers(0,240,3)))
            cv2.circle(self.texture,(int(u),int(v)),radius,color,-1)
        for i in range(90):
            cv2.putText(self.texture,str(i),(int(rng.integers(0,1500)),int(rng.integers(30,780))),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,0),2)
        self.corners=np.array([[-4,6,4],[-4,-6,4],[-4,-6,-1],[-4,6,-1]],float)
        u,v=np.meshgrid(np.arange(WIDTH),np.arange(HEIGHT))
        self.rays=np.stack([(u-160)/210,(v-120)/210,np.ones_like(u)],axis=-1).astype(np.float32)

    def render(self, pose, rear=False):
        wc=camera_pose(pose,rear); cw=np.linalg.inv(wc)
        xyz=self.corners@cw[:3,:3].T+cw[:3,3]
        image=np.full((HEIGHT,WIDTH,3),35,np.uint8); depth=np.zeros((HEIGHT,WIDTH),np.float32)
        if np.min(xyz[:,2]) <= 0:
            return image,depth
        uv=(xyz@K.T); uv=uv[:,:2]/uv[:,2,None]
        h=cv2.getPerspectiveTransform(np.float32([[0,0],[1599,0],[1599,799],[0,799]]),uv.astype(np.float32))
        image=cv2.warpPerspective(self.texture,h,(WIDTH,HEIGHT),borderValue=(35,35,35))
        rx,ry=self.rays[:,:,0],self.rays[:,:,1]
        rayx=rx*wc[0,0]+ry*wc[0,1]+wc[0,2]
        with np.errstate(divide='ignore',invalid='ignore'):
            z=(-4-wc[0,3])/rayx
        wy=wc[1,3]+z*(rx*wc[1,0]+ry*wc[1,1]+wc[1,2])
        wz=wc[2,3]+z*(rx*wc[2,0]+ry*wc[2,1]+wc[2,2])
        valid=(z>0)&(z<20)&(abs(wy)<=6)&(wz>=-1)&(wz<=4)
        depth[valid]=z[valid]
        return image,depth
