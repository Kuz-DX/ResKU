import cv2
import numpy as np
import pytest
from visual_landmark.extrinsics import board_points, surveyed_board, optical_candidates


def test_surveyed_board_recovers_base_optical_transform_without_housing_offset():
    points=board_points()
    k=np.array([[1000.,0,640],[0,1000.,360],[0,0,1.]])
    d=np.array([-.2,.04,.001,-.001,0.])
    rotation=np.array([.15,-.2,.05]); translation=np.array([-.05,-.03,1.])
    camera_board=np.eye(4)
    camera_board[:3,:3]=cv2.Rodrigues(rotation)[0];camera_board[:3,3]=translation
    base_camera=np.eye(4)
    base_camera[:3,:3]=np.array([[0,0,-1],[1,0,0],[0,-1,0]])
    base_camera[:3,3]=[-.34,0,.65]
    base_board=base_camera@camera_board
    world=points@base_board[:3,:3].T+base_board[:3,3]
    survey=surveyed_board(world[0],world[8],world[45])
    np.testing.assert_allclose(survey,base_board,atol=1e-12)
    corners=cv2.projectPoints(points,rotation,translation,k,d)[0]
    candidates=optical_candidates(corners,k,d,survey)
    assert len(candidates)>=1
    np.testing.assert_allclose(candidates[0]['base_from_camera'],base_camera,atol=1e-6)
    assert candidates[0]['reprojection_rms_px']<1e-6
    assert optical_candidates(corners,k,d)[0]['base_from_camera'] is None


@pytest.mark.parametrize('px,py', [([1,0,0],[0,.075,0]),([.12,0,0],[.075,0,0]),([None,0,0],[0,.075,0])])
def test_bad_or_unmeasured_survey_rejected(px,py):
    with pytest.raises(ValueError):surveyed_board([0,0,0],px,py)


def test_missing_corners_or_invalid_survey_matrix_rejected():
    with pytest.raises(ValueError):optical_candidates(np.zeros((2,2)),np.eye(3),np.zeros(5))
    with pytest.raises(ValueError):optical_candidates(np.zeros((54,2)),np.eye(3),np.zeros(5),np.zeros((4,4)))
