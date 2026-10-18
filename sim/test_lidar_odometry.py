import math

import numpy as np
import pytest

from mission_bringup.lidar_odometry import ScanOdometry, rotation


def room():
    t = np.linspace(0., 1., 180)
    return np.concatenate((np.c_[-4+9*t, np.full(180,-3.)],
                           np.c_[-4+9*t, np.full(180,4.)],
                           np.c_[np.full(180,-4.), -3+7*t],
                           np.c_[np.full(180,5.), -3+7*t]))


def test_no_gps_scan_matching_recovers_measured_translation_and_rotation():
    engine=ScanOdometry();world=room()
    engine.update(world, 100.)
    actual=np.array([.14,-.08,math.radians(3)])
    current=(world-actual[:2]) @ rotation(actual[2])
    pose,quality=engine.update(current, 100.1)
    np.testing.assert_allclose(pose,actual,atol=.003)
    assert quality['rmse_m'] < .003


def test_stationary_noise_does_not_accumulate_into_a_rotating_map():
    engine=ScanOdometry();world=room();rng=np.random.default_rng(17)
    engine.update(world,100.)
    for i in range(1,21):
        pose,_=engine.update(world+rng.normal(0,.003,world.shape),100+i*.1)
    assert np.linalg.norm(pose[:2]) < .005
    assert abs(pose[2]) < math.radians(.1)


def test_moving_clutter_does_not_drag_the_map():
    engine=ScanOdometry();world=room();rng=np.random.default_rng(18)
    engine.update(world,100.)
    actual=np.array([.12,.04,.02])
    current=(world-actual[:2]) @ rotation(actual[2])
    current=np.concatenate((current,rng.uniform([-3,-2],[4,3],(70,2))))
    pose,_=engine.update(current,100.1)
    np.testing.assert_allclose(pose,actual,atol=.006)


def test_a_single_wall_is_rejected_instead_of_inventing_odometry():
    engine=ScanOdometry()
    with pytest.raises(ValueError,match='constrain'):
        engine.update(np.c_[np.linspace(-5,5,300),np.ones(300)],100.)


def test_capture_gaps_do_not_get_integrated_as_fresh_motion():
    engine=ScanOdometry();engine.update(room(),100.)
    with pytest.raises(ValueError,match='gap'):
        engine.update(room(),102.)


def test_stationary_scene_can_recover_after_a_gap_without_inventing_motion():
    engine=ScanOdometry();engine.update(room(),100.)
    with pytest.raises(ValueError,match='gap'):engine.update(room(),102.)
    pose,_=engine.update(room(),102.1)
    np.testing.assert_allclose(pose,0,atol=1e-8)


def test_a_moved_scene_is_not_accepted_as_stationary_gap_recovery():
    engine=ScanOdometry();engine.update(room(),100.)
    with pytest.raises(ValueError,match='gap'):engine.update(room(),102.)
    with pytest.raises(ValueError,match='revalidated'):
        engine.update(room()-[.2,0.],102.1)
