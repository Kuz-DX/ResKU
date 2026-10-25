import threading

import pytest

from vision.camera_lanes import (
    Lane,
    LatestFrameSlots,
    next_job,
    parse_pattern,
    validate_lane_assignments,
)


def feed(slots, cameras, value):
    for camera in cameras:
        slots.put(camera, (camera, value))


def test_default_patterns_keep_one_to_one_and_two_to_one_ratios():
    side_slots = LatestFrameSlots()
    side_lane = Lane('lane1', ['left', 'right'])
    side_counts = {'left': 0, 'right': 0}
    for step in range(60):
        feed(side_slots, side_counts, step)
        camera, _owner, _item = next_job(
            side_lane, [side_lane], side_slots, False, float(step)
        )
        side_counts[camera] += 1

    main_slots = LatestFrameSlots()
    main_lane = Lane('lane2', ['drive', 'drive', 'arm'])
    main_counts = {'drive': 0, 'arm': 0}
    for step in range(60):
        feed(main_slots, main_counts, step)
        camera, _owner, _item = next_job(
            main_lane, [main_lane], main_slots, False, float(step)
        )
        main_counts[camera] += 1

    assert side_counts == {'left': 30, 'right': 30}
    assert main_counts == {'drive': 40, 'arm': 20}


def test_missing_arm_does_not_stop_drive_processing():
    slots = LatestFrameSlots()
    lane = Lane('lane2', ['drive', 'drive', 'arm'])

    selected = []
    for step in range(20):
        slots.put('drive', step)
        job = next_job(lane, [lane], slots, False, float(step))
        assert job is not None
        selected.append(job[0])

    assert selected == ['drive'] * 20


def test_lane_max_fps_is_applied_per_camera():
    slots = LatestFrameSlots()
    lane = Lane('lane1', ['left', 'right'], max_fps=10.0)

    feed(slots, ['left', 'right'], 0)
    first = next_job(lane, [lane], slots, False, 1.00)
    second = next_job(lane, [lane], slots, False, 1.01)
    assert first is not None and first[0] == 'left'
    assert second is not None and second[0] == 'right'

    feed(slots, ['left', 'right'], 1)
    assert next_job(lane, [lane], slots, False, 1.05) is None
    third = next_job(lane, [lane], slots, False, 1.10)
    fourth = next_job(lane, [lane], slots, False, 1.11)
    assert third is not None and third[0] == 'left'
    assert fourth is not None and fourth[0] == 'right'


def test_disabling_side_cameras_empties_lane1_pattern():
    enabled = {'drive', 'arm'}

    assert parse_pattern('left,right', enabled) == []
    assert parse_pattern('drive,drive,arm', enabled) == ['drive', 'drive', 'arm']


def test_work_stealing_off_and_on():
    lane1 = Lane('lane1', ['left'])
    lane2 = Lane('lane2', ['drive', 'arm'])
    lanes = [lane1, lane2]
    slots = LatestFrameSlots()
    slots.put('drive', 'frame-1')

    assert next_job(lane1, lanes, slots, False, 1.0) is None
    stolen = next_job(lane1, lanes, slots, True, 1.0)
    assert stolen is not None
    assert stolen[0] == 'drive'
    assert stolen[1] is lane2
    assert stolen[2] == 'frame-1'

    # Stealing advanced the owner's cursor, so its next weighted turn is arm.
    slots.put('drive', 'frame-2')
    slots.put('arm', 'frame-3')
    owned = next_job(lane2, lanes, slots, False, 2.0)
    assert owned is not None and owned[0] == 'arm'


def test_cross_lane_camera_assignment_is_rejected():
    with pytest.raises(ValueError, match='assigned to both'):
        validate_lane_assignments(
            {'lane1': ['left', 'drive'], 'lane2': ['drive', 'arm']},
            {'left', 'drive', 'arm'},
        )


def test_pattern_rejects_unknown_camera_name():
    with pytest.raises(ValueError, match='Unknown camera'):
        parse_pattern('left,typo', {'left'})


def test_concurrent_stealing_preserves_single_slot_consumption():
    lane1 = Lane('lane1', ['left'])
    lane2 = Lane('lane2', ['drive'])
    lanes = [lane1, lane2]
    slots = LatestFrameSlots()
    slots.put('drive', 'only-frame')
    barrier = threading.Barrier(3)
    results = []

    def select(worker):
        barrier.wait()
        results.append(next_job(worker, lanes, slots, True, 1.0))

    threads = [
        threading.Thread(target=select, args=(lane1,)),
        threading.Thread(target=select, args=(lane2,)),
    ]
    for thread in threads:
        thread.start()
    barrier.wait()
    for thread in threads:
        thread.join()

    jobs = [result for result in results if result is not None]
    assert len(jobs) == 1
    assert jobs[0][2] == 'only-frame'
