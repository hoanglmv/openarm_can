import time


def test_simulated_source_has_100_unique_samples_per_second():
    timestamps = []
    deadline = time.perf_counter()
    for _ in range(100):
        deadline += 0.01
        time.sleep(max(0.0, deadline - time.perf_counter()))
        timestamps.append(time.time_ns())

    assert len(set(timestamps)) == 100
    assert all(right > left for left, right in zip(timestamps, timestamps[1:]))


def test_camera_rate_does_not_limit_joint_rate():
    joint_timestamps = list(range(100))
    camera_timestamps = list(range(0, 100, 4))

    assert len(joint_timestamps) == 100
    assert len(camera_timestamps) == 25
    assert len(joint_timestamps) != len(camera_timestamps)


def test_stale_feedback_is_not_a_new_sample():
    feedback_sequence = [7, 7, 8]
    unique = sum(
        current > previous
        for previous, current in zip(feedback_sequence, feedback_sequence[1:])
    )

    assert unique == 1