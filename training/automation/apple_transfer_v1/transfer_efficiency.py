"""Non-navigation timing helpers. No ROS imports, publishers or relaxed gates."""
import math
import time


def wait_ready(check, pump, *, timeout=5., stable_s=.15, clock=time.monotonic):
    """Finish once ready stays true; timeout is a bound, not a fixed sleep."""
    if not all(math.isfinite(x) for x in (timeout, stable_s)) or not 0 <= stable_s < timeout:
        raise ValueError('invalid_readiness_window')
    deadline = clock() + timeout
    since = None
    while clock() < deadline:
        ready = check()
        now = clock()
        if now >= deadline:
            break
        if ready:
            if since is None:
                since = now
            if now - since >= stable_s:
                return
        else:
            since = None
        pump(min(.02, max(0., deadline - clock())))
    raise RuntimeError('transfer_readiness_timeout')


def burst_complete(stamps, count, min_span_s):
    """Distinct ordered source frames, not repeated reads of one image."""
    if type(count) is not int or count < 1 or not math.isfinite(min_span_s) or min_span_s < 0:
        raise ValueError('invalid_capture_burst')
    if len(stamps) < count:
        return False
    return (all(type(s) is int and s > 0 for s in stamps)
            and all(a < b for a, b in zip(stamps, stamps[1:]))
            and (stamps[-1] - stamps[0]) / 1e9 >= min_span_s)


def coalesce_wrist_flex(initial, steps):
    """Merge ONLY adjacent wrist/flex moves at constant lift/opening.

    Retain high-clearance lift, lowering, hand and base ordering. No speed change.
    Both joints use one trajectory duration chosen from the slower joint.
    """
    result = []
    before = list(initial)
    index = 0
    while index < len(steps):
        first = steps[index]
        if index + 1 < len(steps):
            second = steps[index + 1]
            mid, end = first['goal'], second['goal']
            changed1 = {i for i in range(6) if before[i] != mid[i]}
            changed2 = {i for i in range(6) if mid[i] != end[i]}
            if (first['kind'] == second['kind'] == 'arm'
                    and (changed1, changed2) in (({1}, {3}), ({3}, {1}))
                    and abs((before[1] + before[3]) - (end[1] + end[3])) < 1e-6):
                result.append(dict(second, goal=list(end), coordinated_wrist_flex=True))
                before = list(end)
                index += 2
                continue
        result.append(dict(first, goal=list(first['goal'])))
        before = list(first['goal'])
        index += 1
    return result
