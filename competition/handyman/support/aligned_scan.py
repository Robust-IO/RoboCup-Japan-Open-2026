"""Select newest still-fresh scan having all required capture-time transforms."""


def select_aligned(scans, now_ns, stamp_ns, transform_available):
    for scan in sorted(scans, key=stamp_ns, reverse=True):
        age=(now_ns-stamp_ns(scan))/1e9
        if not -.02<=age<=.4:continue
        if transform_available(scan):return scan
    raise RuntimeError('no_fresh_aligned_scan')
