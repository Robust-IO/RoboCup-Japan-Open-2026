"""Explicit sensor requirements. Missing channels are absent, never fabricated."""
PROFILES={'paired':('rgb','depth','hand'),'rgbd':('rgb','depth'),'hand':('hand',),'state':()}

def channels(profile):
    if profile not in PROFILES:raise ValueError('unknown_observation_profile:'+str(profile))
    return PROFILES[profile]

def anchor_key(profile):
    channels(profile)
    return 'depth' if profile in ('paired','rgbd') else 'hand' if profile=='hand' else 'state'

def stamps(snapshot):
    profile=snapshot.get('observation_profile','paired');required=channels(profile)
    try:
        if not required:return [snapshot['joint_stamp_ns']]
        return [snapshot['streams'][k]['stamp_ns'] for k in required]
    except KeyError as exc:raise ValueError('required_observation_channel_missing') from exc

def covers(actual,requested):return set(channels(requested)).issubset(channels(actual))

def select_profile_stamps(history,now_ns,profile,*,last=None,ready=lambda _:True,joint_stamp_ns=None,processing_reserve_ns=0,capture_after_ns=0):
    """Select real post-stop images. No image-age or cross-camera age expiry."""
    required=channels(profile);last=last or {}
    if type(now_ns) is not int or type(capture_after_ns) is not int:raise ValueError('invalid_capture_clock')
    if type(joint_stamp_ns) is not int or not -20_000_000<=now_ns-joint_stamp_ns<=600_000_000:return None
    if profile=='state':
        value={'state':joint_stamp_ns}
        return value if joint_stamp_ns>last.get('state',0) and joint_stamp_ns>=capture_after_ns and ready(value) else None
    def valid(k,s):
        return type(s) is int and 0<s<=now_ns and s>=capture_after_ns and s>last.get(k,0)
    if profile=='hand':candidates=[{'hand':s} for s in sorted(history.get('hand',()),reverse=True) if valid('hand',s)]
    else:
        candidates=[]
        for d in sorted(history.get('depth',()),reverse=True):
            if not valid('depth',d):continue
            rgb=min((s for s in history.get('rgb',()) if valid('rgb',s)),key=lambda s:abs(s-d),default=None)
            if rgb is None or abs(rgb-d)>30_000_000:continue
            if profile=='rgbd':candidates.append(dict(rgb=rgb,depth=d))
            else:
                hand=max((s for s in history.get('hand',()) if valid('hand',s)),default=None)
                if hand is not None:candidates.append(dict(rgb=rgb,depth=d,hand=hand))
    for selected in candidates:
        if ready(selected):return selected
    return None
