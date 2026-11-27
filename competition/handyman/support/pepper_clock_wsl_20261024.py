import json,time
print(json.dumps(dict(utc_ns=time.time_ns(),monotonic_ns=time.monotonic_ns())),flush=True)
