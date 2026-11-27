import ast,hashlib,json,shutil
from pathlib import Path
b=Path('/__HANDYMAN_PACKAGE__/runtime');p=b/'pepper-dynamic-height-adapter-latest.json';d=json.loads(p.read_text());root=Path(d['adapter']);target=root/'pepper_transfer_automation.py'
before='    from pepper_runtime import PepperRuntime';after='    from dynamic_pepper_runtime import DynamicPepperRuntime as PepperRuntime'
s=target.read_text()
if s.count(before)!=1:raise ValueError('expected_original_runtime_import_required')
s=s.replace(before,after,1);ast.parse(s);target.write_text(s);shutil.copy2(b/'dynamic_pepper_runtime.py',root/'dynamic_pepper_runtime.py')
names={v['name'] for v in d['files']}|{'dynamic_pepper_runtime.py'};d['files']=[dict(name=n,sha256=hashlib.sha256((root/n).read_bytes()).hexdigest()) for n in sorted(names)]
p.write_text(json.dumps(d,indent=2));(root/'dynamic-height-adapter.json').write_text(json.dumps(d,indent=2));print(json.dumps(dict(status='stopped_rgbd_runtime_selected',files=len(names),robot_commands_sent=False)))
