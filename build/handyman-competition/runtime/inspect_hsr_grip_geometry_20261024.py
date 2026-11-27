import json,os,sys,subprocess
from pathlib import Path
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');project=Path('/mnt/d/handyman/handyman-unity2/handyman-unity2')
for p in base.glob('*gripp*.json'):
    print('OLD_GRIPPER_REPORT',str(p),p.read_text()[:12000],flush=True)
f=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/geometry/hsr-full-geometry-20261010-203028.json')
if f.is_file():
    data=json.loads(f.read_text());print('FULL_GEOMETRY_KEYS',list(data) if isinstance(data,dict) else type(data).__name__,flush=True)
    def inspect(obj,path=''):
        if isinstance(obj,dict):
            named=[str(obj[k]) for k in ('name','path','hierarchyPath','transformPath','linkName') if k in obj]
            if any('hand_' in s or 'finger' in s or 'gripper' in s for s in named):
                print('HAND_GEOMETRY',path,json.dumps(obj)[:16000],flush=True);return
            for k,v in obj.items():inspect(v,path+'/'+k)
        elif isinstance(obj,list):
            for i,v in enumerate(obj):inspect(v,path+'/'+str(i))
    inspect(data)
prefabs=[]
for directory,dirs,files in os.walk(project/'Assets'):
    dirs[:]=[x for x in dirs if x not in ('Library','Temp','.git','HandymanDatasetGenerator')]
    for name in files:
        p=Path(directory)/name
        if name.endswith('.prefab') and 'hsr' in name.lower():prefabs.append(str(p))
        if name=='HSRSubJointTrajectory.cs':
            content=p.read_text(errors='replace').splitlines()
            for i,line in enumerate(content):
                if any(s in line for s in ('grasped','Grasped','gripper','Gripper','HandMotor','hand_motor','min','max')):
                    print('HSR_CONTROL',str(p),i+1,'\n'.join(content[max(0,i-3):i+8]),flush=True)
print('PREFABS',json.dumps(prefabs),flush=True)
