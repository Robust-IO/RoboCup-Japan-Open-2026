import json
from pathlib import Path
p=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/geometry/hsr-full-geometry-20261010-203028.json');d=json.loads(p.read_text())
print('COORDINATES',d.get('coordinates'));parts=d['parts'];print('PARTS_TYPE',type(parts).__name__)
if isinstance(parts,dict):print('PART_KEYS',list(parts)[:8]);parts=list(parts.values())
for part in parts[:2]:
    print('PART_FIELDS',list(part));print('PART_METADATA',json.dumps({k:v for k,v in part.items() if not isinstance(v,list) or len(v)<20})[:5000])
for part in parts:
    names=[str(v) for k,v in part.items() if isinstance(v,str)]
    if any('hand_palm' in x or 'finger_tip' in x for x in names):
        print('HAND_PART',json.dumps({k:v for k,v in part.items() if not isinstance(v,list) or len(v)<20})[:8000])
