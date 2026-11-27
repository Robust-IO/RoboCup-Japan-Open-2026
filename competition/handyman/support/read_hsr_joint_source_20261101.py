from pathlib import Path
import json
project=Path('/mnt/d/handyman/handyman-unity2/handyman-unity2/Assets/SIGVerse')
for p in project.rglob('*HSR*Joint*.cs'):
    if 'Editor' in p.parts:continue
    lines=p.read_text(errors='replace').splitlines()
    print('SOURCE',str(p))
    for i,line in enumerate(lines):
        if any(key in line for key in ['WristRoll','wrist_roll','ArmRoll','wrist_flex','WristFlex','arm_flex','minAngle','maxAngle']):
            print(i+1,'\n'.join(lines[max(0,i-2):i+4]))
print('PREFABS',json.dumps([str(p) for p in (project/'Models/Robot/HSR').rglob('*.prefab')]))
