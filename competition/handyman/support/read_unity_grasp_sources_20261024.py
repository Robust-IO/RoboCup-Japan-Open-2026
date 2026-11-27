from pathlib import Path
p=Path('/mnt/d/handyman/handyman-unity2/handyman-unity2/Assets/SIGVerse/Models/Robot/Common/Scripts')
for n in ('GraspingDetector.cs','GripperTriggerTransferer.cs','GraspedObjectFixer.cs','GraspingDetectorParallelGripper.cs'):
    f=p/n
    print('\nSOURCE '+str(f)+'\n'+f.read_text(errors='replace')[:20000],flush=True)
for candidate in p.parents[1].rglob('*.prefab'):
    if 'hsr' in candidate.name.lower():print('PREFAB',candidate,flush=True)
