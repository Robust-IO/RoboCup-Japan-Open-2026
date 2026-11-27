"""Resolve package-local absolute references after a directory relocation."""
from pathlib import Path
import os,json,fcntl
SKIP={'audit','runs','logs','.git','__pycache__','unity-reference','.tmp'}
def relocate(root):
 root=Path(root).resolve()
 with (root/'.package-paths.lock').open('a') as lock:
  fcntl.flock(lock,fcntl.LOCK_EX)
  state=root/'.package-root.json'
  previous=json.loads(state.read_text())['root']
  if previous==str(root):return 0
  changed=0
  for base,dirs,files in os.walk(root):
   dirs[:]=[d for d in dirs if d not in SKIP and not (Path(base)/d).is_symlink()]
   for name in files:
    p=Path(base)/name
    if p==state or p.is_symlink() or p.stat().st_size>8_000_000:continue
    raw=p.read_bytes()
    if b'\0' in raw or previous.encode() not in raw:continue
    try:source=raw.decode('utf-8')
    except UnicodeError:continue
    # All package references share this prefix, including ROS hooks and venv scripts.
    updated=source.replace(previous+'/',str(root)+'/').replace("'"+previous+"'","'"+str(root)+"'").replace('"'+previous+'"','"'+str(root)+'"')
    if updated!=source:
     tmp=p.with_name(p.name+'.relocating')
     tmp.write_text(updated);tmp.chmod(p.stat().st_mode);tmp.replace(p);changed+=1
  temp=state.with_suffix('.new')
  temp.write_text(json.dumps({'root':str(root)}));temp.replace(state)
  print(json.dumps({'package_root':str(root),'relocated_files':changed}),flush=True)
  return changed
if __name__=='__main__':relocate(Path(__file__).resolve().parent)
