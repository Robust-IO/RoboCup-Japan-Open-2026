from pathlib import Path
import re,json
p=Path('/__HANDYMAN_PACKAGE__/unity-reference/Assets/Competition/Handyman/Handyman.unity')
parts=re.split(r'(?m)^--- !u!(\d+) &(-?\d+)(?: stripped)?\s*$',p.read_text());docs={}
for i in range(1,len(parts),3):docs[int(parts[i+1])]=(int(parts[i]),parts[i+2])
game={};trans={};bygame={}
for fid,(kind,body) in docs.items():
 if kind==1:
  m=re.search(r'^  m_Name: (.+)$',body,re.M)
  if m:game[fid]=m.group(1)
 elif kind in (4,224):
  g=re.search(r'm_GameObject: \{fileID: (-?\d+)',body);f=re.search(r'm_Father: \{fileID: (-?\d+)',body)
  if g and f:trans[fid]=(int(g.group(1)),int(f.group(1)));bygame[int(g.group(1))]=fid
rows=[]
for gid,name in game.items():
 if 'penguin' not in name.lower():continue
 tid=bygame.get(gid);ancestry=[];cursor=tid
 while cursor in trans:
  obj,parent=trans[cursor];ancestry.append(game.get(obj,str(obj)));cursor=parent
 descendants=set()
 for t,(g,parent) in trans.items():
  cursor=t
  while cursor in trans:
   if cursor==tid:descendants.add(g);break
   cursor=trans[cursor][1]
 components=[]
 for fid,(kind,body) in docs.items():
  if kind not in (54,64,65,135,136,138,59):continue
  m=re.search(r'm_GameObject: \{fileID: (-?\d+)',body)
  if m and int(m.group(1)) in descendants:
   components.append(dict(type_id=kind,object=game.get(int(m.group(1))),fields=[line.strip() for line in body.splitlines() if any(key in line for key in ('m_Radius:','m_Height:','m_Size:','m_Mass:','m_IsTrigger:','m_Convex:','m_Center:'))]))
 rows.append(dict(name=name,ancestry=ancestry,components=components))
print(json.dumps(rows),flush=True)
