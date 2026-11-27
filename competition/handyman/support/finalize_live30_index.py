"""Correct historical salt/pepper grouping; never changes predictions."""
import json
from pathlib import Path
root=Path('/__HANDYMAN_PACKAGE__/runtime/live30-pose6-recheck-20261005-v1')
report=json.loads((root/'report.json').read_text())
page=(root/'index.html').read_text()
ids=[]
for c in report['cases']:
    if '/salt-pepper-live-20261003/capture02/' not in c['source']:continue
    c['targets']=['salt','ground_pepper']
    c['target_basis']=['historical_salt_pepper_session_group_requires_box_review']
    ids.append(c['id'])
    page=page.replace(c['id']+' | 无明确目标／负样本',c['id']+' | salt, ground_pepper（历史同框组，框位置待复核）')
for name in ['salt','ground_pepper']:
    report['coverage'][name]=ids
    page=page.replace('<p>'+name+': 缺少可确定目标记录</p>', '<p>'+name+': '+' · '.join(f'<a href="#{i}">{i}</a>' for i in ids)+'</p>')
page=page.replace('<h2>类别索引</h2>','<p>注意：盐／胡椒按历史同框组索引，不是逐框真值。另有3个历史引用 image0.jpg～image2.jpg 未找到，未纳入本次重跑。</p><h2>类别索引</h2>')
(root/'report.json').write_text(json.dumps(report,indent=2))
(root/'index.html').write_text(page,encoding='utf-8')
print('Salt/pepper grouped cases:',ids)
for c in report['cases']:
    for size in [640,1280]:
        a=next(r for r in c['results'] if r['model']=='old' and r['imgsz']==size)
        b=next(r for r in c['results'] if r['model']=='new' and r['imgsz']==size)
        for target in c['targets']:
            old=max((d['confidence'] for d in a['detections'] if d['name']==target),default=0)
            new=max((d['confidence'] for d in b['detections'] if d['name']==target),default=0)
            if abs(new-old)>.15:print(c['id'],size,target,round(old,3),round(new,3))
