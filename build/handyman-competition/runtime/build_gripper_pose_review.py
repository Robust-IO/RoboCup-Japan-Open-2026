"""Produce an offline, frame-bound SVG review; never an online exclusion mask."""
import base64
import hashlib
import json
from pathlib import Path

ROOT=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
OUT=ROOT/'gripper-pose-review-20261005-v1'
# Manually proposed interior patches, not full silhouettes. The gap stays clear.
PATCHES={
    'left_finger':[[198,416],[202,395],[218,385],[220,375],[228,369],[235,370],[235,416]],
    'right_finger':[[257,370],[262,370],[263,389],[274,386],[280,400],[282,416],[258,416]],
    'wrist_housing':[[130,478],[144,438],[159,428],[181,426],[276,426],[296,435],[307,454],[313,478]],
}

def main():
    OUT.mkdir(exist_ok=False)
    records=[]
    for label,folder,file in [
        ('水平视角：仅记录未见夹爪，未建立排除区域','gripper-calibration-20261005-initial-v4','000001.png'),
        ('下俯视角：候选内部区域，需人工复核','gripper-calibration-20261005-head-down-v4','view-00.png'),
        ('下俯视角：第二张稳定采样','gripper-calibration-20261005-head-down-v4','view-01.png')]:
        p=ROOT/folder/file
        data=p.read_bytes()
        records.append(dict(label=label,source=str(p),sha256=hashlib.sha256(data).hexdigest(),
            image='data:image/png;base64,'+base64.b64encode(data).decode(),
            regions=PATCHES if file.startswith('view') else {}))
    calibration=dict(schema='handyman-self-region-review-v1',approved_for_runtime=False,
        phase='offline_review_only',image_size=[640,480],head_pose_for_down=[0,-.5],
        regions=PATCHES,records=[{k:v for k,v in r.items() if k!='image'} for r in records],
        restrictions=['Hand-picked interior patches, not calibrated full silhouette.',
                      'No interpolation or extrapolation to other joint states.',
                      'Unknown arm pose, held object, or grasp phase must disable future exclusion.',
                      'Latest trial captured two valid frames but failed the three-frame acceptance.',
                      'Exclusion cannot certify absence or unobserved surface coverage.'])
    (OUT/'candidate.json').write_text(json.dumps(calibration,ensure_ascii=False,indent=2))
    page=['<!doctype html><meta charset="utf-8"><title>夹爪姿态区域复核</title><style>body{font:17px sans-serif;margin:24px;max-width:1000px}svg{width:100%;max-width:800px;background:#eee}.patch{fill:#e9303044;stroke:#ef3030;stroke-width:1}button{padding:10px}</style><h1>夹爪姿态区域复核（未上线）</h1><p>红色区域仅为人工提出的内部候选区，不是完整轮廓。左右夹爪之间的空隙保留，不整块裁去底部。不能用于夹持物体时的过滤。</p><button onclick="document.querySelectorAll(\'.patch\').forEach(e=>e.style.display=e.style.display===\'none\'?\'\':\'none\')">显示／隐藏候选区域</button>']
    for r in records:
        page.append(f'<h2>{r["label"]}</h2><svg viewBox="0 0 640 480"><image href="{r["image"]}" width="640" height="480"/>')
        for name,points in r['regions'].items():
            points=' '.join(f'{x},{y}' for x,y in points)
            page.append(f'<polygon class="patch" points="{points}"><title>{name}</title></polygon>')
        page.append('</svg>')
    page.append('<p>请检查红色是否只覆盖机器人、是否碰到地板或物品。头部或机械臂运动后不得套用本区域。当前只存档复核，不修改 RGB，也不抑制任何在线检测。</p>')
    (OUT/'index.html').write_text('\n'.join(page),encoding='utf-8')
    print(str(OUT/'index.html'))

if __name__=='__main__':main()
