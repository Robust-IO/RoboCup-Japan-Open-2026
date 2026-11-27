"""Produce an offline, frame-bound SVG review; never an online exclusion mask."""
import base64
import hashlib
import json
from pathlib import Path

ROOT=Path('/__HANDYMAN_PACKAGE__/runtime')
OUT=ROOT/'gripper-pose-review-20261005-v2'
# Manually proposed interior patches, not full silhouettes. The gap stays clear.
PATCHES={
    'left_finger':[[198,416],[202,395],[218,385],[220,375],[228,369],[235,370],[235,416]],
    'right_finger':[[257,370],[262,370],[263,389],[274,386],[280,400],[282,416],[258,416]],
    'wrist_housing':[[130,478],[144,438],[159,428],[181,426],[276,426],[296,435],[307,454],[313,478]],
}
# Visual tracing proposals, NOT ground truth. Deliberately separate from core.
OUTLINES={
    'left_finger':[[188,421],[193,405],[197,394],[204,385],[218,375],
                   [220,389],[223,368],[231,365],[237,366],[240,399],[236,421]],
    'right_finger':[[253,420],[253,395],[251,390],[254,384],[254,366],
                    [259,365],[266,369],[267,389],[270,377],[278,384],
                    [282,393],[284,409],[289,425]],
    'wrist_housing':[[122,480],[130,455],[139,435],[149,427],[164,424],
                     [188,421],[233,421],[256,422],[280,424],[293,429],
                     [303,438],[308,451],[311,480]],
}

def main():
    OUT.mkdir(exist_ok=False)
    records=[]
    for label,folder,file in [
        ('水平视角：仅记录未见夹爪，未建立排除区域','gripper-calibration-20261005-initial-v4','000001.png'),
        ('下俯视角：内部区与扩展轮廓','gripper-calibration-20261005-head-down-v4','view-00.png'),
        ('下俯视角：第二张稳定采样','gripper-calibration-20261005-head-down-v4','view-01.png')]:
        p=ROOT/folder/file
        data=p.read_bytes()
        records.append(dict(label=label,source=str(p),sha256=hashlib.sha256(data).hexdigest(),
            image='data:image/png;base64,'+base64.b64encode(data).decode(),
            regions=PATCHES if file.startswith('view') else {},
            outline_proposals=OUTLINES if file.startswith('view') else {}))
    calibration=dict(schema='handyman-self-region-review-v2',approved_for_runtime=False,
        phase='offline_review_only',image_size=[640,480],head_pose_for_down=[0,-.5],
        regions=PATCHES,outline_proposals=OUTLINES,
        core_review='User reports v1 interior patches do not cover floor or objects; not pixel-level GT.',
        outline_review='pending',records=[{k:v for k,v in r.items() if k!='image'} for r in records],
        restrictions=['Hand-picked interior patches, not calibrated full silhouette.',
                      'No interpolation or extrapolation to other joint states.',
                      'Unknown arm pose, held object, or grasp phase must disable future exclusion.',
                      'Latest trial captured two valid frames but failed the three-frame acceptance.',
                      'Exclusion cannot certify absence or unobserved surface coverage.'])
    (OUT/'candidate.json').write_text(json.dumps(calibration,ensure_ascii=False,indent=2))
    page=['<!doctype html><meta charset="utf-8"><title>夹爪姿态区域复核 V2</title><style>body{font:17px sans-serif;margin:24px;max-width:1100px}svg{width:100%;max-width:960px;background:#eee}.patch{fill:#e9303044;stroke:#ef3030;stroke-width:.7}.outline{fill:none;stroke:#00a9dc;stroke-width:1;stroke-dasharray:3 2}button{padding:10px;margin:4px}</style><h1>夹爪姿态区域复核 V2（未上线）</h1><p>红色：保留上版内部区域，你已确认未覆盖地板和物品。青色虚线：新描出的外轮廓建议，尚未确认，不用于过滤。红色之外、青线以内仍属于待复核部分；边缘不保证逐像素准确。两指间空隙保持开放。</p><button onclick="document.querySelectorAll(\'.patch\').forEach(e=>e.style.display=e.style.display===\'none\'?\'\':\'none\')">显示／隐藏红色内部区</button><button onclick="document.querySelectorAll(\'.outline\').forEach(e=>e.style.display=e.style.display===\'none\'?\'\':\'none\')">显示／隐藏青色轮廓</button>']
    for r in records:
        page.append(f'<h2>{r["label"]}</h2><svg viewBox="0 0 640 480"><image href="{r["image"]}" width="640" height="480"/>')
        for name,points in r['regions'].items():
            points=' '.join(f'{x},{y}' for x,y in points)
            page.append(f'<polygon class="patch" points="{points}"><title>{name}</title></polygon>')
        for name,points in r['outline_proposals'].items():
            points=' '.join(f'{x},{y}' for x,y in points)
            page.append(f'<polygon class="outline" points="{points}"><title>{name}: 待确认外轮廓</title></polygon>')
        page.append('</svg>')
    page.append('<p>请重点检查青色外轮廓：是否跨进地板、两指间空隙，是否仍漏掉明显的机器人表面。图像放大不增加原始细节。头部或机械臂运动后不得套用本区域。当前只存档复核，不修改 RGB，也不抑制任何在线检测。</p>')
    (OUT/'index.html').write_text('\n'.join(page),encoding='utf-8')
    print(str(OUT/'index.html'))

if __name__=='__main__':main()
