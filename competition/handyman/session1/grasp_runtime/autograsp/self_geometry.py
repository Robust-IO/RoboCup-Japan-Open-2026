"""Link-local rendered forearm/wrist bounds, transformed at image time.

This is self-observation exclusion, not environment collision geometry. The
gripper still uses its measured physical model for target/table clearance.
"""
import itertools
import json
from pathlib import Path
import numpy as np


class ForearmGeometry:
    def __init__(self,path):
        data=json.loads(Path(path).read_text())
        if data['schema']!='handyman-self-geometry-v1':raise ValueError('unknown_self_geometry_schema')
        self.parts=[]
        for part in data['parts']:
            # Only robot assets; never include a held environment object merely
            # because Unity parented its renderer below a robot link.
            if '/Models/Robot/HSR/' not in part.get('assetPath',''):continue
            v=np.array([[p['x'],p['y'],p['z'],1.] for p in part['vertices']])
            local=np.asarray(part['meshToLinkUnityRowMajor']).reshape(4,4)
            pts=(v@local.T)[:,:3];pts[:,0]*=-1
            bounds=[(float(a),float(b)) for a,b in zip(pts.min(0),pts.max(0))]
            self.parts.append((part['link'],np.array(list(itertools.product(*bounds)))))
        self.links=sorted({link for link,_ in self.parts})

    def boxes(self,transforms):
        from hand_camera_ray import rotation
        return [vertices@rotation(transforms[link]['quaternion']).T+transforms[link]['translation']
                for link,vertices in self.parts]
