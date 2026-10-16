"""Conservative data screening, NOT visibility or room-coverage verification."""
import numpy as np

def assess_view(rgb,depth):
    result=dict(schema='handyman-view-health-v1',data_usable=False,coverage_verified=False,
                thresholds=dict(min_valid_depth_fraction=.5,min_rgb_std=2.,near_m=.4,far_m=8.))
    if rgb.dtype!=np.uint8 or rgb.ndim!=3 or rgb.shape[2]!=3 or depth.ndim!=2 or rgb.shape[:2]!=depth.shape or not depth.size:
        return dict(result,reason='invalid_image_shape_or_type')
    valid=np.isfinite(depth)&(depth>.4)&(depth<8.)
    fraction=float(valid.mean())
    # Spatial variation, not colour-channel differences in a flat RGB fill.
    spread=float(np.max(np.std(rgb.astype(float),axis=(0,1))))
    result.update(valid_depth_fraction=fraction,rgb_spatial_std=spread)
    if fraction<.5: return dict(result,reason='insufficient_valid_depth')
    if spread<2.: return dict(result,reason='near_uniform_rgb')
    return dict(result,data_usable=True,reason='data_screen_passed')
