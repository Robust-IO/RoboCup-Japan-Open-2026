"""Object adapters independent of the proven apple entry. No ROS imports."""
import copy
import json
from pathlib import Path

from apple_transfer_auto_plan import validate_config

PROFILE_PATH=Path(__file__).parent/'config/manipulation_objects_v1.json'
BASELINE_PATH=Path(__file__).parent/'config/apple_transfer_auto_v1.json'
GEOMETRIES={'apple':'legacy_apple_red_sphere','rubiks_cube':'upright_cube_rgbd','canned_juice':'upright_cylinder_rgbd',
    'ground_pepper':'round_shaker_rgbd','toy_penguin':'penguin_two_surface_rgbd'}


def object_profile(name, path=PROFILE_PATH):
    catalog=json.loads(Path(path).read_text())
    if catalog.get('version')!=1 or name not in GEOMETRIES or name not in catalog.get('objects',{}):
        raise ValueError('unsupported_manipulation_object:'+str(name))
    profile=copy.deepcopy(catalog['objects'][name])
    if (profile.get('geometry')!=GEOMETRIES[name] or type(profile.get('class_id')) is not int
            or profile['class_id']!= {'apple':0,'rubiks_cube':16,'canned_juice':3,'ground_pepper':11,'toy_penguin':24}[name]
            or profile.get('geometry_confidence')!=.1 or profile.get('identity_confidence')!=.8):
        raise ValueError('invalid_object_adapter_profile')
    if name=='rubiks_cube' and profile.get('side_range_m')!=[.02,.12]:
        raise ValueError('unreviewed_cube_observation_envelope')
    if name=='canned_juice' and (profile.get('radius_range_m')!=[.01,.06] or profile.get('height_range_m')!=[.035,.20]):
        raise ValueError('unreviewed_cylinder_observation_envelope')
    if name=='ground_pepper' and (profile.get('body_radius_range_m')!=[.015,.045]
            or profile.get('cap_radius_range_m')!=[.008,.030] or profile.get('height_range_m')!=[.04,.12]):
        raise ValueError('unreviewed_pepper_observation_envelope')
    if name=='toy_penguin' and (profile.get('body_radius_range_m')!=[.03,.07] or profile.get('head_radius_range_m')!=[.015,.035]
            or profile.get('height_range_m')!=[.08,.17] or profile.get('grasp_part')!='head'):
        raise ValueError('unreviewed_penguin_observation_envelope')
    return dict(profile,name=name)


def transfer_config(name, baseline=None):
    """Keep the original furniture/route/speed policy; vary ONLY the object.

    The returned config is for the generic OFFLINE template, never fed to the
    original apple validator or used as an alternative motion entry.
    """
    baseline=copy.deepcopy(baseline if baseline is not None else json.loads(BASELINE_PATH.read_text()))
    validate_config(baseline)
    profile=object_profile(name)
    return dict(version=1,profile='fixed_route_object_transfer_template_v1',
                object_profile=profile,task=dict(baseline['task'],object=name),
                baseline=baseline,live_motion_supported=False,
                scope='Same LayoutA bedroom white_side_table to living_room square_low_table. Generic offline/observation development only.')


def validate_transfer_config(config):
    rebuilt=transfer_config(config['task']['object'],config['baseline'])
    if config!=rebuilt:raise ValueError('template_configuration_changed_or_unreviewed')
    return config
