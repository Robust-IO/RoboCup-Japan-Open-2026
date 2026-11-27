"""File/CLI preflight only: AST stops entrypoints BEFORE the first ROS import."""
import ast,json,os,sys,tempfile,subprocess
from pathlib import Path
from unittest.mock import patch
ROOT=Path(__file__).resolve().parent

def before_ros(path,argv):
    tree=ast.parse(path.read_text(),filename=str(path))
    main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    cut=next(i for i,n in enumerate(main.body) if isinstance(n,ast.Import) and any(a.name=='rclpy' for a in n.names))
    main.body=main.body[:cut]+[ast.Return(value=ast.Constant(value='pre_ros_checks_passed'))]
    tree.body=[n for n in tree.body if not isinstance(n,ast.If)]
    ast.fix_missing_locations(tree)
    ns={'__name__':'package_pre_ros_probe','__file__':str(path)}
    with patch.object(sys,'argv',[str(path),*argv]),patch.dict(os.environ,ROS_DOMAIN_ID='71',ROS_LOCALHOST_ONLY='1'):
        exec(compile(tree,str(path),'exec'),ns)
        assert ns['main']()=='pre_ros_checks_passed'

def check():
    p=ROOT/'workspace/training/session1'
    from resident_inference import CAPTURE_ROOT
    assert CAPTURE_ROOT.resolve()==Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runs').resolve(), 'resident capture root mismatch'
    from competition_paths import is_run_path
    assert is_run_path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runs/session1/task/test.json')
    assert not is_run_path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runs/../unrelated/test.json')
    assert not is_run_path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runs-other/test.json')
    # No ROS init, command publisher or task execution exists in the probes below.
    with tempfile.TemporaryDirectory(prefix='package-preflight-',dir='/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runs') as tmp:
        tmp=Path(tmp);envelope=tmp/'envelope.json';envelope.write_text(json.dumps({'polygon':[[-.2,-.2],[.2,-.2],[.2,.2],[-.2,.2]],'turn_radius_m':.3}))
        common=['--run','--goal','0','0','0','--semantic-map',str(ROOT/'maps/export/LayoutA/semantic_map.json'),'--output',str(tmp/'navigation.json'),'--carry-envelope',str(envelope)]
        before_ros(p/'carry_navigation_runtime.py',common)
        before_ros(ROOT/'runtime/outcome_finite_adjustment.py',['--run','--simulation-ignore-collision','--reviewed-map-pose','0','0','0','--delta','.1','0','0','--expected-arm','0','0','0','-1.57','0','--holding-opening','.65','--output',str(tmp/'finite.json')])
    from thin_box_clearance import ThinBoxClearance
    physical=ThinBoxClearance();assert physical.hand.parts
    for n in range(1,7):
        cfg=p/('session3-search-config.json' if n==3 else f'session{n}-live-config.json')
        runner=p/('run_session3_search.py' if n==3 else 'run_live_session1.py')
        run=subprocess.run(['/usr/bin/python3',str(runner),'--config',str(cfg)],capture_output=True,text=True,timeout=30)
        assert run.returncode==0,(n,run.stderr)
    print(json.dumps(dict(child_entrypoints='pre_ros_checks_passed',physical_hand='constructed',six_real_cli_checks='passed',robot_commands_sent=False)),flush=True)
if __name__=='__main__':check()
