"""Extend the isolated adapter's pre-extension standoff recovery only."""
import ast,hashlib,json,shutil
from pathlib import Path
b=Path('/__HANDYMAN_PACKAGE__/runtime');p=b/'pepper-dynamic-height-adapter-latest.json';d=json.loads(p.read_text());root=Path(d['adapter'])
shutil.copy2(b/'dynamic_table_height.py',root/'dynamic_table_height.py')
target=root/'pepper_transfer_automation.py';text=target.read_text()
before="                        if not clearance['ready']:raise ValueError('pepper_horizontal_extension_not_outside_table')"
after="""                        from dynamic_table_height import extension_retreat
                        standoff_corrections=0
                        while not clearance['ready']:
                            if standoff_corrections>=5:raise ValueError('pepper_preextension_corrections_exhausted')
                            retreat=extension_retreat(clearance);standoff_corrections+=1
                            actual=runtime.state();backend.stage='withdraw'
                            backend.adjust('preextension-standoff-'+str(standoff_corrections),actual['poses']['map'],
                                [-retreat,0.,0.],actual['joints'][:5],actual['joints'][5])
                            sample=runtime.sample(processing_reserve_ns=0);s=sample['snapshot']
                            plane=measure_table(sample['depth'],s,destination);runtime.fresh_measurement(s)
                            runtime.save(sample,'table-after-standoff-'+str(standoff_corrections))
                            plan=make_plan(sample['depth'],s,destination,plane,held,cfg,task_id)
                            predicted=map_point(s,expected_horizontal_center(plan['palm_height_m'],held['body_minus_palm_horizontal_m']))
                            clearance=extension_clearance(predicted,s,plane,plan)
                            record(dict(event='measured_extension_standoff_repaired',retreat_m=retreat,
                                correction=standoff_corrections,clearance=clearance))
                        report['preextension_standoff_corrections']=standoff_corrections"""
if text.count(before)!=1:raise ValueError('expected_original_extension_gate_required')
target.write_text(text.replace(before,after,1));ast.parse(target.read_text())
names={v['name'] for v in d['files']}|{'pepper_transfer_automation.py'}
d['files']=[dict(name=n,sha256=hashlib.sha256((root/n).read_bytes()).hexdigest()) for n in sorted(names)]
p.write_text(json.dumps(d,indent=2));(root/'dynamic-height-adapter.json').write_text(json.dumps(d,indent=2));print(json.dumps(dict(adapter=str(root),files=len(d['files']),canonical_sources_changed=False,robot_commands_sent=False)))
