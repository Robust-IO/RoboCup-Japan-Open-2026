"""Create an isolated Session1 configuration; never changes shared defaults."""
import argparse
import hashlib
import json
from pathlib import Path
import yaml
from execution_plan import transfer_config

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base',type=Path,required=True);p.add_argument('--environment',type=Path,required=True)
    p.add_argument('--semantic-map',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    environment=yaml.safe_load(a.environment.read_text())
    result=transfer_config(json.loads(a.base.read_text()),environment,str(a.environment.resolve()),hashlib.sha256(a.environment.read_bytes()).hexdigest())
    result['semantic_map']=str(a.semantic_map.resolve())
    semantic=json.loads(a.semantic_map.read_text());ids={t['id'] for t in semantic['destinations']}
    if not {result['source_table_id'],result['destination_id']}<=ids:raise ValueError('source_or_destination_missing')
    with a.output.open('x') as f:json.dump(result,f,ensure_ascii=False,indent=2)
    print(str(a.output))
if __name__=='__main__':main()
