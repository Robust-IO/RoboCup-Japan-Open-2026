from pathlib import Path

base = Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
original = (base / 'start_handyman_desktop_rviz_20261022.ps1').read_text()
old = ' | Where-Object { $_.SessionId -eq $shells[0].SessionId }'
assert old in original
text = original.replace('20261022', '20261101').replace(old, '')
# WSL was started by SSH: its host is in session 0. Still require exactly
# one unique current VM across all hosts, and launch into the explorer owner.
(base / 'start_handyman_desktop_rviz_20261101.ps1').write_text(text)
connector = base / 'handyman_rviz_desktop_connect_20261101.ps1'
if not connector.exists():
    connector.write_text((base / 'handyman_rviz_desktop_connect_20261022.ps1').read_text().replace('20261022', '20261101'))
