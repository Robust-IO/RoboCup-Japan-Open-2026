from pathlib import Path
base = Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
tag = '20261101-05'
text = (base / 'handyman_rviz_desktop_connect_20261022.ps1').read_text().replace('20261022', tag)
text = text.replace('Handyman-WSLg-Desktop-' + tag, 'Handyman-WSLg-Desktop-20261101')
(base / ('handyman_rviz_desktop_connect_' + tag + '.ps1')).write_text(text)
text = (base / 'show_dual_camera_20261101.ps1').read_text().replace('20261101-02', tag)
old = '    [pscustomobject]@{Session='
capture = '''    Add-Type -AssemblyName System.Drawing
    $image = New-Object System.Drawing.Bitmap $bounds.Width,$bounds.Height
    $graphics = [System.Drawing.Graphics]::FromImage($image)
    $graphics.CopyFromScreen($bounds.Location,[System.Drawing.Point]::Empty,$bounds.Size)
    $image.Save('D:\\handyman-training-runs\\rviz-primary-screen-20261101-05.png',[System.Drawing.Imaging.ImageFormat]::Png)
    $graphics.Dispose()
    $image.Dispose()
'''
assert old in text
text = text.replace(old, capture + old)
text = text.replace('show_dual_camera_20261101.ps1 -Desktop', 'show_dual_camera_' + tag + '.ps1 -Desktop')
(base / ('show_dual_camera_' + tag + '.ps1')).write_text(text)
