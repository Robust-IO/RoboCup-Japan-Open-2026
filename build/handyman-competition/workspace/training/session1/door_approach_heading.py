import math

def select_heading(current,pose,goal,door_entry):
    # Only the final 20cm of a door-entry approach. This does not accept XY
    # arrival or bypass static/dynamic swept-footprint checks.
    return goal[2] if door_entry and math.dist(pose[:2],goal[:2])<.20 else current


def approach_command(pose,goal,door_entry):
    if not door_entry or math.dist(pose[:2],goal[:2])>=.20:return None
    dx=goal[0]-pose[0];dy=goal[1]-pose[1];c=math.cos(pose[2]);s=math.sin(pose[2])
    yaw=math.atan2(math.sin(goal[2]-pose[2]),math.cos(goal[2]-pose[2]))
    w=max(-.8,min(.8,1.5*yaw))
    if abs(yaw)>.15:return (0.,0.,w)
    # Holonomic correction: keep the door heading while centering, instead of
    # handing yaw back to a controller that turns toward the nearby point.
    vx=c*dx+s*dy;vy=-s*dx+c*dy
    scale=min(1.,.12/max(math.hypot(vx,vy),1e-9))
    return (vx*scale,vy*scale,w)
