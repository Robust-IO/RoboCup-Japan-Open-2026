"""Distinct head-only source views; stop immediately when identity is confirmed."""
import math

def source_head_views():
    pan=math.pi/6
    # Start with the current centered view, then both sides at each main tilt.
    return [(0.,0.),(pan,0.),(-pan,0.),
            (0.,-math.pi/6),(pan,-math.pi/6),(-pan,-math.pi/6),
            (0.,-.8),(pan,-.8),(-pan,-.8),
            (0.,-.15),(0.,-.30),(0.,-.65)]
