"""Map-frame corridor classification; no ROS dependency (Python 2/3)."""
import math
from region_geometry import validate


def clearance(point, polygon):
    """Signed distance to nearest edge of a convex polygon, positive inside."""
    area = sum(p[0]*polygon[(i+1)%4][1]-polygon[(i+1)%4][0]*p[1]
               for i, p in enumerate(polygon))
    sign = 1 if area > 0 else -1
    return min(sign*((q[0]-p[0])*(point[1]-p[1])-(q[1]-p[1])*(point[0]-p[0])) /
               math.hypot(q[0]-p[0], q[1]-p[1])
               for p, q in zip(polygon, polygon[1:]+polygon[:1]))


def classify(polygon, base_pose):
    """The localization reference point on or inside the boundary is INSIDE."""
    validate(polygon)
    if len(base_pose) != 3 or any(math.isnan(v) or math.isinf(v) for v in base_pose):
        raise ValueError('Invalid vehicle pose')
    return 'INSIDE' if clearance(base_pose[:2], polygon) >= -1e-9 else 'OUTSIDE'
