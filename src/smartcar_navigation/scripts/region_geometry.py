"""Convex region geometry shared by editor and external navigation (Python 2/3)."""
import math
import hashlib
import json


def validate(points):
    if len(points) != 4 or any(math.isnan(v) or math.isinf(v) for p in points for v in p):
        raise ValueError('Select exactly four finite corners in boundary order')
    crosses = []
    for i in range(4):
        a, b, c = points[i], points[(i+1)%4], points[(i+2)%4]
        crosses.append((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))
    if not (all(v > .001 for v in crosses) or all(v < -.001 for v in crosses)):
        raise ValueError('Corners must form a convex quadrilateral without crossings')
    return points


def intersects(a, b):
    # Separating axis test; touching counts as blocked.
    for poly in (a, b):
        for i, p in enumerate(poly):
            q = poly[(i+1)%len(poly)]
            axis = (p[1]-q[1], q[0]-p[0])
            pa = [x*axis[0]+y*axis[1] for x,y in a]
            pb = [x*axis[0]+y*axis[1] for x,y in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


def fingerprint(msg):
    o = msg.info.origin
    meta = [msg.header.frame_id.lstrip('/'), msg.info.width, msg.info.height,
            msg.info.resolution, o.position.x, o.position.y, o.position.z,
            o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w]
    h = hashlib.sha256(json.dumps(meta).encode('ascii'))
    h.update(bytearray((v+256)%256 for v in msg.data))
    return h.hexdigest()
