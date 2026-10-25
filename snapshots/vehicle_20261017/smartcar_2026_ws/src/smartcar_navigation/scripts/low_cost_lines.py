"""Map-bound, atomic persistence for manually drawn navigation lines (Py2/3)."""
import hashlib,json,math,os,tempfile


def map_key(msg):
    o=msg.info.origin;q=o.orientation
    meta=[msg.header.frame_id.lstrip('/'),msg.info.width,msg.info.height,msg.info.resolution,
          o.position.x,o.position.y,o.position.z,q.x,q.y,q.z,q.w]
    digest=hashlib.sha256(json.dumps(meta,separators=(',',':')).encode('ascii'))
    digest.update(bytearray((int(v)+256)%256 for v in msg.data))
    return digest.hexdigest()


def validate(lines):
    result=[]
    if not isinstance(lines,(list,tuple)):raise ValueError('Expected line list')
    for line in lines:
        try:
            (x1,y1),(x2,y2)=line
            values=[float(v) for v in (x1,y1,x2,y2)]
        except (TypeError,ValueError):raise ValueError('Invalid line endpoints')
        if any(math.isnan(v) or math.isinf(v) for v in values) or math.hypot(values[2]-values[0],values[3]-values[1])<.001:
            raise ValueError('Nonfinite or zero-length line')
        result.append((tuple(values[:2]),tuple(values[2:])))
    return result


class LineStore(object):
    def __init__(self,path):self.path=os.path.abspath(os.path.expanduser(path))
    def read(self):
        if not os.path.exists(self.path):return dict(schema_version=1,maps={})
        with open(self.path) as f:data=json.load(f)
        if not isinstance(data,dict) or data.get('schema_version')!=1 or not isinstance(data.get('maps'),dict):
            raise ValueError('Invalid saved low-cost lines file')
        return data
    def load(self,key):
        data=self.read()
        return validate(data['maps'][key]) if key in data['maps'] else None
    def save(self,key,lines):
        if not key:raise ValueError('Wait for map before saving lines')
        lines=validate(lines);data=self.read();data['maps'][key]=lines
        directory=os.path.dirname(self.path)
        if not os.path.isdir(directory):os.makedirs(directory)
        fd,path=tempfile.mkstemp(prefix='.low-cost-lines-',dir=directory)
        try:
            with os.fdopen(fd,'w') as f:
                json.dump(data,f,indent=2);f.flush();os.fsync(f.fileno())
            os.rename(path,self.path)
        finally:
            if os.path.exists(path):os.unlink(path)
