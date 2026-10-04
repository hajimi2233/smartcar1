import os,sys,unittest,math
import numpy as np
sys.path.insert(0,os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),'src/smartcar_localization/scripts'))
from localization_search import SurfaceField,startup_search

class SearchTests(unittest.TestCase):
    def scene(self):
        x=np.linspace(-2,2,161);y=np.linspace(-1,1,81)
        samples=np.vstack((np.column_stack((x,np.ones(len(x)),np.zeros(len(x)),np.ones(len(x)))),
            np.column_stack((np.full(len(y),2.),y,np.ones(len(y)),np.zeros(len(y)))),
            np.column_stack((x,np.full(len(x),-1.),np.zeros(len(x)),np.ones(len(x))))))
        target=dict(id=1,start=[-2,1],end=[2,1],cells=samples[:,:2].tolist(),samples=samples.tolist(),resolution=.025)
        pose=(.23,-.11,.09);c,s=math.cos(pose[2]),math.sin(pose[2]);rotation=np.array([[c,-s],[s,c]])
        points=np.dot(samples[:,:2]-pose[:2],rotation);normals=np.dot(samples[:,2:],rotation)
        return points,normals,[target],pose

    def test_search_recovers_offset_from_measurements(self):
        points,normals,walls,truth=self.scene()
        result,info=startup_search(points,normals,(truth[0]+.35,truth[1]-.2,truth[2]+.15),walls)
        self.assertIsNotNone(result)
        self.assertLess(np.linalg.norm(np.asarray(result)-truth),.025)

    def test_empty_scan_cannot_confirm_pose(self):
        result,info=startup_search(np.empty((0,2)),np.empty((0,2)),(0,0,0),[])
        self.assertIsNone(result)

    def test_outside_map_is_not_zero_cost(self):
        points,normals,walls,truth=self.scene();field=SurfaceField(walls[0]['samples'])
        scores=field.score(points,np.array([truth,(100,100,0)]))
        self.assertGreater(scores[1],scores[0]+.05)

if __name__=='__main__':unittest.main()
