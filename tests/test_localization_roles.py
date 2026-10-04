import math,os,sys,unittest
import numpy as np
sys.path.insert(0,os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),'src/smartcar_localization/scripts'))
from localization_roles import Policies


class RolesTests(unittest.TestCase):
    def scene(self):
        top=np.column_stack((np.linspace(-1.5,1.5,50),np.full(50,1.)))
        right=np.column_stack((np.full(50,2.),np.linspace(-1.5,.5,50)))
        points=np.vstack((top,right))-[.1,0.]
        normals=np.vstack((np.tile([0,1],(50,1)),np.tile([1,0],(50,1))))
        walls=[dict(id=1,start=[-3,1],end=[3,1]),dict(id=2,start=[2,-2],end=[2,1])]
        return points,normals,walls

    def test_role_split_fills_only_missing_direction(self):
        points,normals,walls=self.scene();engine=Policies((0,0,0),(0,0,0),0)
        poses,details=engine.update(.1,(.12,0,0),points,normals,walls[:1],walls,.105,None)
        self.assertAlmostEqual(poses['wall_lidar'][0],.12,places=4)
        self.assertAlmostEqual(poses['wall_weighted'][0],.105,places=4)
        self.assertAlmostEqual(poses['role_split'][0],.1,places=4)
        self.assertEqual(details['role_split']['state'],'WALL_PLUS_MAP_NULLSPACE')

    def test_amcl_anchor_consumed_once_not_repeatedly(self):
        points,normals,walls=self.scene();engine=Policies((0,0,0),(0,0,0),0)
        anchor=((.1,0,0),(.12,0,0),.1,.001)
        engine.update(.1,(.12,0,0),points,normals,walls,walls,.1,anchor)
        poses,info=engine.update(.2,(.24,0,0),points,normals,walls,walls,.1,anchor)
        self.assertAlmostEqual(poses['amcl_scan'][0],.22)
        self.assertAlmostEqual(poses['amcl_wheel'][0],.2)
        self.assertEqual(info['amcl_wheel']['state'],'PREDICT_ONLY')

    def test_missing_map_does_not_claim_valid_localization(self):
        engine=Policies((0,0,0),(0,0,0),0)
        poses,info=engine.update(.1,(.1,0,0),np.empty((0,2)),np.empty((0,2)),[],[],.1,None)
        self.assertFalse(any(v['valid'] for v in info.values()))
        self.assertAlmostEqual(poses['role_split'][0],.1)

    def test_full_map_rank_preserves_selected_wall_normal(self):
        points,normals,walls=self.scene()
        # A conflicting global horizontal wall must not override selected y=1 surface.
        full=[dict(id=3,start=[-3,1.05],end=[3,1.05]),walls[1]]
        engine=Policies((0,0,0),(0,0,0),0)
        poses,info=engine.update(.1,(.12,0,0),points,normals,walls[:1],full,.105,None)
        self.assertAlmostEqual(poses['role_split'][1],0.,places=5)

if __name__=='__main__':unittest.main()
