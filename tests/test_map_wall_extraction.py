import os
import sys
import math
import unittest
import tempfile
import shutil
import numpy as np
sys.path.insert(0,os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src/smartcar_localization/scripts'))
from map_wall_extraction import extract_groups
from wall_features import match, save_file, load_file

class Obj(object): pass

def make_map(grid,yaw=0):
    msg=Obj();msg.info=Obj();msg.info.width=grid.shape[1];msg.info.height=grid.shape[0]
    msg.info.resolution=.05;msg.data=grid.ravel().tolist()
    o=Obj();o.position=Obj();o.position.x=o.position.y=0
    o.orientation=Obj();o.orientation.x=o.orientation.y=0
    o.orientation.z=math.sin(yaw/2);o.orientation.w=math.cos(yaw/2);msg.info.origin=o
    return msg

def selection(y): return dict(inside=[dict(id=1,start=[.3,y],end=[2.5,y])],outside=[])

class ExtractionTests(unittest.TestCase):
    def test_offset_line_selects_same_wall_and_localization_target(self):
        grid=np.zeros((80,80),dtype=int);grid[40,:]=100;msg=make_map(grid)
        for y in (1.85,2.2):
            target=extract_groups(msg,selection(y))['inside'][0]
            self.assertTrue(all(abs(p[1]-2.025)<1e-6 for p in target['cells']))
            points=np.column_stack((np.linspace(.4,2.3,40),np.full(40,1.925)))
            fitted,_=match(points,np.tile([0,1],(40,1)),(0,0,0),[target])
            self.assertAlmostEqual(fitted[1],.1,places=5)

    def test_multiple_parallel_walls_all_selected(self):
        grid=np.zeros((80,80),dtype=int);grid[38,:]=100;grid[46,:]=100
        target=extract_groups(make_map(grid),selection(2.125))['inside'][0]
        self.assertEqual(set(round(p[1],3) for p in target['cells']),set([1.925,2.325]))

    def test_unknown_and_empty_map_rejected(self):
        for grid in (np.zeros((80,80),dtype=int),np.full((80,80),-1,dtype=int)):
            with self.assertRaises(ValueError): extract_groups(make_map(grid),selection(2.))

    def test_rotated_map(self):
        grid=np.zeros((80,80),dtype=int);grid[40,:]=100
        selectors=dict(inside=[dict(id=1,start=[-1.85,.3],end=[-1.85,2.5])],outside=[])
        target=extract_groups(make_map(grid,math.pi/2),selectors)['inside'][0]
        self.assertTrue(all(abs(p[0]+2.025)<1e-6 for p in target['cells']))

    def test_thick_wall_keeps_interior_and_both_surfaces(self):
        grid=np.zeros((80,80),dtype=int);grid[38:47,:]=100
        target=extract_groups(make_map(grid),selection(2.125))['inside'][0]
        self.assertEqual(len(set(round(p[1],3) for p in target['cells'])),9)
        self.assertEqual(set(round(p[1],3) for p in target['samples']),set([1.925,2.325]))
        for face in (1.925,2.325):
            scan=np.column_stack((np.linspace(.4,2.3,40),np.full(40,face-.05)))
            pose,info=match(scan,np.tile([0,1],(40,1)),(0,0,0),[target])
            self.assertAlmostEqual(pose[1],.05,places=4)
            self.assertEqual(info['rank'],2)

    def test_gaps_and_separate_segments_remain_unfilled(self):
        grid=np.zeros((80,80),dtype=int);grid[40,:16]=100;grid[40,40:]=100
        target=extract_groups(make_map(grid),selection(2.))['inside'][0]
        self.assertFalse(any(.8<p[0]<2. for p in target['cells']))

    def test_mask_persistence_and_dense_unselected_returns(self):
        grid=np.zeros((80,80),dtype=int);grid[40,:]=100
        groups=extract_groups(make_map(grid),selection(2.))
        folder=tempfile.mkdtemp()
        try:
            path=folder+'/walls.json'
            save_file(path,groups,'map','map',selectors=selection(2.))
            self.assertEqual(load_file(path,'map','map'),groups)
        finally: shutil.rmtree(folder)
        selected=np.column_stack((np.linspace(.4,2.3,40),np.full(40,1.975)))
        clutter=np.column_stack((np.linspace(.4,2.3,500),np.full(500,3.)))
        points=np.vstack((selected,clutter))
        pose,info=match(points,np.tile([0,1],(540,1)),(0,0,0),groups['inside'])
        self.assertAlmostEqual(pose[1],.05,places=5)
        self.assertEqual(info['points'],40)

    def test_capsule_ends_and_radius(self):
        grid=np.zeros((80,80),dtype=int);grid[40,:]=100
        sel=dict(inside=[dict(id=1,start=[1,2.025],end=[2,2.025],radius=.2)],outside=[])
        target=extract_groups(make_map(grid),sel)['inside'][0]
        xs=[p[0] for p in target['cells']]
        self.assertLess(min(xs),1.);self.assertGreater(max(xs),2.)
        self.assertGreaterEqual(min(xs),.8);self.assertLessEqual(max(xs),2.2)

    def test_curved_wall_selection_not_reduced_to_one_line(self):
        grid=np.zeros((100,100),dtype=int)
        for x in range(20,75):
            y=40+int(6*math.sin((x-20)/55.*math.pi))
            grid[y:y+3,x]=100
        target=extract_groups(make_map(grid),dict(inside=[dict(id=1,start=[1,2.2],end=[3.8,2.2])],outside=[]))['inside'][0]
        self.assertGreater(len(set(round(p[1],3) for p in target['cells'])),3)
        self.assertGreater(len(target['samples']),20)

if __name__=='__main__': unittest.main()
