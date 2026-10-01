import json
import math
import sys
import tempfile
import unittest
import os
import shutil
import numpy as np
sys.path.insert(0,os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src/smartcar_localization/scripts'))
from wall_features import match, compose, inverse, save_file, load_file, validate_groups, scan_points


def wall(ident,a,b): return dict(id=ident,start=a,end=b)

def observations(world,normals,pose):
    c,s=math.cos(pose[2]),math.sin(pose[2]);rot=np.array([[c,-s],[s,c]])
    return np.dot(np.asarray(world)-pose[:2],rot),np.dot(normals,rot)


class FeatureTests(unittest.TestCase):
    def test_feature_wins_despite_many_other_wall_points(self):
        walls=[wall(1,[-2,1],[2,1]),wall(2,[2,-2],[2,1])]
        a=np.column_stack((np.linspace(-1.5,1.5,40),np.ones(40)))
        b=np.column_stack((np.full(40,2.),np.linspace(-1.5,.5,40)))
        # Ten times as many returns from an unselected parallel wall.
        clutter=np.column_stack((np.linspace(-1.5,1.5,800),np.full(800,2.)))
        world=np.vstack((a,b,clutter));normals=np.vstack((np.tile([0,1],(40,1)),np.tile([1,0],(40,1)),np.tile([0,1],(800,1))))
        truth=(.1,-.12,.035);points,normals=observations(world,normals,truth)
        fitted,info=match(points,normals,(0,0,0),walls)
        self.assertEqual(info['reason'],'MATCHED')
        np.testing.assert_allclose(fitted,truth,atol=1e-4)
        self.assertLess(info['points'],150)

    def test_nearby_dense_parallel_returns_do_not_outvote_selected_surface(self):
        selected=np.column_stack((np.linspace(-1,1,30),np.full(30,.95)))
        distractor=np.column_stack((np.linspace(-1,1,600),np.full(600,1.15)))
        points=np.vstack((selected,distractor));normals=np.tile([0,1],(630,1))
        fitted,info=match(points,normals,(0,0,0),[wall(1,[-2,1],[2,1])])
        self.assertAlmostEqual(fitted[1],.05,places=6)
        self.assertEqual(info['points'],30)

    def test_parallel_wall_keeps_along_wall_odometry(self):
        world=np.column_stack((np.linspace(-1,1,50),np.ones(50)))
        points,normals=observations(world,np.tile([0,1],(50,1)),(.1,-.1,.04))
        fitted,info=match(points,normals,(.22,0,0),[wall(1,[-3,1],[3,1])])
        self.assertEqual(info['reason'],'PARTIAL');self.assertEqual(info['rank'],2)
        self.assertAlmostEqual(fitted[0],.22,places=6)
        np.testing.assert_allclose(fitted[1:],(-.1,.04),atol=1e-4)

    def test_group_changes_target_wall(self):
        points=np.column_stack((np.linspace(-1,1,40),np.full(40,.9)));normals=np.tile([0,1],(40,1))
        inside,_=match(points,normals,(0,0,0),[wall(1,[-2,1],[2,1])])
        outside,_=match(points,normals,(0,0,0),[wall(1,[-2,.8],[2,.8])])
        self.assertAlmostEqual(inside[1],.1);self.assertAlmostEqual(outside[1],-.1)

    def test_missing_features_and_bad_association_rejected(self):
        p=np.column_stack((np.linspace(-1,1,40),np.ones(40)));n=np.tile([0,1],(40,1))
        self.assertIsNone(match(p,n,(0,0,0),[])[0])
        self.assertIsNone(match(p,n,(0,0,0),[wall(1,[-2,3],[2,3])])[0])
        self.assertIsNone(match(p,np.tile([1,0],(40,1)),(0,0,0),[wall(1,[-2,1],[2,1])])[0])
        self.assertEqual(match(p,n,(0,0,0),[wall(1,[-2,1.28],[2,1.28])])[1]['reason'],'CORRECTION_TOO_LARGE')

    def test_finite_segment_does_not_match_infinite_extension(self):
        p=np.column_stack((np.linspace(3,4,40),np.ones(40)));n=np.tile([0,1],(40,1))
        self.assertIsNone(match(p,n,(0,0,0),[wall(1,[-1,1],[1,1])])[0])

    def test_persistence_and_map_binding(self):
        folder=tempfile.mkdtemp()
        try:
            path=folder+'/walls.json';groups=dict(inside=[wall(7,[0,0],[2,0])],outside=[])
            save_file(path,groups,'map-a','map');self.assertEqual(load_file(path,'map-a','map'),groups)
            with self.assertRaises(ValueError): load_file(path,'map-b','map')
            with self.assertRaises(ValueError): load_file(path,'map-a','odom')
            groups['inside'].append(groups['inside'][0])
            with self.assertRaises(ValueError): validate_groups(groups)

        finally: shutil.rmtree(folder)

    def test_transform_roundtrip(self):
        a=(2,-1,.8);b=(.3,.4,-.2)
        np.testing.assert_allclose(compose(inverse(a),compose(a,b)),b,atol=1e-10)

    def test_scan_invalid_beams_break_surface(self):
        class Scan: pass
        scan=Scan();scan.range_min=.05;scan.range_max=10.;scan.angle_min=-.4;scan.angle_increment=.01
        angles=scan.angle_min+np.arange(80)*scan.angle_increment
        scan.ranges=(2/np.cos(angles)).tolist();scan.ranges[40]=float('inf')
        points,normals=scan_points(scan,(0,0,0))
        self.assertEqual(len(points),75)
        self.assertTrue(np.all(np.isfinite(points)))
        self.assertTrue(np.all(np.abs(normals[:,0])>.999))


if __name__=='__main__': unittest.main()
