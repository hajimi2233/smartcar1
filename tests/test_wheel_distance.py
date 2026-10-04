import math
import os
import sys
import unittest
import numpy as np
sys.path.insert(0,os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),'src/smartcar_localization/scripts'))
from wheel_distance import PulseBuffer,PulseSimulator,wheel_prediction,distance_prior,prior_residual_jacobian
from wall_features import match


class WheelTests(unittest.TestCase):
    def test_interpolation_signed_distance_and_no_extrapolation(self):
        b=PulseBuffer(1000.)
        for t,n in ((1.,0),(1.1,50),(1.2,0)): b.add(t,n,'a')
        self.assertAlmostEqual(b.delta(1.025,1.075),.025)
        self.assertAlmostEqual(b.delta(1.1,1.2),-.05)
        with self.assertRaises(ValueError): b.delta(1.,1.3)

    def test_gap_epoch_and_discontinuity(self):
        b=PulseBuffer(1000.);b.add(1.,0,'a');b.add(1.4,20,'a')
        with self.assertRaises(ValueError): b.delta(1.,1.4)
        generation=b.generation;b.add(1.5,0,'b');self.assertGreater(b.generation,generation)
        with self.assertRaises(ValueError): b.delta(1.4,1.5)
        with self.assertRaises(ValueError): b.add(1.6,10000,'b')
        with self.assertRaises(ValueError): b.add(1.59,10001,'b')

    def test_simulator_quantization_reverse_bias_and_restart(self):
        s=PulseSimulator(1000.,.05,rear_offset=0.)
        self.assertEqual(s.update(1.,(0,0,0)),0)
        self.assertEqual(s.update(1.1,(.1,0,0)),105)
        self.assertEqual(s.update(1.2,(0,0,0)),0)
        self.assertEqual(s.update(.1,(0,0,0)),0);self.assertEqual(s.epoch,1)

    def test_arc_and_rear_center_reference(self):
        original=(.31,0,0);angle=.1;radius=1.
        result=wheel_prediction(original,angle,radius*angle,.31)
        expected=(math.sin(angle)+.31*math.cos(angle),1-math.cos(angle)+.31*math.sin(angle),angle)
        np.testing.assert_allclose(result,expected,atol=1e-10)
        sensor=PulseSimulator(100000.,rear_offset=.31);sensor.update(1.,original)
        self.assertEqual(sensor.update(1.1,result),10000)
        np.testing.assert_allclose(wheel_prediction(result,-angle,-.1,.31),original,atol=1e-10)

    def test_prior_jacobian(self):
        pose=np.array([.45,.04,.1]);prior=distance_prior((.31,0,0),.14,.31,.01)
        r,j=prior_residual_jacobian(pose,prior)
        for i in range(3):
            shifted=pose.copy();shifted[i]+=1e-6
            numeric=(prior_residual_jacobian(shifted,prior)[0]-r)/1e-6
            self.assertAlmostEqual(numeric,j[i],places=5)

    def test_parallel_wall_longitudinal_constraint(self):
        points=np.column_stack((np.linspace(-1,1,80),np.full(80,1.)))
        normals=np.tile([0,1],(80,1));walls=[dict(id=1,start=[-5,1],end=[5,1])]
        pure,_=match(points,normals,(.2,.02,.005),walls)
        fused,info=match(points,normals,(.2,.02,.005),walls,
                         motion_prior=distance_prior((0,0,0),.1,.0,.005))
        self.assertAlmostEqual(pure[0],.2,places=5)
        self.assertAlmostEqual(fused[0],.1,places=4)
        self.assertEqual(info['rank'],3)

    def test_wrong_encoder_cannot_override_correction_gate(self):
        points=np.column_stack((np.linspace(-1,1,80),np.full(80,1.)))
        pose,info=match(points,np.tile([0,1],(80,1)),(0,0,0),[dict(id=1,start=[-5,1],end=[5,1])],
                        motion_prior=distance_prior((0,0,0),2.,0.,.005))
        self.assertIsNone(pose);self.assertEqual(info['reason'],'CORRECTION_TOO_LARGE')

if __name__=='__main__': unittest.main()
