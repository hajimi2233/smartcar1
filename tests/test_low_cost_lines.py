import os,sys,tempfile,unittest,json,types
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/smartcar_navigation/scripts'))
from low_cost_lines import LineStore,map_key

class LineStoreTests(unittest.TestCase):
 def test_restart_map_switch_and_persistent_clear(self):
  with tempfile.TemporaryDirectory() as d:
   path=d+'/nested/lines.json';a=[((0.,1.),(2.,3.))];b=[((4.,5.),(6.,7.))]
   store=LineStore(path);self.assertIsNone(store.load('map-a'))
   store.save('map-a',a);store.save('map-b',b)
   self.assertEqual(LineStore(path).load('map-a'),a)
   self.assertEqual(store.load('map-b'),b)
   store.save('map-a',[])
   self.assertEqual(LineStore(path).load('map-a'),[])
   self.assertEqual(store.load('map-b'),b)
 def test_invalid_edit_does_not_replace_saved_data(self):
  with tempfile.TemporaryDirectory() as d:
   store=LineStore(d+'/lines.json');store.save('a',[((0,0),(1,1))]);before=Path(store.path).read_bytes()
   for lines in ([((0,0),(0,0))],[((float('nan'),0),(1,1))],[[1,2]]):
    with self.assertRaises(ValueError):store.save('a',lines)
    self.assertEqual(Path(store.path).read_bytes(),before)
 def test_corrupt_file_is_not_silently_overwritten(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'lines.json';p.write_text('{bad')
   with self.assertRaises(ValueError):LineStore(str(p)).save('a',[])
   self.assertEqual(p.read_text(),'{bad')
 def test_map_identity_includes_geometry_and_cells(self):
  ns=types.SimpleNamespace
  m=ns(header=ns(frame_id='map'),info=ns(width=2,height=1,resolution=.05,origin=ns(position=ns(x=0,y=0,z=0),orientation=ns(x=0,y=0,z=0,w=1))),data=[0,100])
  first=map_key(m);m.data=[100,0];self.assertNotEqual(first,map_key(m))
  m.data=[0,100];m.info.origin.position.x=1;self.assertNotEqual(first,map_key(m))

if __name__=='__main__':unittest.main()
