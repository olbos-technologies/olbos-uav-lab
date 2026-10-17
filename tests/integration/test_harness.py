import json, subprocess, unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
class HarnessTests(unittest.TestCase):
 def run_scenario(self, name):
  result=subprocess.run([str(ROOT/'build/uav_demo'), name],capture_output=True,text=True,check=True)
  rows=[json.loads(line) for line in result.stdout.splitlines()]
  self.assertTrue(all(r['source']=='KINEMATIC_HARNESS' for r in rows))
  self.assertTrue(all(-.01<=r['altitude']<=5.01 for r in rows))
  self.assertLess(rows[-1]['altitude'],.01)
  return rows
 def test_nominal(self):
  rows=self.run_scenario('normal');self.assertEqual(rows[-1]['mode'],'COMPLETE');self.assertEqual(rows[-1]['waypoint'],4)
 def test_battery(self):
  rows=self.run_scenario('battery');self.assertEqual(rows[-1]['mode'],'ABORTED');self.assertIn('LOW_BATTERY',[r['alert'] for r in rows])
 def test_stale(self):
  rows=self.run_scenario('stale');self.assertEqual(rows[-1]['mode'],'ABORTED');self.assertIn('STALE_TELEMETRY',[r['alert'] for r in rows])
 def test_geofence(self):
  rows=self.run_scenario('geofence');self.assertEqual(rows[-1]['mode'],'ABORTED');self.assertIn('GEOFENCE',[r['alert'] for r in rows])
if __name__=='__main__':unittest.main()
