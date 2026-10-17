import asyncio,json,socket,subprocess,sys,time,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
class ReplayTest(unittest.TestCase):
 def test_replay(self):
  try:from websockets.asyncio.client import connect
  except ImportError:self.skipTest('install websockets to test bridge')
  # Dedicated subprocess, bounded startup/send wait; no flight commands.
  fixture=ROOT/'build/replay-test.jsonl'
  fixture.parent.mkdir(exist_ok=True)
  fixture.write_text('\n'.join(json.dumps({'source':'KINEMATIC_HARNESS','time_s':i*.05}) for i in range(3)))
  p=subprocess.Popen([sys.executable,str(ROOT/'scripts/telemetry_ws.py'),str(fixture)],stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
  try:
   ready=False
   for _ in range(30):
    if p.poll() is not None:raise RuntimeError(p.stderr.read().decode())
    try:
     with socket.create_connection(('127.0.0.1',8765),timeout=.1):ready=True;break
    except OSError:time.sleep(.1)
   self.assertTrue(ready)
   async def run():
    async with connect('ws://127.0.0.1:8765',proxy=None) as ws:
     rows=[json.loads(await asyncio.wait_for(ws.recv(),2)) for _ in range(3)]
     self.assertEqual([r['time_s'] for r in rows],[0,.05,.1])
     self.assertTrue(all(r['source']=='KINEMATIC_HARNESS' and r['transport']=='REPLAY' for r in rows))
   asyncio.run(run())
  finally:
   p.terminate();p.wait(timeout=3);p.stderr.close()
if __name__=='__main__':unittest.main()
