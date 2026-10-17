"""Loopback-only, read-only replay WebSocket. Does not command PX4."""
import argparse, asyncio, json, time
from pathlib import Path
from websockets.asyncio.server import serve
async def main(path, speed):
 rows=[json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]
 if not rows or speed<=0:raise ValueError('nonempty replay and positive speed required')
 async def client(ws):
  for i,row in enumerate(rows):
   row=dict(row,transport='REPLAY',sent_at=time.time())
   await asyncio.wait_for(ws.send(json.dumps(row,allow_nan=False)),timeout=2)
   delay=max(0,min(1,(rows[min(i+1,len(rows)-1)]['time_s']-row['time_s'])/speed))
   await asyncio.sleep(delay)
 async with serve(client,'127.0.0.1',8765,max_size=4096,max_queue=4,write_limit=4096):
  print('Read-only replay ws://127.0.0.1:8765',flush=True)
  await asyncio.Future()
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('file');p.add_argument('--speed',type=float,default=1)
 args=p.parse_args();asyncio.run(main(args.file,args.speed))
