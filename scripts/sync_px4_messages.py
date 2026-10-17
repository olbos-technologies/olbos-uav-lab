"""Populate a local px4_msgs checkout from exact selected firmware messages.
Usage: python3 scripts/sync_px4_messages.py /absolute/PX4-Autopilot
Run after cloning px4_msgs release/1.16. Never mix main messages with release firmware.
"""
import json,shutil,subprocess,sys
from pathlib import Path
root=Path(__file__).resolve().parents[1];px4=Path(sys.argv[1]).resolve();dest=root/'ros2_ws/src/px4_msgs'
ref=subprocess.check_output(['git','-C',str(px4),'describe','--tags','--exact-match'],text=True).strip()
if ref!='v1.16.0':raise SystemExit('expected PX4 v1.16.0')
if not (dest/'package.xml').exists():raise SystemExit('clone px4_msgs release/1.16 first')
# The package CMake globs msg/*.msg and srv/*.srv. PX4 v1.16 keeps current versioned messages in
# msg/versioned/ (VehicleCommand, VehicleStatus, ...); older versions in msg/px4_msgs_old are excluded.
source=list((px4/'msg').glob('*.msg'))+list((px4/'msg'/'versioned').glob('*.msg'))
services=list((px4/'srv').glob('*.srv'))
if not source:raise SystemExit('missing PX4 message sources')
names=[f.name for f in source+services]
if len(names)!=len(set(names)):raise SystemExit('duplicate message names across msg/ and msg/versioned/')
for kind,files in (('msg',source),('srv',services)):
 (dest/kind).mkdir(exist_ok=True)
 for old in (dest/kind).glob('*.'+kind):old.unlink()
 for item in files:shutil.copy2(item,dest/kind/item.name)
lock={'px4':subprocess.check_output(['git','-C',str(px4),'rev-parse','HEAD'],text=True).strip(),
 'px4_msgs_scaffold':subprocess.check_output(['git','-C',str(dest),'rev-parse','HEAD'],text=True).strip(),'message_files':len(source),'service_files':len(services)}
(root/'config/resolved-sources.json').write_text(json.dumps(lock,indent=2)+'\n');print(json.dumps(lock))
