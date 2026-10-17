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
# The package CMake globs msg/*.msg. Use only public current messages, not nested old versions.
source=list((px4/'msg').glob('*.msg'))
if not source:raise SystemExit('missing PX4 message sources')
for old in (dest/'msg').glob('*.msg'):old.unlink()
for item in source:shutil.copy2(item,dest/'msg'/item.name)
lock={'px4':subprocess.check_output(['git','-C',str(px4),'rev-parse','HEAD'],text=True).strip(),
 'px4_msgs_scaffold':subprocess.check_output(['git','-C',str(dest),'rev-parse','HEAD'],text=True).strip(),'message_files':len(source)}
(root/'config/resolved-sources.json').write_text(json.dumps(lock,indent=2)+'\n');print(json.dumps(lock))
