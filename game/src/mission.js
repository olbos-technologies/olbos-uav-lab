export const BEACONS = Object.freeze([{x:-45,z:-55},{x:42,z:-20},{x:-32,z:44}]);
export const DROP = Object.freeze({x:62,z:63});
export const PATROLS = Object.freeze([{x:0,z:-35,r:17},{x:25,z:25,r:18}]);
export const newMission = () => ({time:0,battery:100,scan:[false,false,false],delivery:false,detected:0,alert:0,status:'READY',score:0});
export function tickMission(s, pos, dt, scanHeld=false) {
  if (s.status==='WON'||s.status==='LOST') return s;
  const next={...s,scan:[...s.scan],time:s.time+dt,battery:Math.max(0,s.battery-dt*.25),status:'RUNNING'};
  for (let i=0;i<BEACONS.length;i++) {
    const b=BEACONS[i];
    if(scanHeld && !next.scan[i] && Math.hypot(b.x-pos.x,b.z-pos.z)<12 && pos.y<20) next.scan[i]=true;
  }
  let seen=false;
  for (let i=0;i<PATROLS.length;i++) {
    const p=PATROLS[i];
    const px=p.x+Math.sin(next.time*.35+i*2)*16;
    const pz=p.z+Math.cos(next.time*.30+i*2)*12;
    if(Math.hypot(px-pos.x,pz-pos.z)<p.r && pos.y<28) seen=true;
  }
  next.alert=Math.max(0,Math.min(100,next.alert+(seen?dt*17:-dt*12)));
  if(seen && s.alert<60 && next.alert>=60) next.detected++;
  if(next.scan.every(Boolean) && Math.hypot(DROP.x-pos.x,DROP.z-pos.z)<11 && pos.y<7 && scanHeld) next.delivery=true;
  if(next.delivery){next.status='WON';next.score=Math.max(100,Math.round(1500-next.time*2-next.detected*150+next.battery*5));}
  else if(next.battery<=0 || next.alert>=100) next.status='LOST';
  return next;
}