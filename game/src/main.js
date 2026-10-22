import * as THREE from 'three';
import {BEACONS,DROP,PATROLS,newMission,tickMission} from './mission.js';
import './style.css';
document.querySelector('#app').innerHTML=`
<header><b>◈ OLBOS</b> <span>SHADOW OPERATIONS</span><small>FICTIONAL ARCADE SIMULATION</small></header>
<aside><label>MISSION 01 / SHADOW RECON</label><h2 id="goal">Scan three signal beacons</h2><div class="stats"><section>BATTERY <strong id="battery">100%</strong></section><section>ALTITUDE <strong id="alt">12m</strong></section><section>ALERT <strong id="alert">0%</strong></section><section>SCANNED <strong id="scans">0/3</strong></section></div><p id="hint">Fly to each cyan ring and press E.</p></aside>
<div class="instructions">WASD MOVE · Q/R ALTITUDE · SHIFT BOOST · E INTERACT · C CAMERA · P PAUSE · ENTER RESTART</div>
<footer><span id="status">SYSTEM READY</span><span id="coord"></span></footer>
<div class="overlay" id="overlay"><div class="modal"><label>OLBOS / INTERACTIVE DEMO</label><h1>SHADOW<br><em>OPERATIONS</em></h1><p id="description">Fly through a fictional canyon, scan three beacons, and deliver rescue supplies while avoiding abstract patrol zones.</p><button id="start">START MISSION →</button><small>No real flight controls, GPS, weapon systems or drone connection.</small></div></div>`;
const scene=new THREE.Scene();scene.background=new THREE.Color(0x07111e);scene.fog=new THREE.FogExp2(0x07111e,.006);
const camera=new THREE.PerspectiveCamera(65,innerWidth/innerHeight,.1,650);
const renderer=new THREE.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.setSize(innerWidth,innerHeight);document.body.prepend(renderer.domElement);
scene.add(new THREE.HemisphereLight(0x92bfff,0x12222b,2));
const sunlight=new THREE.DirectionalLight(0xaad5ff,2);sunlight.position.set(-45,100,50);scene.add(sunlight);
const make=(geometry,material,x=0,y=0,z=0)=>{const mesh=new THREE.Mesh(geometry,material);mesh.position.set(x,y,z);scene.add(mesh);return mesh;};
const ground=make(new THREE.PlaneGeometry(420,420),new THREE.MeshStandardMaterial({color:0x1c2e3e}));ground.rotation.x=-Math.PI/2;
scene.add(new THREE.GridHelper(300,30,0x45617a,0x304557));
const stone=new THREE.MeshStandardMaterial({color:0x304658,roughness:.95});
for(let i=0;i<38;i++){const a=i*2.37,r=95+i%6*9,h=18+i*13%42;const rock=make(new THREE.ConeGeometry(12+i%4*4,h,5),stone,Math.cos(a)*r,h/2,Math.sin(a)*r);rock.rotation.y=a;}
const emissive=color=>new THREE.MeshBasicMaterial({color,side:THREE.DoubleSide});
const markers=BEACONS.map(b=>{const ring=make(new THREE.TorusGeometry(8,.4,8,48),emissive(0x35caff),b.x,3,b.z);ring.rotation.x=Math.PI/2;return ring;});
const drop=make(new THREE.CylinderGeometry(10,10,.3,40),new THREE.MeshStandardMaterial({color:0x117f78,emissive:0x07564f}),DROP.x,0,DROP.z);
const patrols=PATROLS.map(p=>{const orb=make(new THREE.OctahedronGeometry(2.5),emissive(0xff555b),p.x,14,p.z);const zone=make(new THREE.RingGeometry(p.r-1,p.r,48),new THREE.MeshBasicMaterial({color:0xf85a64,transparent:true,opacity:.4,side:THREE.DoubleSide}),p.x,.2,p.z);zone.rotation.x=-Math.PI/2;return {orb,zone};});
const drone=new THREE.Group();scene.add(drone);
const body=new THREE.Mesh(new THREE.BoxGeometry(4,.8,2.3),new THREE.MeshStandardMaterial({color:0xc4dce9,metalness:.7,roughness:.3}));drone.add(body);
const props=[];
for(const x of [-3,3]) for(const z of [-2.4,2.4]){const arm=new THREE.Mesh(new THREE.BoxGeometry(7,.17,.2),new THREE.MeshStandardMaterial({color:0x65829a}));arm.position.z=z;drone.add(arm);const blade=new THREE.Mesh(new THREE.BoxGeometry(3,.08,.22),emissive(0x63dfff));blade.position.set(x,.4,z);drone.add(blade);props.push(blade);}
const $=id=>document.getElementById(id);
let state=newMission(),active=false,paused=false,top=false;drone.position.set(0,12,0);
function reset(){state=newMission();drone.position.set(0,12,0);active=true;paused=false;$('overlay').hidden=true;}
$('start').onclick=reset;
const keys=new Set();
addEventListener('keydown',e=>{const k=e.key.toLowerCase();keys.add(k);if(k==='p'&&!e.repeat)paused=!paused;if(k==='c'&&!e.repeat)top=!top;if(k==='enter'&&!e.repeat)reset();});
addEventListener('keyup',e=>keys.delete(e.key.toLowerCase()));
let last=performance.now();
function frame(now){requestAnimationFrame(frame);const dt=Math.min((now-last)/1000,.07);last=now;
if(active&&!paused&&(state.status==='READY'||state.status==='RUNNING')){
const speed=keys.has('shift')?26:16;let dx=Number(keys.has('d'))-Number(keys.has('a'));let dz=Number(keys.has('s'))-Number(keys.has('w'));const len=Math.hypot(dx,dz)||1;dx/=len;dz/=len;
drone.position.x=THREE.MathUtils.clamp(drone.position.x+dx*dt*speed,-90,90);
drone.position.z=THREE.MathUtils.clamp(drone.position.z+dz*dt*speed,-90,90);
drone.position.y=THREE.MathUtils.clamp(drone.position.y+(Number(keys.has('r'))-Number(keys.has('q')))*dt*12,2,60);
drone.rotation.z=THREE.MathUtils.lerp(drone.rotation.z,-dx*.2,.08);
state=tickMission(state,drone.position,dt,keys.has('e'));
}
props.forEach(p=>p.rotation.y+=dt*42);
markers.forEach((m,i)=>{m.material.color.setHex(state.scan[i]?0x39fca7:0x35caff);m.rotation.z+=dt;});
patrols.forEach((p,i)=>{const base=PATROLS[i];p.orb.position.x=base.x+Math.sin(state.time*.35+i*2)*16;p.orb.position.z=base.z+Math.cos(state.time*.30+i*2)*12;p.zone.position.x=p.orb.position.x;p.zone.position.z=p.orb.position.z;});
const pos=drone.position;const target=top?new THREE.Vector3(pos.x,95,pos.z+.01):new THREE.Vector3(pos.x+22,pos.y+24,pos.z+38);camera.position.lerp(target,.07);camera.lookAt(pos);
$('battery').textContent=Math.ceil(state.battery)+'%';$('alt').textContent=Math.round(pos.y)+'m';$('alert').textContent=Math.floor(state.alert)+'%';$('scans').textContent=state.scan.filter(Boolean).length+'/3';
$('goal').textContent=state.scan.every(Boolean)?'Deliver supplies at green pad':'Scan three signal beacons';
$('hint').textContent=state.scan.every(Boolean)?'Reach green pad below 7m and press E':'Approach cyan rings below 20m and press E';
$('status').textContent=paused?'PAUSED':state.status;$('coord').textContent='X '+pos.x.toFixed(0)+' · Z '+pos.z.toFixed(0);
if(active&&(state.status==='WON'||state.status==='LOST')){active=false;$('overlay').hidden=false;$('overlay').querySelector('h1').innerHTML=state.status==='WON'?'MISSION<br><em>COMPLETE</em>':'MISSION<br><em>FAILED</em>';$('description').textContent='Score '+state.score+' · Detection events '+state.detected+' · Battery '+Math.ceil(state.battery)+'%';$('start').textContent='RESTART MISSION →';}
renderer.render(scene,camera);}
requestAnimationFrame(frame);
addEventListener('resize',()=>{camera.aspect=innerWidth/innerHeight;camera.updateProjectionMatrix();renderer.setSize(innerWidth,innerHeight);});
