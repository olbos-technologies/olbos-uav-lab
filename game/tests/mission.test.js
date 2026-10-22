import test from 'node:test';import assert from 'node:assert/strict';import {newMission,tickMission,BEACONS,DROP} from '../src/mission.js';
test('starts with empty objectives',()=>assert.deepEqual(newMission().scan,[false,false,false]));
test('beacon scan requires proximity and action',()=>{let s=newMission();s=tickMission(s,{x:BEACONS[0].x,y:10,z:BEACONS[0].z},.1,false);assert.equal(s.scan[0],false);s=tickMission(s,{x:BEACONS[0].x,y:10,z:BEACONS[0].z},.1,true);assert.equal(s.scan[0],true)});
test('delivery only after all scans',()=>{let s=tickMission(newMission(),{x:DROP.x,y:3,z:DROP.z},.1,true);assert.equal(s.delivery,false)});
test('can complete an objective',()=>{let s=newMission();for(const b of BEACONS)s=tickMission(s,{x:b.x,y:10,z:b.z},.1,true);s=tickMission(s,{x:DROP.x,y:3,z:DROP.z},.1,true);assert.equal(s.status,'WON')});
test('battery depletion causes failure',()=>{let s=newMission();s.battery=.01;s=tickMission(s,{x:0,y:15,z:0},2,false);assert.equal(s.status,'LOST')});
test('terminal state cannot restart through tick',()=>{let s=newMission();s.status='WON';assert.equal(tickMission(s,{x:0,y:10,z:0},1).status,'WON')});