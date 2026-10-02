import * as T from 'three';
import {STLLoader} from '../vendor/STLLoader.js';
const nums=(v,fallback)=> (v||fallback).trim().split(/\s+/).map(Number);
export async function buildModel(text){
 const xml=new DOMParser().parseFromString(text,'text/xml');
 if(xml.querySelector('parsererror')||!xml.querySelector('robot'))throw Error('유효한 URDF가 아닙니다.');
 const result={links:[],joints:new Map(),missing:0};
 function origin(el){const xyz=nums(el?.getAttribute('xyz'),'0 0 0'),rpy=nums(el?.getAttribute('rpy'),'0 0 0');return new T.Matrix4().compose(new T.Vector3(...xyz),new T.Quaternion().setFromEuler(new T.Euler(...rpy,'ZYX')),new T.Vector3(1,1,1));}
 for(const joint of xml.querySelectorAll('robot > joint'))if(joint.getAttribute('type')==='fixed')result.joints.set(joint.querySelector('child').getAttribute('link'),{parent:joint.querySelector('parent').getAttribute('link'),matrix:origin(joint.querySelector('origin'))});
 const loader=new STLLoader();
 for(const link of xml.querySelectorAll('robot > link')){
  const group=new T.Group();
  for(const visual of link.querySelectorAll(':scope > visual')){
   const g=visual.querySelector('geometry');if(!g)continue;let shape;
   const box=g.querySelector('box'),cyl=g.querySelector('cylinder'),sphere=g.querySelector('sphere'),mesh=g.querySelector('mesh');
   if(box)shape=new T.BoxGeometry(...nums(box.getAttribute('size'),'1 1 1'));
   else if(cyl){shape=new T.CylinderGeometry(+cyl.getAttribute('radius'),+cyl.getAttribute('radius'),+cyl.getAttribute('length'),24);shape.rotateX(Math.PI/2);}
   else if(sphere)shape=new T.SphereGeometry(+sphere.getAttribute('radius'),24,16);
   else if(mesh){const path=mesh.getAttribute('filename');if(!/^package:\/\/yahboom_vehicle\/meshes\/[\w.-]+\.stl$/i.test(path)){result.missing++;continue;}try{shape=await loader.loadAsync('/static/robot_models/'+path.slice('package://'.length));}catch{result.missing++;continue;}}
   else{result.missing++;continue;}
   const material=visual.querySelector('material');let color=material?.querySelector('color');if(!color&&material?.getAttribute('name'))color=Array.from(xml.querySelectorAll('robot > material')).find(m=>m.getAttribute('name')===material.getAttribute('name'))?.querySelector('color');
   const rgba=nums(color?.getAttribute('rgba'),'.9 .6 .1 1');const object=new T.Mesh(shape,new T.MeshStandardMaterial({color:new T.Color(...rgba.slice(0,3)),opacity:rgba[3],transparent:rgba[3]<1}));object.applyMatrix4(origin(visual.querySelector('origin')));if(mesh)object.scale.set(...nums(mesh.getAttribute('scale'),'1 1 1'));group.add(object);
  }
  result.links.push({frame:link.getAttribute('name'),group});
 }
 return result;
}
export function modelTransform(frame,lookup,joints,seen=new Set()){
 const actual=lookup(frame);if(actual)return actual;if(seen.has(frame))return null;seen.add(frame);const joint=joints.get(frame);if(!joint)return null;const parent=modelTransform(joint.parent,lookup,joints,seen);return parent?parent.clone().multiply(joint.matrix):null;
}
