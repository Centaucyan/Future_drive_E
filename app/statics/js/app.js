import * as T from 'three';
import {Navigation} from './navigation.js';
import {buildModel,modelTransform} from './model.js';
import {startDemo} from './demo.js';
import {OrbitControls} from '../vendor/OrbitControls.js';
const $=id=>document.getElementById(id);
let serverConfig={};
try{const response=await fetch('/api/viewer-config');if(response.ok)serverConfig=await response.json();}catch{}
const defaults={url:'',controlApi:serverConfig.control_api_url||'/api/control',fixed:serverConfig.fixed_frame||'map',base:serverConfig.base_frame||'base_footprint',map:'/map',scan:'/scan',path:'/plan',odom:'/odom',tf:'/tf',static:'/tf_static',camera:'/image_raw/compressed',description:'/robot_description',amcl:'/amcl_pose',action:'/navigate_to_pose',cmd:'/cmd_vel',initialpose:'/initialpose',battery:'/battery'};
let profiles=[{...defaults,name:'Robot 1',ip:'.15'},{...defaults,name:'Robot 2',ip:'.17',url:serverConfig.rosbridge_url||'ws://localhost:9090'},{...defaults,name:'Robot 3',ip:'.20'}];
try {const saved=JSON.parse(localStorage.getItem('yahboom-viewer-v1'));if(Array.isArray(saved)&&saved.length===3)profiles=saved.map((p,i)=>{const merged={...profiles[i],...p};if(merged.controlApi==='http://127.0.0.1:8765')merged.controlApi='/api/control';return merged;});}catch{}
const demo=new URLSearchParams(location.search).has('demo');
let demoTimer,modelEpoch=0,modelJoints=new Map(),initialPoseSentAt=0;
let selected=Number(localStorage.getItem('yahboom-selected')||1);if(![0,1,2].includes(selected))selected=1;
let ros,subscriptions=[],epoch=0,last={},messages={},transforms=new Map(),mapMesh,modelLinks=[],mapFitted=false;
const scene=new T.Scene();scene.background=new T.Color('#0d1525');
const camera=new T.PerspectiveCamera(48,1,.01,2000);camera.up.set(0,0,1);camera.position.set(0,-9,13);
const renderer=new T.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));$('viewport').append(renderer.domElement);
const controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;controls.maxPolarAngle=Math.PI*.49;
scene.add(new T.HemisphereLight(0xffffff,0x334466,3));const light=new T.DirectionalLight(0xffffff,2);light.position.set(2,-4,8);scene.add(light);
const grid=new T.GridHelper(100,100,0x35445d,0x202d43);grid.rotation.x=Math.PI/2;scene.add(grid);
const layers={grid,map:new T.Group(),robot:new T.Group(),path:new T.Group(),scan:new T.Group(),tf:new T.Group()};for(const [k,g]of Object.entries(layers))if(k!=='grid')scene.add(g);layers.tf.visible=false;
const body=new T.Mesh(new T.BoxGeometry(.30,.18,.10),new T.MeshStandardMaterial({color:0xf7a817}));body.position.z=.10;layers.robot.add(body);
const arrow=new T.ArrowHelper(new T.Vector3(1,0,0),new T.Vector3(0,0,.19),.45,0x5fb5ff,.13,.08);layers.robot.add(arrow);layers.robot.visible=false;
const pathLine=new T.Line(new T.BufferGeometry(),new T.LineBasicMaterial({color:0x4d9fff}));layers.path.add(pathLine);
const scanPoints=new T.Points(new T.BufferGeometry(),new T.PointsMaterial({color:0xff3535,size:.045,sizeAttenuation:true}));layers.scan.add(scanPoints);
const normalize=s=>(s||'').replace(/^\//,'');
function matrix(p,q){return new T.Matrix4().compose(new T.Vector3(p.x,p.y,p.z),new T.Quaternion(q.x,q.y,q.z,q.w).normalize(),new T.Vector3(1,1,1));}
function transform(frame){const fixed=normalize(profiles[selected].fixed);let f=normalize(frame),m=new T.Matrix4(),visited=new Set();while(f!==fixed){if(!f||visited.has(f))return null;visited.add(f);const edge=transforms.get(f);if(!edge||(!edge.static&&Date.now()-edge.time>5000))return null;m.premultiply(edge.matrix);f=edge.parent;}return m;}
function estimatedBase(){
 const tf=transform(profiles[selected].base);if(tf)return tf;
 const amcl=messages.amcl;if(!amcl||normalize(amcl.header?.frame_id)!==normalize(profiles[selected].fixed))return null;
 return matrix(amcl.pose.pose.position,amcl.pose.pose.orientation);
}
function place(group,mat){group.matrixAutoUpdate=false;group.matrix.copy(mat);}
function disposeGroup(g){for(const child of [...g.children]){child.traverse(o=>{o.geometry?.dispose();if(o.material){for(const m of [].concat(o.material)){m.map?.dispose();m.dispose();}}});g.remove(child);}}
function geometry(object,points){object.geometry.dispose();object.geometry=new T.BufferGeometry();object.geometry.setAttribute('position',new T.Float32BufferAttribute(points,3));object.geometry.computeBoundingSphere();}
function showMap(m){if(!m.info||m.info.width*m.info.height!==m.data?.length||m.data.length>16777216)throw Error('지도 크기/데이터가 유효하지 않습니다.');disposeGroup(layers.map);const {width,height,resolution,origin}=m.info;const bytes=new Uint8Array(width*height*4);for(let i=0;i<m.data.length;i++){const v=m.data[i],shade=v<0?65:Math.round(235-2.15*v);bytes.set([shade,shade,shade,255],i*4);}const texture=new T.DataTexture(bytes,width,height,T.RGBAFormat);texture.needsUpdate=true;texture.magFilter=T.NearestFilter;texture.minFilter=T.NearestFilter;mapMesh=new T.Mesh(new T.PlaneGeometry(width*resolution,height*resolution),new T.MeshBasicMaterial({map:texture,side:T.DoubleSide,transparent:true,opacity:.86}));mapMesh.position.set(width*resolution/2,height*resolution/2,-.025);layers.map.add(mapMesh);messages.mapOrigin=matrix(origin.position,origin.orientation);}
function receive(key,m){if(key==='description'){if(messages.description?.data!==m.data)loadModel(m.data);}
last[key]=Date.now();messages[key]=m;if(key==='tf'||key==='static'){for(const t of m.transforms||[])transforms.set(normalize(t.child_frame_id),{parent:normalize(t.header.frame_id),matrix:matrix(t.transform.translation,t.transform.rotation),time:Date.now(),static:key==='static'});return;}if(key==='map')showMap(m);if(key==='path')geometry(pathLine,(m.poses||[]).flatMap(p=>[p.pose.position.x,p.pose.position.y,p.pose.position.z+.05]));if(key==='scan'){let points=[];for(let i=0;i<m.ranges.length;i++){let r=m.ranges[i];if(Number.isFinite(r)&&r>=m.range_min&&r<=m.range_max){const a=m.angle_min+i*m.angle_increment;points.push(r*Math.cos(a),r*Math.sin(a),.04);}}geometry(scanPoints,points);}if(key==='camera'){if(!/jpeg|jpg|png/i.test(m.format||''))return;const mime=/png/i.test(m.format)?'image/png':'image/jpeg';const data=typeof m.data==='string'?m.data:btoa(Array.from(m.data,c=>String.fromCharCode(c)).join(''));$('camera').src=`data:${mime};base64,${data}`;$('camera').hidden=false;$('cameraEmpty').hidden=true;}}
function reset(){clearDestination();navigation.close();clearInterval(demoTimer);modelEpoch++;initialPoseSentAt=0;modelLinks=[];modelJoints.clear();disposeGroup(layers.robot);const marker=new T.Mesh(new T.BoxGeometry(.30,.18,.1),new T.MeshStandardMaterial({color:0xf7a817}));marker.position.z=.1;layers.robot.add(marker,new T.ArrowHelper(new T.Vector3(1,0,0),new T.Vector3(0,0,.19),.45,0x5fb5ff,.13,.08));$('modelStatus').textContent='기본 표식';last={};messages={};transforms.clear();mapFitted=false;disposeGroup(layers.map);disposeGroup(layers.tf);geometry(pathLine,[]);geometry(scanPoints,[]);layers.robot.visible=false;$('battery').textContent='추정 잔량 —';$('batteryStatus').textContent='수신 대기';$('battery').classList.remove('low');$('speed').textContent='—';$('position').textContent='—';$('camera').hidden=true;$('camera').removeAttribute('src');$('cameraEmpty').hidden=false;}
function connect(){const token=++epoch;for(const s of subscriptions)s.unsubscribe();subscriptions=[];if(ros)ros.close();reset();const p=profiles[selected];$('robotTitle').textContent=p.name;renderRobots();if(demo){document.querySelector('.demo-link').textContent='실제 연결 화면';document.querySelector('.demo-link').href='./';$('connection').textContent='테스트 데이터 · 로봇 미연결';$('notice').textContent='오프라인 표시 테스트';demoTimer=startDemo(receive);fetch('/static/robot_models/yahboom_vehicle/yahboom.urdf').then(r=>r.text()).then(loadModel);return;}if(!p.url){$('connection').textContent='연결 주소 미설정';$('notice').textContent='연결 설정에서 이 로봇의 rosbridge 주소를 입력하세요.';return;}if(!/^wss?:\/\//.test(p.url)){ $('connection').textContent='잘못된 WebSocket 주소';return; }$('connection').textContent='연결 중…';ros=new ROSLIB.Ros({url:p.url});navigation.connect(p.url,p);ros.on('connection',()=>{if(token!==epoch)return;$('connection').textContent='● rosbridge 연결됨';$('notice').textContent='ROS 메시지 수신 대기';const types={map:'nav_msgs/msg/OccupancyGrid',scan:'sensor_msgs/msg/LaserScan',path:'nav_msgs/msg/Path',odom:'nav_msgs/msg/Odometry',tf:'tf2_msgs/msg/TFMessage',static:'tf2_msgs/msg/TFMessage',camera:'sensor_msgs/msg/CompressedImage',description:'std_msgs/msg/String',amcl:'geometry_msgs/msg/PoseWithCovarianceStamped',battery:'std_msgs/msg/UInt16'};for(const[key,type]of Object.entries(types)){if(!p[key])continue;const connection=ros,id=`viewer:${token}:${key}`;const handler=m=>{if(token!==epoch)return;try{receive(key,m);}catch(e){$('notice').textContent=`${key}: ${e.message}`;}};connection.on(p[key],handler);const latched=['map','static','description','amcl'].includes(key);connection.callOnConnection({op:'subscribe',id,topic:p[key],type,throttle_rate:key==='camera'?150:key==='scan'?100:0,queue_length:1,qos:{history:'keep_last',depth:key==='static'?100:5,reliability:latched?'reliable':'best_effort',durability:latched?'transient_local':'volatile'}});subscriptions.push({unsubscribe(){connection.removeListener(p[key],handler);if(connection.isConnected)connection.callOnConnection({op:'unsubscribe',id,topic:p[key]});}});}});ros.on('error',()=>{if(token===epoch){$('connection').textContent='연결 실패';$('notice').textContent='rosbridge 실행 여부와 연결 주소를 확인하세요.';}});ros.on('close',()=>{if(token===epoch){$('connection').textContent='연결 끊김 · 설정에서 재연결';selectionMode(false);navigationReport('연결 끊김 · 로봇 상태를 확인하세요.');}});ros.on('status',s=>{if(token===epoch&&s.level==='error')$('notice').textContent=s.msg;});}
function renderRobots(){$('robots').replaceChildren(...profiles.map((p,i)=>{const b=document.createElement('button');b.textContent=`${p.name} · ${p.ip}`;b.className=i===selected?'selected':'';b.disabled=navigation.busy;b.onclick=()=>{selected=i;localStorage.setItem('yahboom-selected',i);connect();};return b;}));}
const fields={url:'rosbridge 주소',controlApi:'플랫폼 제어 API',fixed:'기준 좌표계',base:'기체 좌표계',map:'지도 토픽',scan:'라이다 토픽',path:'계획 경로 토픽',odom:'속도 토픽',tf:'TF 토픽',static:'고정 TF 토픽',camera:'압축 카메라 토픽',description:'기체 URDF 토픽',amcl:'위치 추정 응답 토픽',action:'Nav2 액션',cmd:'정지 속도 토픽',initialpose:'초기 위치 토픽',battery:'배터리 토픽'};
$('settings').onclick=()=>{if(navigation.busy&&navigation.ready){navigationReport('주행을 정지한 뒤 연결 설정을 변경하세요.');return;} $('configFields').replaceChildren(...Object.entries(fields).map(([k,label])=>{const l=document.createElement('label');l.textContent=label;const i=document.createElement('input');i.name=k;i.value=profiles[selected][k];l.append(i);return l;}));$('configDialog').showModal();};$('cancel').onclick=()=>$('configDialog').close();$('configForm').onsubmit=e=>{e.preventDefault();for(const[k,v]of new FormData(e.target))profiles[selected][k]=v.trim();localStorage.setItem('yahboom-viewer-v1',JSON.stringify(profiles));$('configDialog').close();connect();};
const enabled={grid:true,map:true,robot:true,path:true,scan:true,tf:false};document.querySelectorAll('[data-layer]').forEach(input=>input.onchange=()=>enabled[input.dataset.layer]=input.checked);
function fit(){if(!messages.map||!mapMesh)return;const m=transform(messages.map.header.frame_id);if(!m)return;const center=new T.Vector3(messages.map.info.width*messages.map.info.resolution/2,messages.map.info.height*messages.map.info.resolution/2,0).applyMatrix4(messages.mapOrigin).applyMatrix4(m);const span=Math.max(messages.map.info.width,messages.map.info.height)*messages.map.info.resolution;controls.target.copy(center);camera.position.copy(center).add(new T.Vector3(0,-.01,Math.max(span*1.4,3)));controls.update();mapFitted=true;}
$('fit').onclick=fit;
async function loadModel(text){const token=++modelEpoch;$('modelStatus').textContent='기체 모델 로딩…';try{const result=await buildModel(text);if(token!==modelEpoch){for(const l of result.links)disposeGroup(l.group);return;}disposeGroup(layers.robot);modelLinks=result.links;modelJoints=result.joints;for(const l of modelLinks)layers.robot.add(l.group);$('modelStatus').textContent=`URDF ${modelLinks.length} 링크${result.missing?` · 미지원/누락 메시 ${result.missing}개`:''}`;}catch(e){$('modelStatus').textContent=e.message;}}

let picking=false,goal=null,pointerStart=null,selectionKind='goal';
const raycaster=new T.Raycaster();
const goalMarker=new T.Group();goalMarker.visible=false;scene.add(goalMarker);
const goalRing=new T.Mesh(new T.RingGeometry(.17,.23,64),new T.MeshBasicMaterial({color:0x39dfbd,side:T.DoubleSide,depthTest:false}));goalRing.renderOrder=20;goalMarker.add(goalRing);
const goalDot=new T.Mesh(new T.CircleGeometry(.055,24),new T.MeshBasicMaterial({color:0xffffff,side:T.DoubleSide,depthTest:false}));goalDot.renderOrder=21;goalMarker.add(goalDot);
const goalArrow=new T.ArrowHelper(new T.Vector3(1,0,0),new T.Vector3(),.65,0x39dfbd,.17,.11);goalMarker.add(goalArrow);
function navigationReport(text){$('navigationStatus').textContent=text;}
const navigation=new Navigation(navigationReport,()=>{updateGoalButtons();renderRobots();});
function updateGoalButtons(){
 $('destination').disabled=navigation.busy;
 $('initialPose').disabled=navigation.busy;
 const isInitial=selectionKind==='initial';
 $('applyInitialPose').hidden=!isInitial;
 $('startGoal').hidden=isInitial;
 $('applyInitialPose').disabled=!goal||goal.kind!=='initial'||!goal.headingSet||picking||navigation.busy||(!demo&&(!navigation.ready||!ros?.isConnected));
 const reason=!demo&&(!navigation.ready||!ros?.isConnected)?'자율주행 대기: 제어 연결이 없습니다.':!demo&&!estimatedBase()?`자율주행 대기: ${profiles[selected].fixed} 기준 위치 추정이 없습니다. 상단 초기 위치 버튼으로 현재 위치와 방향을 지정하세요.`:!goal?'지도에서 목적지를 선택하세요.':picking?'목적지 선택을 마쳐주세요.':'';
 $('navigationPrerequisite').textContent=isInitial?(!goal?'지도에서 로봇의 실제 위치를 누른 채 방향을 드래그하세요.':!goal.headingSet?'도착 목적지가 아닌 현재 방향을 드래그로 지정하세요.':!demo&&!navigation.ready?'초기 위치 전송 대기: 제어 연결이 없습니다.':'초기 위치 설정은 로봇을 이동시키지 않습니다.'):reason;
 $('startGoal').title=reason;
 $('startGoal').disabled=!goal||goal.kind!=='goal'||picking||navigation.busy||(!demo&&(!navigation.ready||!ros?.isConnected||!estimatedBase()));
 $('clearGoal').disabled=navigation.busy||(!goal&&!picking);
 $('stop').disabled=!demo&&!navigation.ready;
}
function selectionMode(value){picking=value;controls.enabled=!value;renderer.domElement.style.cursor=value?'crosshair':'';$('destination').setAttribute('aria-pressed',String(value&&selectionKind==='goal'));$('destination').textContent=value&&selectionKind==='goal'?'◎ 목적지 선택 중':'◎ 목적지';$('initialPose').setAttribute('aria-pressed',String(value&&selectionKind==='initial'));$('initialPose').textContent=value&&selectionKind==='initial'?'⊕ 현재 위치 선택 중':'⊕ 초기 위치';updateGoalButtons();}
function clearDestination(){selectionKind='goal';goal=null;pointerStart=null;goalMarker.visible=false;selectionMode(false);$('goalCoordinates').textContent='목적지를 선택하세요.';}
function mapPoint(event,validate=true){
 if(!mapMesh||!layers.map.visible||!messages.map)return null;
 const rect=renderer.domElement.getBoundingClientRect();raycaster.setFromCamera(new T.Vector2((event.clientX-rect.left)/rect.width*2-1,-(event.clientY-rect.top)/rect.height*2+1),camera);
 scene.updateMatrixWorld(true);const hit=raycaster.intersectObject(mapMesh)[0];if(!hit)return null;
 if(validate){const local=layers.map.worldToLocal(hit.point.clone()),info=messages.map.info,x=Math.floor(local.x/info.resolution),y=Math.floor(local.y/info.resolution);if(x<0||y<0||x>=info.width||y>=info.height)return null;const value=messages.map.data[y*info.width+x];if(value<0||value>=50){navigationReport('장애물 또는 미확인 영역입니다. 지도의 빈 공간을 선택하세요.');return null;}}
 return hit.point;
}
function setDestination(point,yaw=0,headingSet=false){goal={kind:selectionKind,headingSet,x:point.x,y:point.y,z:point.z+.025,yaw,frame:normalize(profiles[selected].fixed)};goalMarker.position.set(goal.x,goal.y,goal.z+.06);goalMarker.rotation.z=yaw;goalMarker.visible=true;const color=selectionKind==='initial'?0xffb347:0x39dfbd;goalRing.material.color.setHex(color);goalArrow.setColor(color);$('goalCoordinates').textContent=`${selectionKind==='initial'?'초기 위치':'목적지'}  X ${goal.x.toFixed(2)} / Y ${goal.y.toFixed(2)} m · 방향 ${Math.round(yaw*180/Math.PI)}°`;}
function beginSelection(kind){if(picking&&selectionKind===kind){selectionMode(false);return;}if(!messages.map||!layers.map.visible){navigationReport('지도가 표시된 후 위치를 선택하세요.');return;}clearDestination();selectionKind=kind;$('goalCoordinates').textContent=kind==='initial'?'로봇의 현재 위치와 방향을 선택하세요.':'목적지를 선택하세요.';selectionMode(true);navigationReport(kind==='initial'?'로봇의 실제 위치에서 바라보는 방향으로 드래그하세요. 초기 위치 적용 전에는 전송하지 않습니다.':'지도 클릭: 목적지 · 누른 채 드래그: 도착 방향 · Esc: 취소');}
$('destination').onclick=()=>beginSelection('goal');
$('initialPose').onclick=()=>beginSelection('initial');
renderer.domElement.addEventListener('pointerdown',event=>{if(!picking||event.button!==0)return;const point=mapPoint(event);if(!point)return;pointerStart={id:event.pointerId,point};renderer.domElement.setPointerCapture(event.pointerId);setDestination(point);event.preventDefault();});
renderer.domElement.addEventListener('pointermove',event=>{if(!picking||!pointerStart||pointerStart.id!==event.pointerId)return;const point=mapPoint(event,false);if(point&&point.distanceTo(pointerStart.point)>.08)setDestination(pointerStart.point,Math.atan2(point.y-pointerStart.point.y,point.x-pointerStart.point.x),true);});
renderer.domElement.addEventListener('pointerup',event=>{if(!pointerStart||pointerStart.id!==event.pointerId)return;pointerStart=null;selectionMode(false);navigationReport(selectionKind==='initial'?goal?.headingSet?'위치와 방향을 확인한 뒤 초기 위치 적용을 누르세요.':'방향이 지정되지 않았습니다. 초기 위치 버튼을 눌러 드래그로 다시 지정하세요.':demo?'테스트 목적지를 선택했습니다. 실제 명령은 전송되지 않습니다.':'목적지를 확인한 뒤 자율주행을 누르세요.');});
renderer.domElement.addEventListener('pointercancel',()=>{if(picking){clearDestination();navigationReport('목적지 선택 취소');}});
$('clearGoal').onclick=()=>{clearDestination();navigationReport('목적지 선택 취소');};
document.addEventListener('keydown',event=>{if(event.key==='Escape'&&!navigation.busy&&!$('configDialog').open&&!$('helpDialog').open){clearDestination();navigationReport('목적지 선택 취소');}});
$('applyInitialPose').onclick=()=>{if($('applyInitialPose').disabled)return;if(demo){navigationReport('오프라인 테스트: 초기 위치 적용 동작 확인 · 로봇 전송 없음');return;}try{initialPoseSentAt=Date.now();navigation.setInitialPose(goal);}catch(e){navigationReport(e.message);}};
$('startGoal').onclick=()=>{if($('startGoal').disabled)return;if(demo){navigationReport('오프라인 테스트: 목적지 전송 동작 확인 · 로봇 명령 없음');return;}try{navigation.start(goal);}catch(e){navigationReport(e.message);}};
$('stop').onclick=()=>{selectionMode(false);if(demo){navigationReport('오프라인 테스트: 정지 버튼 동작 확인 · 로봇 명령 없음');return;}try{navigation.stop();}catch(e){navigationReport(e.message);}};
$('help').onclick=()=>$('helpDialog').showModal();$('closeHelp').onclick=()=>$('helpDialog').close();

function updateBattery(){
 const raw=messages.battery?.data,age=Date.now()-(last.battery||0);
 const valid=Number.isInteger(raw)&&raw>=0&&raw<=65535;
 const connected=demo||ros?.isConnected;
  if(!valid||age>10000||!connected){$('battery').textContent='추정 잔량 —';$('batteryStatus').textContent=!connected?'연결 끊김':!last.battery?'수신 대기':!valid?'값 확인 필요':'수신 지연';$('battery').classList.remove('low');return;}
 const voltage=raw/10,percent=Math.round(Math.max(0,Math.min(100,10+(voltage-6.5)*60)));
 $('battery').textContent=`추정 잔량 ${percent}%`;
 $('battery').classList.toggle('low',voltage<=6.5);
 $('batteryStatus').textContent=voltage<=6.5?'저전압 · 충전 필요':'배터리 추정치';
}
let lastUI=0;
function animate(now){
 requestAnimationFrame(animate);
 const rect=$('viewport').getBoundingClientRect();
 if(renderer.domElement.width!==Math.round(rect.width*renderer.getPixelRatio())||renderer.domElement.height!==Math.round(rect.height*renderer.getPixelRatio())){
  renderer.setSize(rect.width,rect.height,false);camera.aspect=rect.width/rect.height;camera.updateProjectionMatrix();
 }
 const p=profiles[selected],missing=[];
 layers.grid.visible=enabled.grid;
 for(const key of ['map','path','scan']){
  const m=messages[key],tr=m&&transform(m.header.frame_id),fresh=key!=='scan'||Date.now()-(last.scan||0)<3000;
  layers[key].visible=!!(enabled[key]&&tr&&fresh);
  if(m&&!tr)missing.push(m.header.frame_id);
  if(tr)place(layers[key],key==='map'?tr.clone().multiply(messages.mapOrigin):tr);
 }
 if(!mapFitted&&messages.map)fit();
 const base=estimatedBase();
 layers.robot.visible=enabled.robot&&!!base;
 if(base){
  if(modelLinks.length){
   place(layers.robot,new T.Matrix4());
   const lookup=frame=>normalize(frame)===normalize(p.base)?base:transform(frame);
   for(const l of modelLinks){const mat=modelTransform(l.frame,lookup,modelJoints);l.group.visible=!!mat;if(mat)place(l.group,mat);}
  }else place(layers.robot,base);
 }
 if(now-lastUI>500){
  lastUI=now;updateBattery();
  $('topicStatus').replaceChildren(...['map','scan','path','odom','tf','amcl','camera'].map(k=>{
   const s=document.createElement('span'),age=last[k]?(Date.now()-last[k])/1000:Infinity;
   s.textContent=`${{map:'지도',scan:'라이다',path:'경로',odom:'속도',tf:'TF',amcl:'위치 추정',camera:'카메라'}[k]} · ${age===Infinity?'수신 대기':age<3?'수신 중':`${Math.floor(age)}초 전`}`;
   s.className=age<3?'live':'';return s;
  }));
  const velocity=messages.odom?.twist?.twist?.linear;
  $('speed').textContent=velocity&&Date.now()-last.odom<3000?Math.hypot(velocity.x,velocity.y).toFixed(2):'—';
  if(base){const v=new T.Vector3().setFromMatrixPosition(base);$('position').textContent=`${v.x.toFixed(2)}, ${v.y.toFixed(2)}`;}else $('position').textContent='—';
  const hasTfBase=!!transform(p.base);
  $('frameStatus').textContent=hasTfBase?`${p.fixed} → ${p.base}`:messages.amcl?`${p.fixed} 위치 추정 수신 · TF 대기`:missing.length?`TF 대기: ${[...new Set(missing)].join(', ')} → ${p.fixed}`:`기체 TF 대기: ${p.base}`;
  if(initialPoseSentAt&&last.amcl>=initialPoseSentAt){navigationReport('초기 위치 적용 확인 · 위치 추정 응답 수신');initialPoseSentAt=0;}
  else if(initialPoseSentAt&&Date.now()-initialPoseSentAt>5000){navigationReport('초기 위치 응답 없음 · 위치 추정 모듈이 실행 중인지 확인하세요.');initialPoseSentAt=0;}
  $('cameraStatus').textContent=last.camera?Date.now()-last.camera<3000?'수신 중':'마지막 프레임 · 수신 지연':'수신 대기';
  if(Object.keys(last).length)$('notice').textContent=demo?'오프라인 테스트 데이터 · 실제 로봇 상태가 아닙니다.':missing.length?'데이터 수신됨 · 좌표 변환 대기':'실제 ROS 데이터 표시 · 마지막 수신 시간은 아래에서 확인하세요';
  disposeGroup(layers.tf);
  if(enabled.tf)for(const frame of transforms.keys()){const mat=transform(frame);if(mat){const axes=new T.AxesHelper(.3);place(axes,mat);layers.tf.add(axes);}}
 }
 layers.tf.visible=enabled.tf;updateGoalButtons();controls.update();renderer.render(scene,camera);
}
connect();requestAnimationFrame(animate);
