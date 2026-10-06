import * as T from 'three';
import {
  Navigation
} from './navigation.js';
import {
  buildModel,
  modelTransform
} from './model.js';
import {
  startDemo
} from './demo.js';
import {
  OrbitControls
} from '../vendor/OrbitControls.js';

// 기본 도우미와 로봇별 연결 설정을 준비한다.
const $ = id => document.getElementById(id);
const normalize = s => (s || '').replace(/^\//, '');
const robotName = index => `Robot ${index+1}`;
let serverConfig = {};
try {
  const response = await fetch('/api/viewer-config');
  if (response.ok) serverConfig = await response.json();
} catch {}
const defaults = {
  url: '',
  controlApi: '',
  fixed: serverConfig.fixed_frame || 'map',
  base: serverConfig.base_frame || 'base_footprint',
  map: '/map',
  scan: '/scan',
  path: '/plan',
  odom: '/odom',
  tf: '/tf',
  static: '/tf_static',
  camera: '/yolo/result_image/compressed',
  description: '/robot_description',
  amcl: '/amcl_pose',
  action: '/navigate_to_pose',
  cmd: '/cmd_vel',
  initialpose: '/initialpose',
  battery: '/battery'
};
let profiles = [{
  ...defaults,
  name: 'Robot 1'
}, {
  ...defaults,
  name: 'Robot 2'
}, {
  ...defaults,
  name: 'Robot 3'
}];
try {
  const saved = JSON.parse(localStorage.getItem('yahboom-viewer-v1'));
  if (Array.isArray(saved) && saved.length === 3) profiles = saved.map((p, i) => {
    const merged = {
      ...profiles[i],
      ...p,
      name: robotName(i)
    };
    delete merged.robotIp;
    delete merged.ip;
    if (merged.controlApi === 'http://127.0.0.1:8765') merged.controlApi = '/api/control';
    return merged;
  });
} catch {}
const demo = new URLSearchParams(location.search).has('demo');
let selectedMode = localStorage.getItem('yahboom-selected') || '1';
if (!['0', '1', '2', 'all'].includes(selectedMode)) selectedMode = '1';

// Three.js 장면, 카메라, 조명과 기준 격자를 구성한다.
const scene = new T.Scene();
scene.background = new T.Color('#0d1525');
const camera = new T.PerspectiveCamera(48, 1, .01, 2000);
camera.up.set(0, 0, 1);
camera.position.set(0, -9, 13);
const renderer = new T.WebGLRenderer({
  antialias: true
});
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
$('viewport').append(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.maxPolarAngle = Math.PI * .49;
scene.add(new T.HemisphereLight(0xffffff, 0x334466, 3));
const light = new T.DirectionalLight(0xffffff, 2);
light.position.set(2, -4, 8);
scene.add(light);
const grid = new T.GridHelper(100, 100, 0x35445d, 0x202d43);
grid.rotation.x = Math.PI / 2;
scene.add(grid);

// 3D 객체의 좌표 변환과 그래픽 자원 해제를 담당한다.
function matrix(p, q) {
  return new T.Matrix4().compose(new T.Vector3(p.x, p.y, p.z), new T.Quaternion(q.x, q.y, q.z, q.w)
    .normalize(), new T.Vector3(1, 1, 1));
}

function place(group, mat) {
  group.matrixAutoUpdate = false;
  group.matrix.copy(mat);
}

function disposeGroup(group) {
  for (const child of [...group.children]) {
    child.traverse(object => {
      object.geometry?.dispose();
      if (object.material)
        for (const material of [].concat(object.material)) {
          material.map?.dispose();
          material.dispose();
        }
    });
    group.remove(child);
  }
}

function geometry(object, points) {
  object.geometry.dispose();
  object.geometry = new T.BufferGeometry();
  object.geometry.setAttribute('position', new T.Float32BufferAttribute(points, 3));
  object.geometry.computeBoundingSphere();
}

function addFallbackMarker(state) {
  const marker = new T.Mesh(new T.BoxGeometry(.30, .18, .1), new T.MeshStandardMaterial({
    color: state.index === 0 ? 0xf7a817 : state.index === 1 ? 0x36b7ff : 0xbb75ff
  }));
  marker.position.z = .1;
  state.groups.robot.add(marker, new T.ArrowHelper(new T.Vector3(1, 0, 0), new T.Vector3(0, 0, .19),
    .45, state.index === 1 ? 0x7bd7ff : 0x5fb5ff, .13, .08));
}

function createRobotState(index) {
  const root = new T.Group(),
    groups = {
      map: new T.Group(),
      robot: new T.Group(),
      path: new T.Group(),
      scan: new T.Group(),
      tf: new T.Group()
    };
  root.name = `robot-${index+1}`;
  for (const group of Object.values(groups)) root.add(group);
  scene.add(root);
  const pathLine = new T.Line(new T.BufferGeometry(), new T.LineBasicMaterial({
    color: index === 1 ? 0x41dfc1 : 0x4d9fff
  }));
  groups.path.add(pathLine);
  const scanPoints = new T.Points(new T.BufferGeometry(), new T.PointsMaterial({
    color: index === 1 ? 0xffa53d : 0xff3535,
    size: .045,
    sizeAttenuation: true
  }));
  groups.scan.add(scanPoints);
  const state = {
    index,
    root,
    groups,
    pathLine,
    scanPoints,
    ros: null,
    subscriptions: [],
    epoch: 0,
    last: {},
    messages: {},
    transforms: new Map(),
    mapMesh: null,
    mapFitted: false,
    modelLinks: [],
    modelJoints: new Map(),
    modelEpoch: 0,
    initialPoseSentAt: 0,
    demoTimer: null,
    status: '연결 대기',
    cameraUrl: '',
    navigationStatus: '목적지 선택 후 자율주행을 누르세요.'
  };
  state.navigation = new Navigation(text => {
    state.navigationStatus = text;
    if (selectedMode === 'all') navigationReport(`${robotName(index)} · ${text}`, false);
    else if (Number(selectedMode) === index) navigationReport(text, false);
  }, () => {
    updateGoalButtons();
    renderRobots();
  });
  addFallbackMarker(state);
  groups.robot.visible = false;
  groups.tf.visible = false;
  return state;
}

// Each entry owns its ROS socket, subscriptions, Navigation instance, TF tree and Three.js groups.
const robotStates = profiles.map((_, index) => createRobotState(index));
const activeIndices = () => selectedMode === 'all' ? [0, 1] : [Number(selectedMode)];
const primaryState = () => robotStates[selectedMode === 'all' ? 0 : Number(selectedMode)];
const isVisible = state => activeIndices().includes(state.index);

// ROS 좌표계와 수신 메시지를 화면에 표시할 데이터로 변환한다.
function transform(state, frame) {
  const fixed = normalize(profiles[state.index].fixed);
  let current = normalize(frame),
    result = new T.Matrix4(),
    visited = new Set();
  while (current !== fixed) {
    if (!current || visited.has(current)) return null;
    visited.add(current);
    const edge = state.transforms.get(current);
    if (!edge || (!edge.static && Date.now() - edge.time > 5000)) return null;
    result.premultiply(edge.matrix);
    current = edge.parent;
  }
  return result;
}

function estimatedBase(state) {
  const p = profiles[state.index],
    tf = transform(state, p.base);
  if (tf) return tf;
  const amcl = state.messages.amcl;
  if (!amcl || normalize(amcl.header?.frame_id) !== normalize(p.fixed)) return null;
  return matrix(amcl.pose.pose.position, amcl.pose.pose.orientation);
}

function showMap(state, message) {
  if (!message.info || message.info.width * message.info.height !== message.data?.length || message
    .data.length > 16777216) throw Error('지도 크기/데이터가 유효하지 않습니다.');
  disposeGroup(state.groups.map);
  const {
    width,
    height,
    resolution,
    origin
  } = message.info, bytes = new Uint8Array(width * height * 4);
  for (let i = 0; i < message.data.length; i++) {
    const value = message.data[i],
      shade = value < 0 ? 65 : Math.round(235 - 2.15 * value);
    bytes.set([shade, shade, shade, 255], i * 4);
  }
  const texture = new T.DataTexture(bytes, width, height, T.RGBAFormat);
  texture.needsUpdate = true;
  texture.magFilter = T.NearestFilter;
  texture.minFilter = T.NearestFilter;
  state.mapMesh = new T.Mesh(new T.PlaneGeometry(width * resolution, height * resolution), new T
    .MeshBasicMaterial({
      map: texture,
      side: T.DoubleSide,
      transparent: true,
      opacity: .86
    }));
  state.mapMesh.position.set(width * resolution / 2, height * resolution / 2, -.025);
  state.groups.map.add(state.mapMesh);
  state.messages.mapOrigin = matrix(origin.position, origin.orientation);
}

function imageUrl(message) {
  if (!/jpeg|jpg|png/i.test(message.format || '')) return '';
  const mime = /png/i.test(message.format) ? 'image/png' : 'image/jpeg',
    data = typeof message.data === 'string' ? message.data : btoa(Array.from(message.data, c =>
      String.fromCharCode(c)).join(''));
  return `data:${mime};base64,${data}`;
}

function receive(state, key, message) {
  if (key === 'description' && state.messages.description?.data !== message.data) loadModel(state,
    message.data);
  state.last[key] = Date.now();
  state.messages[key] = message;
  if (key === 'tf' || key === 'static') {
    for (const item of message.transforms || []) state.transforms.set(normalize(item
      .child_frame_id), {
      parent: normalize(item.header.frame_id),
      matrix: matrix(item.transform.translation, item.transform.rotation),
      time: Date.now(),
      static: key === 'static'
    });
    return;
  }
  if (key === 'map') showMap(state, message);
  if (key === 'path') geometry(state.pathLine, (message.poses || []).flatMap(p => [p.pose.position
    .x, p.pose.position.y, p.pose.position.z + .05
  ]));
  if (key === 'scan') {
    const points = [];
    for (let i = 0; i < message.ranges.length; i++) {
      const range = message.ranges[i];
      if (Number.isFinite(range) && range >= message.range_min && range <= message.range_max) {
        const angle = message.angle_min + i * message.angle_increment;
        points.push(range * Math.cos(angle), range * Math.sin(angle), .04);
      }
    }
    geometry(state.scanPoints, points);
  }
  if (key === 'camera') {
    state.cameraUrl = imageUrl(message);
    if (isVisible(state)) renderCameras();
  }
}

function resetState(state, {
  closeNavigation = true
} = {}) {
  for (const subscription of state.subscriptions) subscription.unsubscribe();
  state.subscriptions = [];
  if (state.ros) {
    state.ros.close();
    state.ros = null;
  }
  if (closeNavigation) state.navigation.close();
  clearInterval(state.demoTimer);
  state.demoTimer = null;
  state.modelEpoch++;
  state.initialPoseSentAt = 0;
  state.modelLinks = [];
  state.modelJoints.clear();
  disposeGroup(state.groups.robot);
  addFallbackMarker(state);
  disposeGroup(state.groups.map);
  disposeGroup(state.groups.tf);
  geometry(state.pathLine, []);
  geometry(state.scanPoints, []);
  state.groups.robot.visible = false;
  state.last = {};
  state.messages = {};
  state.transforms.clear();
  state.mapMesh = null;
  state.mapFitted = false;
  state.cameraUrl = '';
  state.status = '연결 대기';
}

function closeState(state, {
  closeNavigation = true
} = {}) {
  state.epoch++;
  resetState(state, {
    closeNavigation
  });
  state.root.visible = false;
}

// 로봇별 ROS 토픽 구독과 WebSocket 연결 생명주기를 관리한다.
function subscribeState(state, token) {
  const p = profiles[state.index],
    types = {
      map: 'nav_msgs/msg/OccupancyGrid',
      scan: 'sensor_msgs/msg/LaserScan',
      path: 'nav_msgs/msg/Path',
      odom: 'nav_msgs/msg/Odometry',
      tf: 'tf2_msgs/msg/TFMessage',
      static: 'tf2_msgs/msg/TFMessage',
      camera: 'sensor_msgs/msg/CompressedImage',
      description: 'std_msgs/msg/String',
      amcl: 'geometry_msgs/msg/PoseWithCovarianceStamped',
      battery: 'std_msgs/msg/UInt16'
    };
  for (const [key, type] of Object.entries(types)) {
    if (!p[key]) continue;
    const connection = state.ros,
      id = `viewer:${state.index}:${token}:${key}`,
      handler = message => {
        if (token !== state.epoch) return;
        try {
          receive(state, key, message);
        } catch (error) {
          $('notice').textContent = `${robotName(state.index)} ${key}: ${error.message}`;
        }
      };
    connection.on(p[key], handler);
    const latched = ['map', 'static', 'description', 'amcl'].includes(key);
    connection.callOnConnection({
      op: 'subscribe',
      id,
      topic: p[key],
      type,
      throttle_rate: key === 'camera' ? 150 : key === 'scan' ? 100 : 0,
      queue_length: 1,
      qos: {
        history: 'keep_last',
        depth: key === 'static' ? 100 : 5,
        reliability: latched ? 'reliable' : 'best_effort',
        durability: latched ? 'transient_local' : 'volatile'
      }
    });
    state.subscriptions.push({
      unsubscribe() {
        connection.removeListener(p[key], handler);
        if (connection.isConnected) connection.callOnConnection({
          op: 'unsubscribe',
          id,
          topic: p[key]
        });
      }
    });
  }
}

function connectState(state) {
  const p = profiles[state.index];
  state.root.visible = true;
  if (demo) {
    if (state.demoTimer) return;
    state.status = '테스트 데이터';
    state.demoTimer = startDemo((key, message) => receive(state, key, message));
    fetch('/static/robot_models/yahboom_vehicle/yahboom.urdf').then(r => r.text()).then(text =>
      loadModel(state, text));
    return;
  }
  if (state.ros && (state.ros.isConnected || state.status === '연결 중…')) return;
  resetState(state, {
    closeNavigation: false
  });
  state.root.visible = true;
  const token = ++state.epoch;
  if (!p.url) {
    state.status = '연결 주소 미설정';
    return;
  }
  if (!/^wss?:\/\//.test(p.url)) {
    state.status = '잘못된 WebSocket 주소';
    return;
  }
  state.status = '연결 중…';
  state.ros = new ROSLIB.Ros({
    url: p.url
  });
  state.navigation.connect(p.url, p);
  state.ros.on('connection', () => {
    if (token !== state.epoch) return;
    state.status = '● 연결됨';
    subscribeState(state, token);
    renderConnection();
  });
  state.ros.on('error', () => {
    if (token !== state.epoch) return;
    state.status = '연결 실패';
    $('notice').textContent = `${robotName(state.index)} rosbridge 실행 여부와 주소를 확인하세요.`;
    renderConnection();
  });
  state.ros.on('close', () => {
    if (token !== state.epoch) return;
    state.status = '연결 끊김';
    selectionMode(false);
    navigationReport(`${robotName(state.index)} 연결 끊김 · 로봇 상태를 확인하세요.`);
    renderConnection();
  });
  state.ros.on('status', status => {
    if (token === state.epoch && status.level === 'error') $('notice').textContent =
      `${robotName(state.index)} · ${status.msg}`;
  });
}

function reconcileConnections() {
  const wanted = new Set(activeIndices());
  for (const state of robotStates) {
    if (wanted.has(state.index)) connectState(state);
    else if (state.ros || state.demoTimer) closeState(state, {
      closeNavigation: false
    });
    else state.root.visible = false;
  }
  renderConnection();
  renderCameras();
}

function renderConnection() {
  const states = activeIndices().map(index => robotStates[index]);
  $('connection').textContent = selectedMode === 'all' ? states.map(state =>
    `R${state.index+1} ${state.status.replace('● ','')}`).join(' · ') : states[0].status;
  if (states.some(state => !profiles[state.index].url) && !demo) $('notice').textContent =
    selectedMode === 'all' ? '연결 설정에서 Robot 1과 Robot 2의 rosbridge를 먼저 준비하세요.' :
    '연결 설정에서 이 로봇의 rosbridge 주소를 입력하세요.';
}

// 단일 로봇/ALL 모드 선택과 연결 설정 화면을 처리한다.
function selectMode(mode) {
  clearDestination();
  selectedMode = mode;
  localStorage.setItem('yahboom-selected', mode);
  $('robotTitle').textContent = mode === 'all' ? 'Robot 1 + Robot 2' : robotName(Number(mode));
  $('settings').disabled = mode === 'all';
  reconcileConnections();
  renderRobots();
  $('navigationStatus').textContent = mode === 'all' ? 'ALL에서는 새 주행을 시작할 수 없으며 정지는 두 로봇에 전달됩니다.' :
    primaryState().navigationStatus;
  updateGoalButtons();
}

function renderRobots() {
  const buttons = profiles.map((profile, index) => {
    const button = document.createElement('button');
    button.textContent = profile.name + (robotStates[index].navigation.busy ? ' · 주행 중' : '');
    button.className = selectedMode === String(index) ? 'selected' : '';
    button.onclick = () => selectMode(String(index));
    return button;
  });
  const all = document.createElement('button');
  all.textContent = 'ALL';
  all.className = selectedMode === 'all' ? 'selected all-button' : 'all-button';
  all.onclick = () => selectMode('all');
  buttons.push(all);
  $('robots').replaceChildren(...buttons);
}

const fields = {
  url: 'rosbridge 주소',
  controlApi: '플랫폼 제어 API (선택)',
  fixed: '기준 좌표계',
  base: '기체 좌표계',
  map: '지도 토픽',
  scan: '라이다 토픽',
  path: '계획 경로 토픽',
  odom: '속도 토픽',
  tf: 'TF 토픽',
  static: '고정 TF 토픽',
  camera: '압축 카메라 토픽',
  description: '기체 URDF 토픽',
  amcl: '위치 추정 응답 토픽',
  action: 'Nav2 액션',
  cmd: '정지 속도 토픽',
  initialpose: '초기 위치 토픽',
  battery: '배터리 토픽'
};
$('settings').onclick = () => {
  if (selectedMode === 'all') {
    navigationReport('ALL에서는 연결 설정을 변경할 수 없습니다. Robot 1 또는 Robot 2를 선택하세요.');
    return;
  }
  const state = primaryState();
  if (state.navigation.busy) {
    navigationReport('주행을 정지한 뒤 연결 설정을 변경하세요.');
    return;
  }
  const p = profiles[state.index];
  $('configFields').replaceChildren(...Object.entries(fields).map(([key, label]) => {
    const element = document.createElement('label');
    element.textContent = label;
    const input = document.createElement('input');
    input.name = key;
    input.value = p[key];
    if (key === 'url') input.placeholder = 'ws://192.168.0.x:9090';
    element.append(input);
    return element;
  }));
  $('configDialog').showModal();
};
$('cancel').onclick = () => $('configDialog').close();
$('configForm').onsubmit = event => {
  event.preventDefault();
  if (selectedMode === 'all') return;
  const state = primaryState(),
    p = profiles[state.index];
  for (const [key, value] of new FormData(event.target)) p[key] = String(value).trim();
  if (!/^wss?:\/\//.test(p.url)) {
    navigationReport('rosbridge 주소는 ws:// 또는 wss://로 시작해야 합니다.');
    state.status = '연결 설정 실패';
    renderConnection();
    return;
  }
  p.controlApi = '';
  localStorage.setItem('yahboom-viewer-v1', JSON.stringify(profiles));
  $('configDialog').close();
  closeState(state);
  connectState(state);
  renderConnection();
};

const enabled = {
  grid: true,
  map: true,
  robot: true,
  path: true,
  scan: true,
  tf: false
};
document.querySelectorAll('[data-layer]').forEach(input => input.onchange = () => enabled[input
  .dataset.layer] = input.checked);

// 지도 화면 맞춤과 URDF 로봇 모델 로딩을 처리한다.
function fit() {
  const state = primaryState(),
    message = state.messages.map;
  if (!message || !state.mapMesh) return;
  const tf = transform(state, message.header.frame_id);
  if (!tf) return;
  const center = new T.Vector3(message.info.width * message.info.resolution / 2, message.info
      .height * message.info.resolution / 2, 0).applyMatrix4(message.mapOrigin).applyMatrix4(tf),
    span = Math.max(message.info.width, message.info.height) * message.info.resolution;
  controls.target.copy(center);
  camera.position.copy(center).add(new T.Vector3(0, -.01, Math.max(span * 1.4, 3)));
  controls.update();
  state.mapFitted = true;
}
$('fit').onclick = fit;
async function loadModel(state, text) {
  const token = ++state.modelEpoch;
  renderModelStatus();
  try {
    const result = await buildModel(text);
    if (token !== state.modelEpoch) {
      for (const link of result.links) disposeGroup(link.group);
      return;
    }
    disposeGroup(state.groups.robot);
    state.modelLinks = result.links;
    state.modelJoints = result.joints;
    for (const link of state.modelLinks) state.groups.robot.add(link.group);
    state.modelMissing = result.missing;
    renderModelStatus();
  } catch (error) {
    state.modelError = error.message;
    renderModelStatus();
  }
}

function renderModelStatus() {
  const states = activeIndices().map(index => robotStates[index]);
  $('modelStatus').textContent = states.map(state =>
    `${robotName(state.index)}: ${state.modelError||(state.modelLinks.length?`URDF ${state.modelLinks.length} 링크${state.modelMissing?` · 누락 ${state.modelMissing}`:''}`:'기본 표식')}`
  ).join(selectedMode === 'all' ? ' / ' : '');
}

let picking = false,
  goal = null,
  pointerStart = null,
  selectionKind = 'goal';
const raycaster = new T.Raycaster(),
  goalMarker = new T.Group();
goalMarker.visible = false;
scene.add(goalMarker);
const goalRing = new T.Mesh(new T.RingGeometry(.17, .23, 64), new T.MeshBasicMaterial({
  color: 0x39dfbd,
  side: T.DoubleSide,
  depthTest: false
}));
goalRing.renderOrder = 20;
goalMarker.add(goalRing);
const goalDot = new T.Mesh(new T.CircleGeometry(.055, 24), new T.MeshBasicMaterial({
  color: 0xffffff,
  side: T.DoubleSide,
  depthTest: false
}));
goalDot.renderOrder = 21;
goalMarker.add(goalDot);
const goalArrow = new T.ArrowHelper(new T.Vector3(1, 0, 0), new T.Vector3(), .65, 0x39dfbd, .17,
  .11);
goalMarker.add(goalArrow);

// 초기 위치와 목적지 선택 상태 및 버튼 활성 조건을 관리한다.
function navigationReport(text, save = true) {
  $('navigationStatus').textContent = text;
  if (save && selectedMode !== 'all') primaryState().navigationStatus = text;
}

function updateGoalButtons() {
  const all = selectedMode === 'all',
    state = primaryState(),
    navigation = state.navigation,
    connected = demo || (navigation.ready && state.ros?.isConnected),
    isInitial = selectionKind === 'initial';
  $('destination').disabled = all || navigation.busy;
  $('initialPose').disabled = all || navigation.busy;
  $('applyInitialPose').hidden = !isInitial;
  $('startGoal').hidden = isInitial;
  $('applyInitialPose').disabled = all || !goal || goal.kind !== 'initial' || !goal.headingSet ||
    picking || navigation.busy || !connected;
  const reason = all ? 'ALL 모드에서는 주행 제어가 비활성화됩니다. 정지 명령만 두 로봇에 전송할 수 있습니다.' : !demo && !connected ?
    '자율주행 대기: 제어 연결이 없습니다.' : !demo && !estimatedBase(state) ?
    `자율주행 대기: ${profiles[state.index].fixed} 기준 위치 추정이 없습니다. 상단 초기 위치 버튼으로 현재 위치와 방향을 지정하세요.` : !
    goal ? '지도에서 목적지를 선택하세요.' : picking ? '목적지 선택을 마쳐주세요.' : '';
  $('navigationPrerequisite').textContent = all ? reason : isInitial ? (!goal ?
    '지도에서 로봇의 실제 위치를 누른 채 방향을 드래그하세요.' : !goal.headingSet ? '현재 방향을 드래그로 지정하세요.' : !demo && !
    navigation.ready ? '초기 위치 전송 대기: 제어 연결이 없습니다.' : '초기 위치 설정은 로봇을 이동시키지 않습니다.') : reason;
  $('startGoal').title = reason;
  $('startGoal').disabled = all || !goal || goal.kind !== 'goal' || picking || navigation.busy || !
    connected || (!demo && !estimatedBase(state));
  $('clearGoal').disabled = all || navigation.busy || (!goal && !picking);
  const stopTargets = activeIndices().map(index => robotStates[index]);
  $('stop').disabled = !demo && !stopTargets.some(item => item.navigation.ready);
}

function selectionMode(value) {
  picking = value;
  controls.enabled = !value;
  renderer.domElement.style.cursor = value ? 'crosshair' : '';
  $('destination').setAttribute('aria-pressed', String(value && selectionKind === 'goal'));
  $('destination').textContent = value && selectionKind === 'goal' ? '◎ 목적지 선택 중' : '◎ 목적지';
  $('initialPose').setAttribute('aria-pressed', String(value && selectionKind === 'initial'));
  $('initialPose').textContent = value && selectionKind === 'initial' ? '⊕ 현재 위치 선택 중' : '⊕ 초기 위치';
  updateGoalButtons();
}

function clearDestination() {
  selectionKind = 'goal';
  goal = null;
  pointerStart = null;
  goalMarker.visible = false;
  selectionMode(false);
  $('goalCoordinates').textContent = '목적지를 선택하세요.';
}

function mapPoint(event, validate = true) {
  const state = primaryState(),
    message = state.messages.map;
  if (selectedMode === 'all' || !state.mapMesh || !state.groups.map.visible || !message)
    return null;
  const rect = renderer.domElement.getBoundingClientRect();
  raycaster.setFromCamera(new T.Vector2((event.clientX - rect.left) / rect.width * 2 - 1, -(event
    .clientY - rect.top) / rect.height * 2 + 1), camera);
  scene.updateMatrixWorld(true);
  const hit = raycaster.intersectObject(state.mapMesh)[0];
  if (!hit) return null;
  if (validate) {
    const local = state.groups.map.worldToLocal(hit.point.clone()),
      info = message.info,
      x = Math.floor(local.x / info.resolution),
      y = Math.floor(local.y / info.resolution);
    if (x < 0 || y < 0 || x >= info.width || y >= info.height) return null;
    const value = message.data[y * info.width + x];
    if (value < 0 || value >= 50) {
      navigationReport('장애물 또는 미확인 영역입니다. 지도의 빈 공간을 선택하세요.');
      return null;
    }
  }
  return hit.point;
}

function setDestination(point, yaw = 0, headingSet = false) {
  const state = primaryState();
  goal = {
    kind: selectionKind,
    headingSet,
    x: point.x,
    y: point.y,
    z: point.z + .025,
    yaw,
    frame: normalize(profiles[state.index].fixed)
  };
  goalMarker.position.set(goal.x, goal.y, goal.z + .06);
  goalMarker.rotation.z = yaw;
  goalMarker.visible = true;
  const color = selectionKind === 'initial' ? 0xffb347 : 0x39dfbd;
  goalRing.material.color.setHex(color);
  goalArrow.setColor(color);
  $('goalCoordinates').textContent =
    `${selectionKind==='initial'?'초기 위치':'목적지'}  X ${goal.x.toFixed(2)} / Y ${goal.y.toFixed(2)} m · 방향 ${Math.round(yaw*180/Math.PI)}°`;
}

function beginSelection(kind) {
  if (selectedMode === 'all') return;
  if (picking && selectionKind === kind) {
    selectionMode(false);
    return;
  }
  const state = primaryState();
  if (!state.messages.map || !state.groups.map.visible) {
    navigationReport('지도가 표시된 후 위치를 선택하세요.');
    return;
  }
  clearDestination();
  selectionKind = kind;
  $('goalCoordinates').textContent = kind === 'initial' ? '로봇의 현재 위치와 방향을 선택하세요.' : '목적지를 선택하세요.';
  selectionMode(true);
  navigationReport(kind === 'initial' ? '로봇의 실제 위치에서 바라보는 방향으로 드래그하세요. 초기 위치 적용 전에는 전송하지 않습니다.' :
    '지도 클릭: 목적지 · 누른 채 드래그: 도착 방향 · Esc: 취소');
}
$('destination').onclick = () => beginSelection('goal');
$('initialPose').onclick = () => beginSelection('initial');
renderer.domElement.addEventListener('pointerdown', event => {
  if (!picking || event.button !== 0) return;
  const point = mapPoint(event);
  if (!point) return;
  pointerStart = {
    id: event.pointerId,
    point
  };
  renderer.domElement.setPointerCapture(event.pointerId);
  setDestination(point);
  event.preventDefault();
});
renderer.domElement.addEventListener('pointermove', event => {
  if (!picking || !pointerStart || pointerStart.id !== event.pointerId) return;
  const point = mapPoint(event, false);
  if (point && point.distanceTo(pointerStart.point) > .08) setDestination(pointerStart.point,
    Math.atan2(point.y - pointerStart.point.y, point.x - pointerStart.point.x), true);
});
renderer.domElement.addEventListener('pointerup', event => {
  if (!pointerStart || pointerStart.id !== event.pointerId) return;
  pointerStart = null;
  selectionMode(false);
  navigationReport(selectionKind === 'initial' ? goal?.headingSet ?
    '위치와 방향을 확인한 뒤 초기 위치 적용을 누르세요.' : '방향이 지정되지 않았습니다. 초기 위치 버튼을 눌러 다시 지정하세요.' : demo ?
    '테스트 목적지를 선택했습니다. 실제 명령은 전송되지 않습니다.' : '목적지를 확인한 뒤 자율주행을 누르세요.');
});
renderer.domElement.addEventListener('pointercancel', () => {
  if (picking) {
    clearDestination();
    navigationReport('목적지 선택 취소');
  }
});
$('clearGoal').onclick = () => {
  clearDestination();
  navigationReport('목적지 선택 취소');
};
document.addEventListener('keydown', event => {
  if (event.key === 'Escape' && !primaryState().navigation.busy && !$('configDialog').open && !
    $('helpDialog').open) {
    clearDestination();
    navigationReport('목적지 선택 취소');
  }
});
$('applyInitialPose').onclick = () => {
  if ($('applyInitialPose').disabled) return;
  if (demo) {
    navigationReport('오프라인 테스트: 초기 위치 적용 동작 확인 · 로봇 전송 없음');
    return;
  }
  const state = primaryState();
  try {
    state.initialPoseSentAt = Date.now();
    state.navigation.setInitialPose(goal);
  } catch (error) {
    navigationReport(error.message);
  }
};
$('startGoal').onclick = () => {
  if ($('startGoal').disabled) return;
  if (demo) {
    navigationReport('오프라인 테스트: 목적지 전송 동작 확인 · 로봇 명령 없음');
    return;
  }
  try {
    primaryState().navigation.start(goal);
  } catch (error) {
    navigationReport(error.message);
  }
};
$('stop').onclick = () => {
  selectionMode(false);
  if (demo) {
    navigationReport('오프라인 테스트: 정지 버튼 동작 확인 · 로봇 전송 없음');
    return;
  }
  const targets = activeIndices().map(index => robotStates[index]);
  let sent = 0;
  const failures = [];
  for (const state of targets) {
    try {
      state.navigation.stop();
      sent++;
    } catch (error) {
      failures.push(`${robotName(state.index)}: ${error.message}`);
    }
  }
  if (selectedMode === 'all') navigationReport(
    `ALL 정지 · ${sent}대에 주행 취소와 속도 0 전송${failures.length?` · ${failures.join(' / ')}`:''}`);
};
$('help').onclick = () => $('helpDialog').showModal();
$('closeHelp').onclick = () => $('helpDialog').close();

// 배터리, 카메라, 좌표계와 토픽 상태를 대시보드에 반영한다.
function batteryInfo(state) {
  const raw = state.messages.battery?.data,
    age = Date.now() - (state.last.battery || 0),
    valid = Number.isInteger(raw) && raw >= 0 && raw <= 65535,
    connected = demo || state.ros?.isConnected;
  if (!valid || age > 10000 || !connected) return {
    text: '—',
    status: !connected ? '연결 끊김' : !state.last.battery ? '수신 대기' : !valid ? '값 확인 필요' : '수신 지연',
    low: false
  };
  const voltage = raw / 10,
    percent = Math.round(Math.max(0, Math.min(100, 10 + (voltage - 6.5) * 60)));
  return {
    text: `${percent}%`,
    status: voltage <= 6.5 ? '저전압' : '정상',
    low: voltage <= 6.5
  };
}

function renderCameras() {
  const indices = activeIndices(),
    slots = [0, 1];
  for (const slot of slots) {
    const panel = $(`cameraPanel${slot}`),
      state = robotStates[indices[slot]];
    panel.hidden = !state || selectedMode !== 'all' && slot === 1;
    if (!state) continue;
    $(`cameraLabel${slot}`).textContent = robotName(state.index);
    const image = $(`camera${slot}`),
      empty = $(`cameraEmpty${slot}`);
    if (state.cameraUrl) {
      image.src = state.cameraUrl;
      image.hidden = false;
      empty.hidden = true;
    } else {
      image.removeAttribute('src');
      image.hidden = true;
      empty.hidden = false;
    }
  }
}

function mapsMatch() {
  const a = robotStates[0].messages.map,
    b = robotStates[1].messages.map;
  if (!a || !b) return {
    ok: null,
    text: '공통 /map 확인 대기'
  };
  const frameA = normalize(a.header?.frame_id),
    frameB = normalize(b.header?.frame_id),
    fixedA = normalize(profiles[0].fixed),
    fixedB = normalize(profiles[1].fixed);
  if (frameA !== frameB || fixedA !== fixedB || frameA !== fixedA) return {
    ok: false,
    text: `좌표계 불일치: R1 ${frameA||'?'} / R2 ${frameB||'?'}`
  };
  const ia = a.info,
    ib = b.info,
    oa = ia.origin,
    ob = ib.origin,
    close = (x, y) => Math.abs(x - y) < 1e-5,
    same = ia.width === ib.width && ia.height === ib.height && close(ia.resolution, ib
      .resolution) && ['x', 'y', 'z'].every(k => close(oa.position[k], ob.position[k])) && ['x',
      'y',
      'z', 'w'
    ].every(k => close(oa.orientation[k], ob.orientation[k]));
  return same ? {
    ok: true,
    text: `공통 /${frameA} 좌표계 · 지도 메타데이터 일치`
  } : {
    ok: false,
    text: `/${frameA} 이름은 같지만 지도 원점/크기가 다릅니다.`
  };
}

function updateDashboard() {
  const states = activeIndices().map(index => robotStates[index]),
    all = selectedMode === 'all';
  $('topicStatus').replaceChildren(...states.flatMap(state => ['map', 'scan', 'path', 'odom', 'tf',
    'amcl', 'camera'
  ].map(key => {
    const element = document.createElement('span'),
      age = state.last[key] ? (Date.now() - state.last[key]) / 1000 : Infinity,
      label = {
        map: '지도',
        scan: '라이다',
        path: '경로',
        odom: '속도',
        tf: 'TF',
        amcl: '위치',
        camera: '카메라'
      } [key];
    element.textContent =
      `${all?`R${state.index+1} `:''}${label} · ${age===Infinity?'대기':age<3?'수신 중':`${Math.floor(age)}초 전`}`;
    element.className = age < 3 ? 'live' : '';
    return element;
  })));
  const speed = state => {
      const value = state.messages.odom?.twist?.twist?.linear;
      return value && Date.now() - state.last.odom < 3000 ? Math.hypot(value.x, value.y).toFixed(
        2) : '—';
    },
    position = state => {
      const base = estimatedBase(state);
      if (!base) return '—';
      const value = new T.Vector3().setFromMatrixPosition(base);
      return `${value.x.toFixed(2)}, ${value.y.toFixed(2)}`;
    };
  $('speed').textContent = states.map(state => `${all?`R${state.index+1} `:''}${speed(state)}`)
    .join(' / ');
  $('position').textContent = states.map(state =>
    `${all?`R${state.index+1} `:''}${position(state)}`).join(' / ');
  const batteries = states.map(batteryInfo);
  $('battery').textContent = states.map((state, index) =>
    `${all?`R${state.index+1} `:''}${batteries[index].text}`).join(' / ');
  $('batteryStatus').textContent = states.map((state, index) =>
    `${all?`R${state.index+1} `:''}${batteries[index].status}`).join(' · ');
  $('battery').classList.toggle('low', batteries.some(item => item.low));
  if (all) {
    const compatibility = mapsMatch();
    $('frameStatus').textContent = compatibility.text;
    $('frameStatus').classList.toggle('frame-warning', compatibility.ok === false);
  } else {
    const state = states[0],
      p = profiles[state.index],
      hasTf = !!transform(state, p.base);
    $('frameStatus').textContent = hasTf ? `${p.fixed} → ${p.base}` : state.messages.amcl ?
      `${p.fixed} 위치 추정 수신 · TF 대기` : `기체 TF 대기: ${p.base}`;
    $('frameStatus').classList.remove('frame-warning');
  }
  $('cameraStatus').textContent = states.map(state =>
      `R${state.index+1} ${state.last.camera?Date.now()-state.last.camera<3000?'수신 중':'지연':'대기'}`)
    .join(' · ');
  for (const state of states) {
    if (state.initialPoseSentAt && state.last.amcl >= state.initialPoseSentAt) {
      navigationReport('초기 위치 적용 확인 · 위치 추정 응답 수신');
      state.initialPoseSentAt = 0;
    } else if (state.initialPoseSentAt && Date.now() - state.initialPoseSentAt > 5000) {
      navigationReport('초기 위치 응답 없음 · 위치 추정 모듈을 확인하세요.');
      state.initialPoseSentAt = 0;
    }
  }
}

let lastUI = 0;

// 각 프레임에서 로봇과 센서 객체를 갱신하고 장면을 렌더링한다.
function animate(now) {
  requestAnimationFrame(animate);
  const rect = $('viewport').getBoundingClientRect();
  if (renderer.domElement.width !== Math.round(rect.width * renderer.getPixelRatio()) || renderer
    .domElement.height !== Math.round(rect.height * renderer.getPixelRatio())) {
    renderer.setSize(rect.width, rect.height, false);
    camera.aspect = rect.width / rect.height;
    camera.updateProjectionMatrix();
  }
  grid.visible = enabled.grid;
  const active = activeIndices();
  for (const state of robotStates) {
    state.root.visible = active.includes(state.index);
    if (!state.root.visible) continue;
    const p = profiles[state.index],
      all = selectedMode === 'all';
    for (const key of ['map', 'path', 'scan']) {
      const message = state.messages[key],
        tf = message && transform(state, message.header.frame_id),
        fresh = key !== 'scan' || Date.now() - (state.last.scan || 0) < 3000;
      state.groups[key].visible = !!(enabled[key] && tf && fresh && !(all && key === 'map' && state
        .index === 1 && robotStates[0].messages.map));
      if (tf) place(state.groups[key], key === 'map' ? tf.clone().multiply(state.messages
        .mapOrigin) : tf);
    }
    if (!state.mapFitted && state.messages.map && state.index === active[0]) fit();
    const base = estimatedBase(state);
    state.groups.robot.visible = enabled.robot && !!base;
    if (base) {
      if (state.modelLinks.length) {
        place(state.groups.robot, new T.Matrix4());
        const lookup = frame => normalize(frame) === normalize(p.base) ? base : transform(state,
          frame);
        for (const link of state.modelLinks) {
          const mat = modelTransform(link.frame, lookup, state.modelJoints);
          link.group.visible = !!mat;
          if (mat) place(link.group, mat);
        }
      } else place(state.groups.robot, base);
    }
    if (now - (state.lastTfRender || 0) > 500) {
      state.lastTfRender = now;
      disposeGroup(state.groups.tf);
      if (enabled.tf)
        for (const frame of state.transforms.keys()) {
          const mat = transform(state, frame);
          if (mat) {
            const axes = new T.AxesHelper(.3);
            place(axes, mat);
            state.groups.tf.add(axes);
          }
        }
    }
    state.groups.tf.visible = enabled.tf;
  }
  if (now - lastUI > 500) {
    lastUI = now;
    updateDashboard();
    renderConnection();
    renderModelStatus();
    const states = active.map(index => robotStates[index]);
    if (states.some(state => Object.keys(state.last).length)) $('notice').textContent = demo ?
      '오프라인 테스트 데이터 · 실제 로봇 상태가 아닙니다.' : selectedMode === 'all' ? mapsMatch().text :
      '실제 ROS 데이터 표시 · 마지막 수신 시간은 아래에서 확인하세요';
  }
  updateGoalButtons();
  controls.update();
  renderer.render(scene, camera);
}

if (demo) {
  document.querySelector('.demo-link').textContent = '실제 연결 화면';
  document.querySelector('.demo-link').href = './';
}
$('robotTitle').textContent = selectedMode === 'all' ? 'Robot 1 + Robot 2' : robotName(Number(
  selectedMode));
$('settings').disabled = selectedMode === 'all';
$('navigationStatus').textContent = selectedMode === 'all' ?
  'ALL에서는 새 주행을 시작할 수 없으며 정지는 두 로봇에 전달됩니다.' : primaryState().navigationStatus;
renderRobots();
reconcileConnections();
updateGoalButtons();
requestAnimationFrame(animate);
