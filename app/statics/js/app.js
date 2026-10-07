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
import { PlatformConnection } from './platform.js';

// 기본 도우미와 로봇별 연결 설정을 준비한다.
const $ = id => document.getElementById(id);
const normalize = s => (s || '').replace(/^\//, '');
const robotName = index => `Robot ${index+1}`;
const errorMessage = error => error instanceof Error ? error.message : String(error);

// 브라우저 보안 설정이나 시크릿 모드에서 localStorage 접근이 실패해도
// 화면 초기화 전체가 중단되지 않도록 읽기/쓰기를 안전하게 감싼다.
function readStorage(key) {
  try {
    return localStorage.getItem(key);
  } catch (error) {
    console.warn(`로컬 설정을 읽지 못했습니다 (${key}):`, error);
    return null;
  }
}
function writeStorage(key, value) {
  try {
    localStorage.setItem(key, value);
    return true;
  } catch (error) {
    console.warn(`로컬 설정을 저장하지 못했습니다 (${key}):`, error);
    $('notice').textContent = '브라우저 저장소를 사용할 수 없어 연결 설정이 저장되지 않았습니다.';
    return false;
  }
}
let serverConfig = {};
try {
  // 설정 API가 실패하거나 잘못된 JSON을 반환하면 아래 catch에서 기본값으로 계속 실행한다.
  const response = await fetch('/api/viewer-config');
  if (!response.ok) throw Error(`HTTP ${response.status}`);
  const config = await response.json();
  if (!config || typeof config !== 'object' || Array.isArray(config)) throw Error('응답 형식 오류');
  serverConfig = config;
} catch (error) {
  console.warn('서버 설정을 불러오지 못해 기본값을 사용합니다:', error);
}
const defaults = {
  platformUrl: serverConfig.platform_bridge_url || '',
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
  battery: '/battery',
  navigationStatus: '/navigate_to_pose/_action/status'
};
let profiles = [{
  ...defaults,
  name: 'Robot 1'
}, {
  ...defaults,
  name: 'Robot 2',
  platformUrl: ''
}, {
  ...defaults,
  name: 'Robot 3',
  platformUrl: ''
}];
try {
  const raw = readStorage('yahboom-viewer-v1');
  const saved = raw ? JSON.parse(raw) : null;
  if (Array.isArray(saved) && saved.length === 3) profiles = saved.map((p, i) => {
    const stored = p && typeof p === 'object' && !Array.isArray(p) ? p : {};
    const validFields = Object.fromEntries(Object.keys(defaults).filter(key => typeof stored[key] ===
      'string').map(key => [key, stored[key]]));
    const merged = {
      ...profiles[i],
      ...validFields,
      name: robotName(i)
    };
    delete merged.robotIp;
    delete merged.ip;
    if (merged.controlApi === 'http://127.0.0.1:8765') merged.controlApi = '/api/control';
    return merged;
  });
} catch (error) {
  console.warn('저장된 연결 설정이 손상되어 기본값을 사용합니다:', error);
}
const demo = new URLSearchParams(location.search).has('demo');
let selectedMode = readStorage('yahboom-selected') || '0';
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
    epoch: 0,
    last: {},
    messages: {},
    topicStatus: {},
    platformSnapshot: null,
    transforms: new Map(),
    mapMesh: null,
    mapFitted: false,
    modelLinks: [],
    modelJoints: new Map(),
    modelEpoch: 0,
    initialPoseSentAt: 0,
    demoTimer: null,
    navigationStateKnown: demo,
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

// Each entry owns its platform connection, Navigation instance, TF tree and Three.js groups.
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
  // 잘못된 CompressedImage 한 건 때문에 전체 ROS 메시지 처리가 멈추지 않도록
  // 이미지 형식과 데이터 유무를 변환 전에 명확하게 검사한다.
  if (!message || typeof message !== 'object') throw Error('카메라 메시지가 없습니다.');
  if (!/jpeg|jpg|png/i.test(message.format || '')) throw Error(
    `지원하지 않는 이미지 형식: ${message.format||'없음'}`);
  if (message.data == null || message.data.length === 0) throw Error('카메라 데이터가 비어 있습니다.');
  let data = message.data;
  if (typeof data !== 'string') {
    let bytes;
    try {
      bytes = data instanceof Uint8Array ? data : Uint8Array.from(data);
    } catch (error) {
      throw Error(`카메라 데이터 변환 실패: ${errorMessage(error)}`);
    }
    let binary = '';
    // 큰 이미지를 한 번에 펼치면 브라우저 호출 스택을 초과할 수 있어 32KB씩 변환한다.
    for (let offset = 0; offset < bytes.length; offset += 32768) binary += String.fromCharCode(...bytes
      .subarray(offset, offset + 32768));
    data = btoa(binary);
  }
  const mime = /png/i.test(message.format) ? 'image/png' : 'image/jpeg';
  return `data:${mime};base64,${data}`;
}

function receive(state, key, message) {
  const cameraUrl = key === 'camera' ? imageUrl(message) : null;
  if (key === 'description' && state.messages.description?.data !== message.data) loadModel(state,
    message.data);
  state.last[key] = Date.now();
  state.messages[key] = message;
  if (key === 'navigationStatus') {
    // 새로고침하면 브라우저의 goal ID는 사라지지만 로봇 주행은 계속될 수 있다.
    // Nav2 상태 1(수락), 2(실행), 3(취소 중)을 실제 주행 중으로 복원한다.
    const statuses = Array.isArray(message.status_list) ? message.status_list : [];
    const remoteBusy = statuses.some(item => [1, 2, 3].includes(item?.status));
    const firstResponse = !state.navigationStateKnown;
    const statusChanged = state.navigation.remoteBusy !== remoteBusy;
    state.navigationStateKnown = true;
    state.navigation.setRemoteBusy(remoteBusy);
    if (firstResponse || statusChanged) {
      state.navigationStatus = remoteBusy ?
        '로봇에서 진행 중인 주행을 확인했습니다. 새 목적지를 보내려면 먼저 정지하세요.' :
        '로봇 주행 상태 확인 완료 · 목적지를 선택할 수 있습니다.';
      if (isVisible(state) && selectedMode !== 'all') navigationReport(state.navigationStatus, false);
    }
    updateGoalButtons();
    return;
  }
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
    state.cameraUrl = cameraUrl;
    if (isVisible(state)) renderCameras();
  }
  if (key === 'platformCamera') {
    state.cameraUrl = message.url;
    state.last.camera = Date.now();
    if (isVisible(state)) renderCameras();
  }
}

function resetState(state, {
  closeNavigation = true
} = {}) {
  if (state.ros) {
    try {
      state.ros.close();
    } catch (error) {
      console.warn(`${robotName(state.index)} 플랫폼 브리지 종료 실패:`, error);
    }
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
  state.topicStatus = {};
  state.platformSnapshot = null;
  state.transforms.clear();
  state.mapMesh = null;
  state.mapFitted = false;
  state.cameraUrl = '';
  state.navigationStateKnown = demo;
  state.navigation.setRemoteBusy(false);
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

// 로봇별 Autonomy Studio 상태 스트림 연결 생명주기를 관리한다.

function connectState(state) {
  const p = profiles[state.index];
  state.root.visible = true;
  if (demo) {
    if (state.demoTimer) return;
    state.status = '테스트 데이터';
    state.navigationStateKnown = true;
    state.demoTimer = startDemo((key, message) => receive(state, key, message));
    fetch('/static/robot_models/yahboom_vehicle/yahboom.urdf').then(response => {
      if (!response.ok) throw Error(`HTTP ${response.status}`);
      return response.text();
    }).then(text => loadModel(state, text)).catch(error => {
      console.warn('데모 URDF를 불러오지 못했습니다:', error);
      state.modelError = `URDF 요청 실패: ${errorMessage(error)}`;
      renderModelStatus();
    });
    return;
  }
  if (state.ros && (state.ros.isConnected || state.status === '연결 중…')) return;
  resetState(state, {closeNavigation: false});
  state.root.visible = true;
  const token = ++state.epoch;
  if (!p.platformUrl || !/^https?:\/\//.test(p.platformUrl)) {
    state.status = '플랫폼 주소 미설정';
    return;
  }
  state.status = '연결 중…';
  p.controlApi = p.platformUrl.replace(/\/+$/, '');
  state.navigation.connectPlatform(p.controlApi);
  try {
    state.ros = new PlatformConnection(p.platformUrl, (key, message) => {
      if (token !== state.epoch) return;
      try {
        receive(state, key, message);
      } catch (error) {
        console.warn(`${robotName(state.index)} ${key} 메시지 처리 실패:`, error);
        $('notice').textContent = `${robotName(state.index)} ${key}: ${errorMessage(error)}`;
      }
    }, {
      open() {
        if (token !== state.epoch) return;
        state.status = '● 연결됨';
        renderConnection();
        updateGoalButtons();
      },
      close() {
        if (token !== state.epoch) return;
        state.navigationStateKnown = false;
        state.status = '연결 끊김';
        navigationReport(`${robotName(state.index)} 플랫폼 연결 끊김`);
        renderConnection();
        updateGoalButtons();
      },
      error() {
        if (token !== state.epoch) return;
        state.status = '연결 실패';
        $('notice').textContent = `${robotName(state.index)} Autonomy Studio 브리지 주소를 확인하세요.`;
        renderConnection();
      },
      snapshot(snapshot) {
        if (token !== state.epoch) return;
        state.topicStatus = snapshot.topic_status || {};
        state.platformSnapshot = snapshot;
        const firstSnapshot = !state.navigationStateKnown;
        state.navigationStateKnown = true;
        state.navigation.sync(snapshot.navigation);
        if (firstSnapshot && !state.navigation.busy) {
          state.navigationStatus = '로봇 주행 상태 확인 완료 · 목적지를 선택할 수 있습니다.';
          if (isVisible(state) && selectedMode !== 'all') navigationReport(state.navigationStatus,
            false);
        }
        updateGoalButtons();
      },
      messageError(error) {
        if (token !== state.epoch) return;
        $('notice').textContent =
          `${robotName(state.index)} 플랫폼 데이터 오류: ${errorMessage(error)}`;
      }
    });
    state.ros.connect();
  } catch (error) {
    state.status = '연결 생성 실패';
    state.ros = null;
    $('notice').textContent = `${robotName(state.index)} 연결 생성 실패: ${errorMessage(error)}`;
    console.error(`${robotName(state.index)} 연결 생성 실패:`, error);
    renderConnection();
  }
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
  if (states.some(state => !profiles[state.index].platformUrl) && !demo) $('notice').textContent =
    selectedMode === 'all' ? '연결 설정에서 Robot 1과 Robot 2의 플랫폼 브리지를 먼저 준비하세요.' :
    '연결 설정에서 이 로봇의 플랫폼 브리지 주소를 입력하세요.';
}

// 단일 로봇/ALL 모드 선택과 연결 설정 화면을 처리한다.
function selectMode(mode) {
  clearDestination();
  selectedMode = mode;
  writeStorage('yahboom-selected', mode);
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
  platformUrl: 'Autonomy Studio 브리지 주소',
  fixed: '기준 좌표계',
  base: '기체 좌표계'
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
    if (key === 'platformUrl') input.placeholder = 'http://브리지-PC-IP:8765';
    element.append(input);
    return element;
  }));
  $('configDialog').showModal();
};
$('cancel').onclick = () => $('configDialog').close();
$('configForm').onsubmit = event => {
  event.preventDefault();
  if (selectedMode === 'all') return;
  const state = primaryState(), p = profiles[state.index];
  for (const [key, value] of new FormData(event.target)) p[key] = String(value).trim();
  if (!/^https?:\/\//.test(p.platformUrl)) {
    navigationReport('플랫폼 브리지 주소는 http:// 또는 https://로 시작해야 합니다.');
    state.status = '연결 설정 실패';
    renderConnection();
    return;
  }
  p.controlApi = p.platformUrl.replace(/\/+$/, '');
  writeStorage('yahboom-viewer-v1', JSON.stringify(profiles));
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
      .height * message.info.resolution / 2, 0).applyMatrix4(state.messages.mapOrigin).applyMatrix4(tf),
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
    '자율주행 대기: 제어 연결이 없습니다.' : !demo && !state.navigationStateKnown ?
    state.navigationStatus : state.navigation.remoteBusy ?
    '로봇에서 진행 중인 주행이 있습니다. 새 목적지를 보내려면 먼저 정지하세요.' : !demo && !estimatedBase(state) ?
    `자율주행 대기: ${profiles[state.index].fixed} 기준 위치 추정이 없습니다. 상단 초기 위치 버튼으로 현재 위치와 방향을 지정하세요.` : !
    goal ? '지도에서 목적지를 선택하세요.' : picking ? '목적지 선택을 마쳐주세요.' : '';
  $('navigationPrerequisite').textContent = all ? reason : isInitial ? (!goal ?
    '지도에서 로봇의 실제 위치를 누른 채 방향을 드래그하세요.' : !goal.headingSet ? '현재 방향을 드래그로 지정하세요.' : !demo && !
    navigation.ready ? '초기 위치 전송 대기: 제어 연결이 없습니다.' : '초기 위치 설정은 로봇을 이동시키지 않습니다.') : reason;
  $('startGoal').title = reason;
  $('startGoal').disabled = all || !goal || goal.kind !== 'goal' || picking || navigation.busy || !
    connected || (!demo && (!state.navigationStateKnown || !estimatedBase(state)));
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
      const source = state.cameraUrl;
      image.onerror = () => {
        // 메시지 형식은 정상이어도 JPEG/PNG 바이트가 손상된 경우 마지막 깨진 화면을 제거한다.
        if (state.cameraUrl !== source) return;
        console.warn(`${robotName(state.index)} 카메라 이미지를 표시할 수 없습니다.`);
        state.cameraUrl = '';
        image.removeAttribute('src');
        image.hidden = true;
        empty.hidden = false;
        $('notice').textContent = `${robotName(state.index)} 카메라 이미지가 손상되었습니다.`;
      };
      image.src = source;
      image.hidden = false;
      empty.hidden = true;
    } else {
      image.onerror = null;
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
  let cameraExpired = false;
  for (const state of states) {
    // 영상이 끊겼는데 마지막 프레임이 정상 화면처럼 남는 것을 막기 위해 3초 후 비운다.
    if (state.cameraUrl && Date.now() - (state.last.camera || 0) > 3000) {
      state.cameraUrl = '';
      cameraExpired = true;
    }
  }
  if (cameraExpired) renderCameras();
  const topicDefinitions = [
    {key: 'scan', fallback: '/scan', mode: 'live'},
    {key: 'odom', fallback: '/odom', mode: 'live'},
    {key: 'map', fallback: '/map', mode: 'latched'},
    {key: 'plan', fallback: '/plan', mode: 'endpoint'},
    {key: 'amcl_pose', fallback: '/amcl_pose', mode: 'live', receivedKey: 'amcl'},
    {key: 'tf', fallback: '/tf', mode: 'live'},
    {key: 'camera_image', fallback: '/image_raw/compressed', mode: 'live', receivedKey: 'camera'},
    {key: 'cmd_vel', fallback: '/cmd_vel', mode: 'endpoint'},
    {key: 'navigate_to_pose', fallback: '/navigate_to_pose', mode: 'endpoint'}
  ];
  const topicRows = states.flatMap(state => topicDefinitions.map(definition => {
    const info = state.topicStatus[definition.key] || {};
    const receivedKey = definition.receivedKey || ({plan: 'path'}[definition.key] || definition.key);
    const last = state.last[receivedKey] || 0;
    const age = last ? (Date.now() - last) / 1000 : Infinity;
    const connected = demo || state.ros?.isConnected;
    const available = info.available === true || (!(definition.key in state.topicStatus) && age < 3);
    let level = 'bad', text = '미수신';
    if (!connected) text = '연결 끊김';
    else if (demo) { level = age < 3 ? 'ok' : 'wait'; text = age < 3 ? '테스트 수신' : '대기'; }
    else if (definition.mode === 'live') {
      const bridgeReported = definition.key in state.topicStatus;
      if (info.available === true) { level = 'ok'; text = age < 3 ? '정상' : 'ROS 수신 중'; }
      else if (bridgeReported) { level = 'bad'; text = '미수신'; }
      else if (age < 3) { level = 'ok'; text = '정상'; }
      else if (last) { level = 'warn'; text = `${Math.floor(age)}초 전 · 지연`; }
    } else if (definition.mode === 'latched') {
      if (available || last) { level = 'ok'; text = '수신됨'; }
    } else if (available) { level = 'ok'; text = '사용 가능'; }
    const row = document.createElement('div');
    row.className = `topic-row ${level}`;
    const name = info.name || definition.fallback;
    row.innerHTML = `<code>${all?`R${state.index+1} `:''}${name}</code><b>${level==='ok'?'●':level==='warn'?'▲':'×'} ${text}</b>`;
    return row;
  }));
  $('topicStatus').replaceChildren(...topicRows);
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
