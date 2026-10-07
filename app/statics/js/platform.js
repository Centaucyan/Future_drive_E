const quaternion = yaw => ({x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2)});
const position = pose => ({x: Number(pose?.x || 0), y: Number(pose?.y || 0), z: Number(pose?.z || 0)});

export class PlatformConnection {
  constructor(baseUrl, receive, handlers = {}) {
    this.baseUrl = baseUrl.replace(/\/+$/, '');
    this.receive = receive;
    this.handlers = handlers;
    this.socket = null;
    this.isConnected = false;
    this.snapshot = null;
    this.cameraStamp = null;
    this.retryTimer = null;
    this.stopped = false;
  }

  connect() {
    this.stopped = false;
    clearTimeout(this.retryTimer);
    this.retryTimer = null;
    const url = new URL(this.baseUrl);
    url.protocol = url.protocol === 'https:' ? 'wss:' : 'ws:';
    url.pathname = url.pathname.replace(/\/+$/, '') + '/ws/state';
    const socket = this.socket = new WebSocket(url);
    socket.binaryType = 'arraybuffer';
    socket.onopen = () => {
      if (socket !== this.socket) return;
      this.isConnected = true;
      this.handlers.open?.();
    };
    socket.onerror = () => socket === this.socket && this.handlers.error?.();
    socket.onclose = () => {
      if (socket !== this.socket) return;
      this.isConnected = false;
      this.socket = null;
      this.handlers.close?.();
      if (!this.stopped) this.retryTimer = setTimeout(() => this.connect(), 2000);
    };
    socket.onmessage = event => {
      if (socket !== this.socket) return;
      try {
        if (typeof event.data === 'string') this.handleSnapshot(JSON.parse(event.data));
        else this.handleBinary(event.data);
      } catch (error) {
        this.handlers.messageError?.(error);
      }
    };
  }

  handleSnapshot(snapshot) {
    this.snapshot = snapshot;
    const pose = snapshot.pose_map || (snapshot.pose?.frame_id === 'map' ? snapshot.pose : null);
    if (pose) {
      this.receive('amcl', {
        header: {frame_id: 'map'},
        pose: {pose: {position: position(pose), orientation: quaternion(Number(pose.yaw || 0))}}
      });
    }
    if (snapshot.pose) {
      const value = snapshot.pose;
      this.receive('odom', {
        header: {frame_id: value.frame_id || 'odom'},
        child_frame_id: value.child_frame_id || 'base_footprint',
        pose: {pose: {position: position(value), orientation: quaternion(Number(value.yaw || 0))}},
        twist: {twist: {linear: {x: Number(value.linear || 0), y: 0, z: 0}, angular: {x: 0, y: 0, z: Number(value.angular || 0)}}}
      });
    }
    if (snapshot.plan) {
      this.receive('path', {
        header: {frame_id: snapshot.plan.frame_id || 'map'},
        poses: (snapshot.plan.poses || []).map(value => ({pose: {position: position(value), orientation: quaternion(Number(value.yaw || 0))}}))
      });
    }
    const scanPose = snapshot.scan_pose_map;
    if (scanPose) {
      this.receive('tf', {transforms: [{
        header: {frame_id: 'map'},
        child_frame_id: snapshot.scan?.frame_id || scanPose.source_frame_id || 'laser_frame',
        transform: {translation: position(scanPose), rotation: quaternion(Number(scanPose.yaw || 0))}
      }]});
    }
    if (snapshot.camera_image?.t && snapshot.camera_image.t !== this.cameraStamp) {
      this.cameraStamp = snapshot.camera_image.t;
      this.receive('platformCamera', {url: `${this.baseUrl}/camera-frame?t=${encodeURIComponent(this.cameraStamp)}`});
    }
    this.handlers.snapshot?.(snapshot);
  }

  handleBinary(buffer) {
    const view = new DataView(buffer);
    if (view.byteLength < 8) return;
    const magic = String.fromCharCode(...new Uint8Array(buffer, 0, 4));
    if (magic === 'SCN1') {
      const count = view.getUint32(4, true), metadata = this.snapshot?.scan;
      if (!metadata || view.byteLength !== 8 + count * 4) return;
      const ranges = Array.from(new Float32Array(buffer, 8, count));
      this.receive('scan', {...metadata, ranges, header: {frame_id: metadata.frame_id}});
    } else if (magic === 'MAP1') {
      if (view.byteLength < 12) return;
      const width = view.getUint32(4, true), height = view.getUint32(8, true), metadata = this.snapshot?.map;
      if (!metadata || width * height !== view.byteLength - 12) return;
      const yaw = Number(metadata.origin?.yaw || 0);
      this.receive('map', {
        header: {frame_id: metadata.frame_id || 'map'},
        info: {
          width, height, resolution: Number(metadata.resolution),
          origin: {position: position(metadata.origin), orientation: quaternion(yaw)}
        },
        data: Array.from(new Int8Array(buffer, 12, width * height))
      });
    }
  }

  close() {
    this.stopped = true;
    clearTimeout(this.retryTimer);
    this.retryTimer = null;
    const socket = this.socket;
    this.socket = null;
    this.isConnected = false;
    if (socket) {
      socket.onclose = null;
      socket.close();
    }
  }
}
