export function initialPoseMessage(pose, nowMs = Date.now()) {
  if (![pose.x,pose.y,pose.z,pose.yaw].every(Number.isFinite) || !pose.frame) throw Error('초기 위치 좌표를 확인하세요.');
  const covariance=Array(36).fill(0);covariance[0]=.25;covariance[7]=.25;covariance[35]=(Math.PI/12)**2;
  const sec=Math.floor(nowMs/1000),nanosec=Math.floor((nowMs-sec*1000)*1e6);
  return {header:{frame_id:pose.frame,stamp:{sec,nanosec}},pose:{pose:{position:{x:pose.x,y:pose.y,z:pose.z},orientation:{x:0,y:0,z:Math.sin(pose.yaw/2),w:Math.cos(pose.yaw/2)}},covariance}};
}
// ROS 2 rosbridge action protocol. This module never sends a goal on connection.
export class Navigation {
  constructor(report, changed) { this.report=report; this.changed=changed; this.socket=null; this.active=null; this.pendingStop=null; this.timers=[]; }
  get busy(){return !!(this.active||this.pendingStop);}
  connect(url, config) {
    this.close(); this.config=config;
    if(!url)return;
    const socket=this.socket=new WebSocket(url);
    socket.onmessage=event=>{if(socket!==this.socket)return;let m;try{m=JSON.parse(event.data);}catch{return;}
      if(m.op==='action_feedback'&&m.id===this.active){this.report('자율주행 중 · 남은 거리 '+Number(m.values.distance_remaining||0).toFixed(2)+' m');}
      if(m.op==='action_result'&&m.id===this.active){this.active=null;this.report(m.result&&m.status===4?'목적지 도착':m.status===5?'주행 취소 확인':'주행 실패: '+(typeof m.values==='string'?m.values:'상태 '+m.status));this.changed();}
      if(m.op==='service_response'&&m.id===this.pendingStop){this.pendingStop=null;const ok=m.result!==false&&m.values?.return_code===0;this.report(ok?'주행 취소 응답 수신 · 실제 정지 상태를 확인하세요.':'정지 확인 실패 · 로봇/플랫폼에서 상태를 확인하세요.');this.changed();}
      if(m.op==='status'&&m.level==='error')this.report('제어 오류: '+m.msg);
    };
    socket.onopen=()=>{if(socket!==this.socket)return;this.send({op:'advertise',topic:this.config.initialpose,type:'geometry_msgs/msg/PoseWithCovarianceStamped',qos:{history:'keep_last',depth:10,reliability:'reliable',durability:'volatile'}});this.changed();};
    socket.onclose=()=>{if(socket===this.socket){this.report(this.busy?'제어 연결 끊김 · 주행 종료를 확인하지 못했습니다.':'제어 연결 끊김');this.changed();}};
    socket.onerror=()=>{if(socket===this.socket)this.report('제어 연결 오류');};
  }
  get ready(){return this.socket?.readyState===WebSocket.OPEN;}
  send(message){if(!this.ready)throw Error('제어 연결이 없습니다.');this.socket.send(JSON.stringify(message));}
  setInitialPose(pose) {
    if(this.busy)throw Error('주행을 정지한 뒤 초기 위치를 설정하세요.');
    const controlApi=(this.config.controlApi||'').replace(/\/+$/,'');
    if(controlApi){
      this.report('초기 위치 전송 중 · 플랫폼 제어 API 응답을 기다립니다.');
      fetch(controlApi+'/initialpose',{
        method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify({x:pose.x,y:pose.y,yaw:pose.yaw,frame_id:pose.frame})
      }).then(async response=>{
        if(!response.ok){let detail='HTTP '+response.status;try{detail=(await response.json()).detail||detail;}catch{}throw Error(detail);}
        this.report('초기 위치 전송 완료 · AMCL 위치 응답과 TF 갱신을 기다립니다.');
        this.changed();
      }).catch(error=>{
        this.report('초기 위치 전송 실패 · 플랫폼 제어 API 확인: '+error.message);
        this.changed();
      });
      return;
    }
    const message=initialPoseMessage(pose);
    for(let i=0;i<3;i++)this.timers.push(setTimeout(()=>{
      if(this.ready)this.send({op:'publish',topic:this.config.initialpose,msg:message});
    },i*150));
    this.report('초기 위치 전송 중 · AMCL 위치 응답과 TF 갱신을 기다립니다.');
  }
  start(goal) {
    if(this.busy)throw Error('진행 중인 주행을 먼저 정지하세요.');
    const id='goal:'+crypto.randomUUID();
    this.send({op:'send_action_goal',id,action:this.config.action,action_type:'nav2_msgs/action/NavigateToPose',feedback:true,args:{pose:{header:{frame_id:goal.frame,stamp:{sec:0,nanosec:0}},pose:{position:{x:goal.x,y:goal.y,z:goal.z},orientation:{x:0,y:0,z:Math.sin(goal.yaw/2),w:Math.cos(goal.yaw/2)}}},behavior_tree:''}});
    this.active=id;this.report('목적지 전송 · Nav2 응답 대기');this.changed();
  }
  stop() {
    if(this.pendingStop)return;
    if(!this.ready)throw Error('연결이 없어 정지 명령을 전달할 수 없습니다. 플랫폼에서 정지하세요.');
    const id='stop:'+crypto.randomUUID();
    // Also cancel our own pending goal after acceptance, avoiding the cancel-before-accept race.
    if(this.active)this.send({op:'cancel_action_goal',id:this.active,action:this.config.action});
    this.send({op:'call_service',id,service:this.config.action+'/_action/cancel_goal',type:'action_msgs/srv/CancelGoal',args:{goal_info:{goal_id:{uuid:Array(16).fill(0)},stamp:{sec:0,nanosec:0}}}});
    this.pendingStop=id;
    this.send({op:'advertise',topic:this.config.cmd,type:'geometry_msgs/msg/Twist'});
    const zero={linear:{x:0,y:0,z:0},angular:{x:0,y:0,z:0}};
    for(let i=0;i<6;i++)this.timers.push(setTimeout(()=>{if(this.ready)this.send({op:'publish',topic:this.config.cmd,msg:zero});},i*100));
    this.timers.push(setTimeout(()=>{if(this.ready)this.send({op:'unadvertise',topic:this.config.cmd});},700));
    this.timers.push(setTimeout(()=>{if(this.pendingStop===id){this.pendingStop=null;this.report('정지 응답 시간 초과 · 실제 로봇 상태를 확인하고 다시 정지하세요.');this.changed();}},6000));
    this.report('정지 요청 중 · 주행 취소 및 속도 0 전송');this.changed();
  }
  close(){for(const t of this.timers)clearTimeout(t);this.timers=[];if(this.socket){this.socket.onclose=null;this.socket.close();}this.socket=null;this.active=null;this.pendingStop=null;}
}
