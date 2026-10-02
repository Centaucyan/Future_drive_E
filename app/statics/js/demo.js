// Browser-only fixtures. No ROS publisher, socket, or robot commands are used.
export function startDemo(receive){
 const q={x:0,y:0,z:0,w:1},header=frame_id=>({frame_id,stamp:{sec:0,nanosec:0}});
 const width=120,height=90,resolution=.1,data=new Array(width*height).fill(0);
 for(let y=0;y<height;y++)for(let x=0;x<width;x++){if(x<2||y<2||x>117||y>87||(x>42&&x<47&&y>20&&y<65)||(x>78&&x<99&&y>48&&y<53))data[y*width+x]=100;}
 receive('map',{header:header('map'),info:{width,height,resolution,origin:{position:{x:-6,y:-4.5,z:0},orientation:q}},data});
 receive('static',{transforms:[{header:header('base_footprint'),child_frame_id:'laser_frame',transform:{translation:{x:0,y:0,z:.16},rotation:q}}]});
 receive('path',{header:header('map'),poses:[[-4,-2],[-2.8,-2],[-2.8,2.5],[1,2.5],[3.5,1.8]].map(([x,y])=>({pose:{position:{x,y,z:0},orientation:q}}))});
 const canvas=document.createElement('canvas');canvas.width=640;canvas.height=360;const ctx=canvas.getContext('2d');
 let t=0;const tick=()=>{ctx.fillStyle='#101d30';ctx.fillRect(0,0,640,360);ctx.strokeStyle='#2d4563';for(let x=0;x<640;x+=40){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,360);ctx.stroke();}ctx.fillStyle='#40dcb3';ctx.font='26px sans-serif';ctx.fillText('OFFLINE CAMERA TEST',135,155);ctx.font='16px sans-serif';ctx.fillText('Synthetic frame · no robot connection',170,195);receive('camera',{format:'jpeg',data:canvas.toDataURL('image/jpeg',.6).split(',')[1]});t+=.04;const yaw=.15*Math.sin(t),rotation={x:0,y:0,z:Math.sin(yaw/2),w:Math.cos(yaw/2)};
 receive('tf',{transforms:[{header:header('map'),child_frame_id:'base_footprint',transform:{translation:{x:-3.8+.15*Math.sin(t),y:-2,z:.05},rotation}}]});
 receive('battery',{data:74});
 receive('odom',{header:header('odom'),twist:{twist:{linear:{x:.18,y:0,z:0}}}});
 receive('scan',{header:header('laser_frame'),range_min:.1,range_max:12,angle_min:-Math.PI,angle_increment:Math.PI/180,ranges:Array.from({length:360},(_,i)=>2.4+.45*Math.sin(i*.08))});};tick();return setInterval(tick,150);
}
