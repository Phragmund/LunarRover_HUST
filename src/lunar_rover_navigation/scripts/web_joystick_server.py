#!/usr/bin/env python3
"""Phone web-joystick remote for the lunar rover.

Serves a touch joystick page and publishes lunar_rover_msgs/ManualControl
(LX/LY/RX 杆量) to /manual_control, the same topic the keyboard controller uses.
The motion_solver node subscribes to it and drives the H7.

Control model:
  * Joystick horizontal -> LX (right positive, Ackermann steering).
  * Joystick vertical   -> LY (up positive, throttle).
  * Spin hold buttons   -> RX (+right spin / -left spin).
  * Estop button        -> latched zero until the arm button is pressed.
"""

import asyncio
import base64
import json
import math
import secrets
import socket
import struct
import subprocess
import sys
import threading
import time
from contextlib import asynccontextmanager

import rclpy
from lunar_rover_msgs.msg import ManualControl
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray

try:
    import uvicorn
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.responses import HTMLResponse
except ImportError as exc:  # pragma: no cover
    sys.stderr.write(
        """web_joystick_server 缺少 Web 依赖 (fastapi / uvicorn)。请先安装：
  pip3 install --user 'uvicorn[standard]' fastapi
"""
    )
    raise SystemExit(1) from exc


HTML_PAGE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no, viewport-fit=cover">
<title>月球车遥控</title>
<style>
  * { box-sizing: border-box; -webkit-user-select: none; user-select: none; -webkit-tap-highlight-color: transparent; }
  html, body { margin: 0; min-height: 100%; background: #0f172a; color: #e2e8f0; font-family: system-ui, -apple-system, "Segoe UI", Roboto, sans-serif; }
  body { min-height: 100svh; display: grid; place-items: center; padding: max(8px, env(safe-area-inset-top)) max(10px, env(safe-area-inset-right)) max(8px, env(safe-area-inset-bottom)) max(10px, env(safe-area-inset-left)); overflow: hidden; }
  #app { width: min(1120px, 100%); height: min(680px, calc(100svh - 16px)); display: grid; grid-template-columns: minmax(250px, 0.92fr) minmax(330px, 1.08fr); gap: clamp(12px, 2.5vw, 30px); align-items: center; }
  .drive-panel { min-width: 0; height: 100%; display: grid; grid-template-rows: auto auto minmax(0, 1fr); justify-items: center; }
  .control-panel { min-width: 0; align-self: stretch; display: flex; flex-direction: column; justify-content: center; }
  header { width: 100%; display: flex; align-items: center; justify-content: space-between; margin-bottom: 4px; }
  h1 { font-size: clamp(15px, 2vw, 19px); margin: 0; }
  #status { font-size: 11px; padding: 3px 8px; border-radius: 999px; background: #334155; color: #94a3b8; }
  #status.on { background: #052e16; color: #22c55e; }
  .tab-link { flex: 0 0 auto; height: 28px; padding: 3px 9px; border-radius: 8px; background: #334155; font-size: 11px; }
  #readout { width: 100%; text-align: center; color: #94a3b8; font-size: 11px; margin: 2px 0 6px; font-variant-numeric: tabular-nums; }
  #readout b { color: #e2e8f0; }
  /* The grid gives the stick a dedicated square.  It never shares width with controls. */
  #padwrap { position: relative; width: min(100%, 58svh, 430px); aspect-ratio: 1 / 1; align-self: center; }
  #pad { position: absolute; inset: 0; border-radius: 50%; background: radial-gradient(circle at 50% 40%, #24334a, #16202f); box-shadow: 0 10px 30px rgba(0,0,0,.5), inset 0 2px 6px rgba(255,255,255,.06); touch-action: none; }
  #knob { position: absolute; left: 50%; top: 50%; width: 44%; height: 44%; margin: -22% 0 0 -22%; border-radius: 50%; background: linear-gradient(145deg, #3b4b63, #27354a); box-shadow: 0 6px 16px rgba(0,0,0,.55); }
  .hint { position: absolute; color: #64748b; font-size: 10px; }
  .hint.top { left: 50%; top: 6%; transform: translateX(-50%); }
  .hint.bottom { left: 50%; bottom: 6%; transform: translateX(-50%); }
  .hint.left { left: 8%; top: 50%; transform: translateY(-50%); }
  .hint.right { right: 8%; top: 50%; transform: translateY(-50%); }
  .button-grid { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 7px; width: 100%; }
  button { min-width: 0; height: clamp(42px, 8.6svh, 52px); border: none; border-radius: 10px; font-size: clamp(10px, 1.55vw, 13px); line-height: 1.15; font-weight: 650; color: #fff; touch-action: none; cursor: pointer; white-space: normal; overflow-wrap: anywhere; padding: 4px 3px; }
  .span-2 { grid-column: auto; }
  .spin { background: #1d4ed8; }
  .spin.active { background: #3b82f6; box-shadow: inset 0 0 0 3px #93c5fd; }
  #estop { background: #dc2626; }
  #estop.latched { box-shadow: inset 0 0 0 3px #fca5a5; }
  #arm { background: #15803d; }
  #autoBtn { background: #1e293b; }
  #autoBtn.on { background: #0e7490; box-shadow: inset 0 0 0 3px #67e8f9; }
  #startMappingBtn { background: #7c3aed; }
  #startMappingBtn.starting { background: #a21caf; }
  #startMappingBtn.started { background: #166534; }
  #startMappingBtn.failed { background: #b91c1c; }
  #resetBtn { background: #475569; grid-column: 2; }
  #resetBtn.resetting { background: #b45309; }
  .note { width: 100%; color: #64748b; font-size: 10px; line-height: 1.45; margin: 8px 0 0; }
  #startupStatus { min-height: 15px; width: 100%; margin-top: 6px; color: #94a3b8; font-size: 10px; line-height: 1.35; text-align: center; }
  #startupStatus.starting { color: #c4b5fd; }
  #startupStatus.failed { color: #fca5a5; }
  @media (orientation: portrait) {
    body { overflow: auto; }
    #app { min-height: 100svh; height: auto; grid-template-columns: 1fr; grid-template-rows: auto auto; padding: 4px 0; }
    .drive-panel { height: auto; }
    #padwrap { width: min(70vw, 42svh); }
    .control-panel { justify-content: start; }
  }
  @media (max-height: 430px) and (orientation: landscape) {
    .note { display: none; }
    #padwrap { width: min(100%, 72svh); }
    button { height: 38px; }
  }
</style>
</head>
<body>
  <main id="app">
    <section class="drive-panel">
      <header><h1>月球车遥控</h1><span id="status">连接中…</span><button class="tab-link" id="monitorTab">监控</button></header>
      <div id="readout">LX <b id="lx">0</b> · LY <b id="ly">0</b> · RX <b id="rx">0</b> · <b id="mode">stop</b></div>
      <div id="padwrap">
        <div id="pad">
          <div id="knob"></div>
          <span class="hint top">前</span><span class="hint bottom">后</span>
          <span class="hint left">左</span><span class="hint right">右</span>
        </div>
      </div>
    </section>
    <section class="control-panel">
      <div class="button-grid">
        <button class="spin" id="spinL">⟲ 原地左转</button>
        <button class="spin" id="spinR">⟳ 原地右转</button>
        <button id="estop">⛔ 急停</button>
        <button id="arm">✅ 解除急停</button>
        <button class="span-2" id="startMappingBtn">🗺️ 开始建图导航</button>
        <button class="span-2" id="autoBtn">💻 电脑接管</button>
        <button id="relaxBtn" style="background:#b45309;">🔌 断开转矩</button>
        <button id="rearmBtn" style="background:#0e7490;">🔄 重启设零</button>
        <button class="span-2" id="calibBtn" style="background:#7c3aed;">🛠 标定四轮</button>
        <button id="resetBtn">↩️ 恢复待命</button>
      </div>
      <div class="note">摇杆上下=前进/后退；左右=阿克曼转向。原地转向请按住左右按键；松手、切后台或断连自动停。</div>
      <div id="startupStatus" aria-live="polite">待命：点击“开始建图导航”后启动。</div>
    </section>
  </main>
  <script>
    var statusEl = document.getElementById('status');
    var lxEl = document.getElementById('lx');
    var lyEl = document.getElementById('ly');
    var rxEl = document.getElementById('rx');
    var modeEl = document.getElementById('mode');
    var pad = document.getElementById('pad');
    var knob = document.getElementById('knob');
    var spinL = document.getElementById('spinL');
    var spinR = document.getElementById('spinR');
    var estopBtn = document.getElementById('estop');
    var armBtn = document.getElementById('arm');
    var autoBtn = document.getElementById('autoBtn');
    var startMappingBtn = document.getElementById('startMappingBtn');
    var startupStatus = document.getElementById('startupStatus');
    var resetBtn = document.getElementById('resetBtn');
    document.getElementById('monitorTab').addEventListener('click', function () { location.href = '/monitor' + location.search; });

    var ws = null;
    function connect() {
      ws = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws' + location.search);
      ws.onopen = function () { statusEl.textContent = '已连接'; statusEl.classList.add('on'); };
      ws.onclose = function () { statusEl.textContent = '已断开'; statusEl.classList.remove('on'); setTimeout(connect, 1500); };
      ws.onerror = function () { try { ws.close(); } catch (e) {} };
      ws.onmessage = function (ev) {
        try {
          var m = JSON.parse(ev.data);
          if (m.t === 'status') {
            lxEl.textContent = m.lx;
            lyEl.textContent = m.ly;
            rxEl.textContent = m.rx;
            modeEl.textContent = m.estop ? 'ESTOP' : m.mode;
            estopBtn.classList.toggle('latched', !!m.estop);
            autoBtn.classList.toggle('on', !!m.auto);
            var mappingState = m.mapping_state || 'waiting';
            startMappingBtn.classList.toggle('starting', mappingState === 'starting');
            startMappingBtn.classList.toggle('started', mappingState === 'started');
            startMappingBtn.disabled = mappingState === 'starting' || mappingState === 'started';
            startMappingBtn.textContent = mappingState === 'waiting' ? '🗺️ 开始建图导航' :
              (mappingState === 'starting' ? '⏳ 正在启动建图导航…' :
                (mappingState === 'started' ? '✅ 建图导航已启动' : '⚠️ 启动失败，点击重试'));
            startMappingBtn.classList.toggle('failed', mappingState === 'failed');
            startupStatus.textContent = m.mapping_detail || '待命：点击“开始建图导航”后启动。';
            startupStatus.classList.toggle('starting', mappingState === 'starting');
            startupStatus.classList.toggle('failed', mappingState === 'failed');
            var resetting = !!m.reset_state;
            resetBtn.disabled = resetting;
            resetBtn.classList.toggle('resetting', resetting);
            resetBtn.textContent = resetting ? '⏳ 正在恢复待命…' : '↩️ 恢复待命';
          }
        } catch (e) {}
      };
    }
    function send(o) { if (ws && ws.readyState === 1) { ws.send(JSON.stringify(o)); } }

    var dragging = false;
    function padCenter() {
      var r = pad.getBoundingClientRect();
      // Use the smaller side even if a browser transiently reports a non-square
      // layout during rotation, so both the pad and its travel stay circular.
      var size = Math.min(r.width, r.height);
      return { x: r.left + r.width / 2, y: r.top + r.height / 2, R: size / 2 };
    }
    function onMove(cx, cy) {
      var c = padCenter();
      var dx = cx - c.x, dy = cy - c.y;
      // 方形映射：x/y 各自独立夹到 [-1,1]，四角可同时打到 ±255
      var x = Math.max(-1, Math.min(1, dx / c.R));
      var y = Math.max(-1, Math.min(1, -(dy / c.R)));
      var rr = Math.hypot(x, y);
      var vk = rr > 1 ? 1 / rr : 1;
      knob.style.transform = 'translate(' + (x * vk * c.R) + 'px, ' + (y * vk * c.R) + 'px)';
      send({ t: 'joy', x: x, y: y });
    }
    function joyEnd() {
      dragging = false;
      knob.style.transform = 'translate(0px, 0px)';
      send({ t: 'joy', x: 0, y: 0 });
    }
    pad.addEventListener('touchstart', function (e) { e.preventDefault(); dragging = true; onMove(e.touches[0].clientX, e.touches[0].clientY); }, { passive: false });
    pad.addEventListener('touchmove', function (e) { e.preventDefault(); if (dragging) { onMove(e.touches[0].clientX, e.touches[0].clientY); } }, { passive: false });
    pad.addEventListener('touchend', joyEnd);
    pad.addEventListener('touchcancel', joyEnd);
    pad.addEventListener('mousedown', function (e) { dragging = true; onMove(e.clientX, e.clientY); });
    window.addEventListener('mousemove', function (e) { if (dragging) { onMove(e.clientX, e.clientY); } });
    window.addEventListener('mouseup', joyEnd);

    function bindHold(btn, dir) {
      var down = function (e) { e.preventDefault(); btn.classList.add('active'); send({ t: 'spin', dir: dir, on: true }); };
      var up = function () { btn.classList.remove('active'); send({ t: 'spin', dir: dir, on: false }); };
      btn.addEventListener('pointerdown', down);
      btn.addEventListener('pointerup', up);
      btn.addEventListener('pointercancel', up);
      btn.addEventListener('pointerleave', up);
      btn.addEventListener('touchstart', function (e) { e.preventDefault(); }, { passive: false });
    }
    bindHold(spinL, 'left');
    bindHold(spinR, 'right');

    setInterval(function () {
      if (dragging || spinL.classList.contains('active') || spinR.classList.contains('active')) {
        send({ t: 'heartbeat' });
      }
    }, 100);

    estopBtn.addEventListener('click', function () { send({ t: 'estop' }); });
    armBtn.addEventListener('click', function () { send({ t: 'arm' }); });
    autoBtn.addEventListener('click', function () { send({ t: 'toggle_auto' }); });
    startMappingBtn.addEventListener('click', function () { send({ t: 'start_mapping_navigation' }); });
    resetBtn.addEventListener('click', function () {
      if (window.confirm('将停止建图、Nav2、探索和 RViz，并恢复为手机遥控待命状态。继续吗？')) {
        resetBtn.disabled = true;
        resetBtn.classList.add('resetting');
        resetBtn.textContent = '⏳ 正在恢复待命…';
        startupStatus.textContent = '正在停止建图导航并恢复待命…';
        send({ t: 'reset_to_standby' });
      }
    });
    document.getElementById('relaxBtn').addEventListener('click', function () { send({ t: 'relax' }); });
    document.getElementById('rearmBtn').addEventListener('click', function () { send({ t: 'rearm' }); });
    document.getElementById('calibBtn').addEventListener('click', function () { send({ t: 'calibrate' }); });

    document.addEventListener('visibilitychange', function () {
      if (document.hidden) {
        joyEnd();
        send({ t: 'spin', dir: 'left', on: false });
        send({ t: 'spin', dir: 'right', on: false });
      }
    });

    connect();
  </script>
</body>
</html>
"""


MONITOR_PAGE = """<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover">
<title>月球车监控</title><style>
*{box-sizing:border-box;-webkit-user-select:none;user-select:none}html,body{margin:0;min-height:100%;background:#0b1220;color:#dbeafe;font-family:system-ui,-apple-system,"Segoe UI",sans-serif}body{min-height:100svh;padding:8px;overflow:hidden}.shell{height:calc(100svh - 16px);max-width:1280px;margin:auto;display:grid;grid-template-rows:auto 1fr;gap:8px}.bar{display:flex;gap:7px;align-items:center}.bar h1{font-size:15px;margin:0 auto 0 0}.tab{border:0;border-radius:8px;padding:7px 12px;background:#334155;color:#cbd5e1;font-weight:700}.tab.on{background:#0e7490;color:#ecfeff}.state{font-size:11px;color:#94a3b8}.board{display:grid;grid-template-columns:minmax(0,1.25fr) minmax(280px,.75fr);gap:8px;min-height:0}.card{min-height:0;background:#111c2e;border:1px solid #263751;border-radius:12px;padding:8px;display:grid;grid-template-rows:auto 1fr}.label{font-size:11px;color:#93c5fd;padding:0 2px 6px;display:flex;justify-content:space-between}.canvas{min-width:0;min-height:0;width:100%;height:100%;border-radius:8px;background:#07101e;touch-action:none}.stack{display:grid;grid-template-rows:1fr 1fr;gap:8px;min-height:0}.legend{font-size:10px;color:#64748b}@media(orientation:portrait){body{overflow:auto}.shell{height:auto;min-height:calc(100svh - 16px)}.board{grid-template-columns:1fr;grid-template-rows:52svh 42svh}.stack{grid-template-columns:1fr 1fr;grid-template-rows:1fr}.bar h1{font-size:14px}}
</style></head><body><main class="shell"><header class="bar"><h1>月球车监控</h1><span id="state" class="state">正在连接…</span><button id="control" class="tab">遥控</button><button class="tab on">监控</button></header><section class="board"><article class="card"><div class="label"><span>三维点云地图 · 拖动旋转</span><span id="cloudInfo" class="legend">等待地图点云</span></div><canvas id="scene" class="canvas"></canvas></article><section class="stack"><article class="card"><div class="label"><span>Nav2 通行代价图</span><span class="legend">全局 / 局部 / 地形</span></div><canvas id="map" class="canvas"></canvas></article><article class="card"><div class="label"><span>定位与规划</span><span id="poseInfo" class="legend">等待定位</span></div><canvas id="route" class="canvas"></canvas></article></section></section></main><script>
var state=document.getElementById('state'),scene=document.getElementById('scene'),mapCanvas=document.getElementById('map'),route=document.getElementById('route'),cloudInfo=document.getElementById('cloudInfo'),poseInfo=document.getElementById('poseInfo');document.getElementById('control').onclick=function(){location.href='/' + location.search};var data={pose:null,cloud:[],grids:{},plan:[],lio:[],candidates:[]},yaw=.72,pitch=.55,drag=null;
function fit(c){var r=c.getBoundingClientRect(),d=devicePixelRatio||1,w=Math.max(1,Math.floor(r.width*d)),h=Math.max(1,Math.floor(r.height*d));if(c.width!==w||c.height!==h){c.width=w;c.height=h}return [c.getContext('2d'),w,h,d]}
function xy(x,y,z,w,h,scale){var cy=Math.cos(yaw),sy=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch),a=cy*x-sy*y,b=sy*x+cy*y,depth=cp*z-sp*a;return [w*.5+(sy*x+cy*y)*scale,w*.56-(sp*z+cp*a)*scale,depth]}
function drawScene(){var t=fit(scene),g=t[0],w=t[1],h=t[2];g.clearRect(0,0,w,h);g.fillStyle='#07101e';g.fillRect(0,0,w,h);var s=Math.min(w,h)/24;g.strokeStyle='#1b3657';g.lineWidth=1;for(var i=-10;i<=10;i++){var a=xy(i,-10,0,w,h,s),b=xy(i,10,0,w,h,s),c=xy(-10,i,0,w,h,s),d=xy(10,i,0,w,h,s);g.beginPath();g.moveTo(a[0],a[1]);g.lineTo(b[0],b[1]);g.moveTo(c[0],c[1]);g.lineTo(d[0],d[1]);g.stroke()}g.fillStyle='rgba(96,220,255,.64)';for(var p=0;p<data.cloud.length;p++){var q=data.cloud[p],v=xy(q[0],q[1],q[2],w,h,s);if(v[2]>-20&&v[2]<30)g.fillRect(v[0],v[1],2,2)}if(data.pose){var x=data.pose.x,y=data.pose.y,z=data.pose.z,a=xy(x,y,z,w,h,s);g.fillStyle='#facc15';g.beginPath();g.arc(a[0],a[1],6,0,Math.PI*2);g.fill();g.strokeStyle='#facc15';var b=xy(x+Math.cos(data.pose.yaw)*.8,y+Math.sin(data.pose.yaw)*.8,z,w,h,s);g.beginPath();g.moveTo(a[0],a[1]);g.lineTo(b[0],b[1]);g.stroke()}cloudInfo.textContent=data.cloud.length?data.cloud.length+' 点（降采样）':'等待地图点云'}
function bounds(){var all=data.plan.concat(data.lio);if(data.pose)all.push([data.pose.x,data.pose.y]);for(var n in data.grids){var q=data.grids[n];if(q)all.push([q.x,q.y],[q.x+q.w*q.r,q.y+q.h*q.r])}if(!all.length)return[-5,5,-5,5];var x0=Infinity,x1=-Infinity,y0=Infinity,y1=-Infinity;all.forEach(function(p){x0=Math.min(x0,p[0]);x1=Math.max(x1,p[0]);y0=Math.min(y0,p[1]);y1=Math.max(y1,p[1])});var m=Math.max(2,(x1-x0)*.15,(y1-y0)*.15);return[x0-m,x1+m,y0-m,y1+m]}
function project(x,y,b,w,h){return[(x-b[0])/(b[1]-b[0])*w,h-(y-b[2])/(b[3]-b[2])*h]}
function gridColor(v,kind){if(v===255)return 'rgba(0,0,0,0)';if(kind==='terrain')return v>=95?'#ef4444':'hsl('+Math.max(0,130-v*1.25)+',72%,48%)';return v>=95?'#ef4444':v>0?'hsl('+(55-v*.55)+',90%,50%)':'rgba(37,99,235,.18)'}
function drawMap(){var t=fit(mapCanvas),g=t[0],w=t[1],h=t[2],b=bounds();g.fillStyle='#08121f';g.fillRect(0,0,w,h);['terrain','global','local'].forEach(function(kind){var q=data.grids[kind];if(!q)return;var cw=q.r/(b[1]-b[0])*w,ch=q.r/(b[3]-b[2])*h;for(var j=0;j<q.h;j+=2)for(var i=0;i<q.w;i+=2){var v=q.d[j*q.w+i];if(v===255)continue;var p=project(q.x+i*q.r,q.y+j*q.r,b,w,h);g.fillStyle=gridColor(v,kind);g.fillRect(p[0],p[1]-ch,Math.max(1,cw*2),Math.max(1,ch*2))}});drawLines(g,data.plan,b,w,h,'#22c55e',3);drawLines(g,data.lio,b,w,h,'#fbbf24',2);if(data.pose){var p=project(data.pose.x,data.pose.y,b,w,h);g.fillStyle='#fff';g.beginPath();g.arc(p[0],p[1],5,0,7);g.fill()} }
function drawLines(g,arr,b,w,h,color,width){if(!arr.length)return;g.strokeStyle=color;g.lineWidth=width;g.beginPath();arr.forEach(function(p,i){var q=project(p[0],p[1],b,w,h);if(i)g.lineTo(q[0],q[1]);else g.moveTo(q[0],q[1])});g.stroke()}
function drawRoute(){var t=fit(route),g=t[0],w=t[1],h=t[2],b=bounds();g.fillStyle='#08121f';g.fillRect(0,0,w,h);drawLines(g,data.plan,b,w,h,'#22c55e',3);data.candidates.forEach(function(c){drawLines(g,c,b,w,h,'rgba(217,70,239,.35)',1)});drawLines(g,data.lio,b,w,h,'#fbbf24',2);if(data.pose){var p=project(data.pose.x,data.pose.y,b,w,h);g.fillStyle='#fff';g.beginPath();g.arc(p[0],p[1],5,0,7);g.fill();poseInfo.textContent='x '+data.pose.x.toFixed(2)+' · y '+data.pose.y.toFixed(2)+' · yaw '+(data.pose.yaw*180/Math.PI).toFixed(1)+'°'}else poseInfo.textContent='等待定位'}
function draw(){drawScene();drawMap();drawRoute()}setInterval(draw,120);scene.addEventListener('pointerdown',function(e){drag=[e.clientX,e.clientY];scene.setPointerCapture(e.pointerId)});scene.addEventListener('pointermove',function(e){if(!drag)return;yaw+=(e.clientX-drag[0])*.009;pitch=Math.max(-1.3,Math.min(1.3,pitch+(e.clientY-drag[1])*.009));drag=[e.clientX,e.clientY]});scene.addEventListener('pointerup',function(){drag=null});
function unpackGrid(q){var b=atob(q.d),a=new Uint8Array(b.length);for(var i=0;i<b.length;i++)a[i]=b.charCodeAt(i);q.d=a;return q}var ws=new WebSocket((location.protocol==='https:'?'wss://':'ws://')+location.host+'/monitor/ws'+location.search);ws.onopen=function(){state.textContent='监控已连接'};ws.onclose=function(){state.textContent='监控已断开，重连中…';setTimeout(function(){location.reload()},1500)};ws.onmessage=function(e){try{var m=JSON.parse(e.data);if(m.pose)data.pose=m.pose;if(m.cloud)data.cloud=m.cloud;if(m.grids)Object.keys(m.grids).forEach(function(k){data.grids[k]=unpackGrid(m.grids[k])});if(m.plan)data.plan=m.plan;if(m.lio)data.lio=m.lio;if(m.candidates)data.candidates=m.candidates}catch(_){}};
</script></body></html>"""


def _clamp(value, low, high):
    return max(low, min(high, value))


def _point_field_offset(fields, name):
    for field in fields:
        if field.name == name:
            return field.offset, field.datatype
    return None, None


def _yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class WebJoystickServer:
    def __init__(self):
        node = Node("web_joystick_server")
        self.node = node

        self.command_topic = node.declare_parameter("command_topic", "/manual_control").value
        self.access_token = str(node.declare_parameter("access_token", "").value)
        if not self.access_token:
            node.get_logger().warn(
                "Web joystick access_token is empty; every host on this network can connect"
            )
        self.port = int(node.declare_parameter("port", 8000).value)
        self.publish_rate = float(node.declare_parameter("publish_rate", 50.0).value)
        self.deadzone = float(node.declare_parameter("deadzone", 0.05).value)
        self.deadman_timeout = float(node.declare_parameter("deadman_timeout", 0.5).value)
        self.idle_grace_period = float(node.declare_parameter("idle_grace_period", 2.0).value)
        self.navigation_start_command = str(node.declare_parameter(
            "navigation_start_command",
            "/home/phragmund/LunarRover/scripts/start_real_mapping_navigation.sh",
        ).value)
        self.standby_reset_command = str(node.declare_parameter(
            "standby_reset_command",
            "/home/phragmund/LunarRover/scripts/restart_real_nav2.sh",
        ).value)
        self.mapping_status_file = "/tmp/rover_mapping_startup_status"

        if self.publish_rate <= 0.0:
            raise ValueError("publish_rate must be positive")

        self.publisher = node.create_publisher(ManualControl, self.command_topic, 1)

        self.state = {
            "nx": 0.0,
            "ny": 0.0,
            "spin_left": False,
            "spin_right": False,
            "estop": False,
            "relax_pending": False,
            "rearm_pending": False,
            "calibrate_pending": False,
            "auto_takeover": False,
            "mapping_state": "started" if self._navigation_actions_available() else "waiting",
            "mapping_detail": "建图导航已启动。" if self._navigation_actions_available()
            else "待命：点击“开始建图导航”后启动。",
            "reset_state": False,
            "last_input": time.monotonic(),
            "last_disconnect": -1.0e9,
            "clients": 0,
            "last_published": {"lx": 0, "ly": 0, "rx": 0, "mode": "stop", "estop": False},
        }
        self.lock = threading.Lock()
        self.connections = []
        self.monitor_connections = []
        self.monitor = {
            "pose": None, "cloud": None, "grids": {}, "plan": None,
            "lio": None, "candidates": None,
            "versions": {"cloud": 0, "terrain": 0, "global": 0, "local": 0,
                         "plan": 0, "lio": 0, "candidates": 0},
            "last_cloud": 0.0, "last_grid": {},
        }
        node.create_subscription(Odometry, "/localization/lio_odometry", self._on_monitor_odom, 10)
        node.create_subscription(PointCloud2, "/localization/lio_map_cloud", self._on_monitor_cloud, 2)
        node.create_subscription(OccupancyGrid, "/terrain/traversability", self._on_monitor_terrain, 2)
        node.create_subscription(OccupancyGrid, "/global_costmap/costmap", self._on_monitor_global_costmap, 2)
        node.create_subscription(OccupancyGrid, "/local_costmap/costmap", self._on_monitor_local_costmap, 2)
        node.create_subscription(Path, "/plan", self._on_monitor_plan, 2)
        node.create_subscription(Path, "/localization/lio_path", self._on_monitor_lio_path, 2)
        node.create_subscription(MarkerArray, "/trajectories", self._on_monitor_trajectories, 2)

        self.app = FastAPI(lifespan=self._lifespan)
        self._register_routes()

    @staticmethod
    def _navigation_actions_available():
        """Use the Nav2 action as the authoritative mapping-stack readiness signal."""
        try:
            result = subprocess.run(
                ["ros2", "action", "list"],
                capture_output=True,
                text=True,
                timeout=3.0,
                check=False,
            )
        except OSError:
            return False
        return result.returncode == 0 and "/compute_path_to_pose" in result.stdout

    # ---------------- read-only mobile monitor ----------------

    def _on_monitor_odom(self, msg):
        pose = msg.pose.pose
        with self.lock:
            self.monitor["pose"] = {
                "x": round(pose.position.x, 3), "y": round(pose.position.y, 3),
                "z": round(pose.position.z, 3), "yaw": round(_yaw_from_quaternion(pose.orientation), 4),
            }

    def _on_monitor_cloud(self, msg):
        now = time.monotonic()
        with self.lock:
            if now - self.monitor["last_cloud"] < 0.9:
                return
            self.monitor["last_cloud"] = now
        if msg.is_bigendian:
            return
        xyz = [_point_field_offset(msg.fields, name) for name in ("x", "y", "z")]
        if any(offset is None or datatype != PointField.FLOAT32 for offset, datatype in xyz):
            return
        x_offset, y_offset, z_offset = (offset for offset, _ in xyz)
        total = msg.width * msg.height
        stride = max(1, math.ceil(total / 6000))
        raw = bytes(msg.data)
        cloud = []
        for index in range(0, total, stride):
            start = index * msg.point_step
            x, y, z = struct.unpack_from("<fff", raw, start + x_offset)
            # Livox fields are contiguous x/y/z, but retain the robust path if
            # a future driver changes their offsets.
            if y_offset != x_offset + 4 or z_offset != x_offset + 8:
                y = struct.unpack_from("<f", raw, start + y_offset)[0]
                z = struct.unpack_from("<f", raw, start + z_offset)[0]
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                cloud.append((round(x, 2), round(y, 2), round(z, 2)))
        with self.lock:
            self.monitor["cloud"] = cloud
            self.monitor["versions"]["cloud"] += 1

    def _store_monitor_grid(self, name, msg):
        now = time.monotonic()
        with self.lock:
            if now - self.monitor["last_grid"].get(name, 0.0) < 0.8:
                return
            self.monitor["last_grid"][name] = now
        # Quarter the cells (2x2) before base64 encoding: enough for a phone
        # overview while keeping each update comfortably below Wi-Fi MTU bursts.
        factor = 2
        width = (msg.info.width + factor - 1) // factor
        height = (msg.info.height + factor - 1) // factor
        values = bytearray()
        for row in range(0, msg.info.height, factor):
            base = row * msg.info.width
            for column in range(0, msg.info.width, factor):
                value = msg.data[base + column]
                values.append(255 if value < 0 else min(100, max(0, value)))
        grid = {
            "x": round(msg.info.origin.position.x, 3), "y": round(msg.info.origin.position.y, 3),
            "r": round(msg.info.resolution * factor, 4), "w": width, "h": height,
            "d": base64.b64encode(bytes(values)).decode("ascii"),
        }
        with self.lock:
            self.monitor["grids"][name] = grid
            self.monitor["versions"][name] += 1

    def _on_monitor_terrain(self, msg):
        self._store_monitor_grid("terrain", msg)

    def _on_monitor_global_costmap(self, msg):
        self._store_monitor_grid("global", msg)

    def _on_monitor_local_costmap(self, msg):
        self._store_monitor_grid("local", msg)

    def _store_monitor_path(self, name, msg):
        poses = msg.poses
        stride = max(1, math.ceil(len(poses) / 1200))
        path = [(round(p.pose.position.x, 3), round(p.pose.position.y, 3)) for p in poses[::stride]]
        with self.lock:
            self.monitor[name] = path
            self.monitor["versions"][name] += 1

    def _on_monitor_plan(self, msg):
        self._store_monitor_path("plan", msg)

    def _on_monitor_lio_path(self, msg):
        self._store_monitor_path("lio", msg)

    def _on_monitor_trajectories(self, msg):
        candidates = []
        for marker in msg.markers[:48]:
            if marker.type != Marker.LINE_STRIP or not marker.points:
                continue
            stride = max(1, math.ceil(len(marker.points) / 80))
            candidates.append([(round(p.x, 3), round(p.y, 3)) for p in marker.points[::stride]])
        with self.lock:
            self.monitor["candidates"] = candidates
            self.monitor["versions"]["candidates"] += 1

    def _mapping_progress(self):
        """Read the launcher-owned progress text without blocking web updates."""
        try:
            with open(self.mapping_status_file, "r", encoding="utf-8") as status_file:
                return status_file.read(256).strip()
        except OSError:
            return ""

    # ---------------- control ----------------

    def compute_sticks(self, nx, ny, spin_left, spin_right, estop, stale):
        if estop:
            return 0, 0, 0, "estop"
        if stale:
            return 0, 0, 0, "deadman"
        if spin_left:
            return 0, 0, -255, "spin left"
        if spin_right:
            return 0, 0, 255, "spin right"

        sx = nx if abs(nx) >= self.deadzone else 0.0
        sy = ny if abs(ny) >= self.deadzone else 0.0
        sx = _clamp(sx, -1.0, 1.0)
        sy = _clamp(sy, -1.0, 1.0)
        lx = int(round(sx * 255.0))
        ly = int(round(sy * 255.0))
        if lx != 0 and ly != 0:
            mode = "ackermann"
        elif ly > 0:
            mode = "forward"
        elif ly < 0:
            mode = "reverse"
        elif lx != 0:
            mode = "turn"
        else:
            mode = "stop"
        return lx, ly, 0, mode

    def publisher_loop(self):
        period = 1.0 / self.publish_rate
        last_mode = ""
        last_log = 0.0
        while rclpy.ok():
            now = time.monotonic()
            with self.lock:
                nx = self.state["nx"]
                ny = self.state["ny"]
                spin_left = self.state["spin_left"]
                spin_right = self.state["spin_right"]
                estop = self.state["estop"]
                clients = self.state["clients"]
                last_input = self.state["last_input"]
                last_disconnect = self.state["last_disconnect"]
                relax_pending = self.state["relax_pending"]
                rearm_pending = self.state["rearm_pending"]
                calibrate_pending = self.state["calibrate_pending"]
                auto_takeover = self.state["auto_takeover"]
                self.state["relax_pending"] = False
                self.state["rearm_pending"] = False
                self.state["calibrate_pending"] = False

            stale = (now - last_input) > self.deadman_timeout
            in_grace = (now - last_disconnect) < self.idle_grace_period
            lx, ly, rx, mode = self.compute_sticks(
                nx, ny, spin_left, spin_right, estop, stale
            )

            publishing = estop or clients > 0 or in_grace
            if publishing:
                message = ManualControl()
                message.lx = lx
                message.ly = ly
                message.rx = rx
                message.ry = 0
                message.relax = relax_pending
                message.rearm = rearm_pending
                message.boost = False
                message.calibrate = calibrate_pending
                message.auto_takeover = auto_takeover
                self.publisher.publish(message)

            display_mode = mode if publishing else "idle"

            with self.lock:
                self.state["last_published"] = {
                    "lx": lx, "ly": ly, "rx": rx, "mode": display_mode, "estop": estop,
                    "auto": auto_takeover,
                }

            if display_mode != last_mode or now - last_log > 1.0:
                print(
                    "mode=%s lx=%d ly=%d rx=%d clients=%d"
                    % (display_mode, lx, ly, rx, clients)
                )
                last_mode = display_mode
                last_log = now

            time.sleep(period)

    def handle_message(self, text):
        try:
            data = json.loads(text)
        except (ValueError, TypeError):
            return
        if not isinstance(data, dict):
            return
        kind = data.get("t")
        with self.lock:
            if kind == "joy":
                try:
                    x = float(data.get("x", 0.0))
                    y = float(data.get("y", 0.0))
                except (TypeError, ValueError):
                    return
                if not (math.isfinite(x) and math.isfinite(y)):
                    return
                self.state["last_input"] = time.monotonic()
                x = _clamp(x, -1.0, 1.0)
                y = _clamp(y, -1.0, 1.0)
                self.state["nx"] = x
                self.state["ny"] = y
            elif kind == "spin":
                self.state["last_input"] = time.monotonic()
                direction = data.get("dir")
                on = bool(data.get("on"))
                if direction == "left":
                    self.state["spin_left"] = on
                elif direction == "right":
                    self.state["spin_right"] = on
            elif kind == "estop":
                self.state["last_input"] = time.monotonic()
                self.state["estop"] = True
                self.state["nx"] = 0.0
                self.state["ny"] = 0.0
                self.state["spin_left"] = False
                self.state["spin_right"] = False
                self.state["auto_takeover"] = False
            elif kind == "arm":
                self.state["last_input"] = time.monotonic()
                self.state["estop"] = False
                self.state["nx"] = 0.0
                self.state["ny"] = 0.0
                self.state["spin_left"] = False
                self.state["spin_right"] = False
            elif kind == "relax":
                self.state["last_input"] = time.monotonic()
                self.state["relax_pending"] = True
            elif kind == "rearm":
                self.state["last_input"] = time.monotonic()
                self.state["rearm_pending"] = True
            elif kind == "calibrate":
                self.state["last_input"] = time.monotonic()
                self.state["calibrate_pending"] = True
            elif kind == "toggle_auto":
                self.state["last_input"] = time.monotonic()
                self.state["auto_takeover"] = not self.state["auto_takeover"]
            elif kind == "start_mapping_navigation":
                if self.state["mapping_state"] != "waiting":
                    return
                try:
                    process = subprocess.Popen(
                        ["bash", self.navigation_start_command],
                        stdin=subprocess.DEVNULL,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        start_new_session=True,
                    )
                except OSError as exc:
                    self.node.get_logger().error(
                        f"Cannot start mapping/navigation stack: {exc}"
                    )
                    return
                self.state["mapping_state"] = "starting"
                self.state["mapping_detail"] = "已收到请求：正在准备启动…"
                threading.Thread(
                    target=self._watch_mapping_start,
                    args=(process,),
                    daemon=True,
                ).start()
                self.node.get_logger().info(
                    "Phone requested mapping/navigation startup"
                )
            elif kind == "reset_to_standby":
                if self.state["reset_state"]:
                    return
                self.state["reset_state"] = True
                self.state["mapping_detail"] = "正在停止建图导航并恢复待命…"
                try:
                    subprocess.Popen(
                        ["bash", self.standby_reset_command],
                        stdin=subprocess.DEVNULL,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        start_new_session=True,
                    )
                except OSError as exc:
                    self.state["reset_state"] = False
                    self.state["mapping_detail"] = "恢复待命失败，请查看电脑端日志。"
                    self.node.get_logger().error(f"Cannot restore standby: {exc}")
                    return
                self.node.get_logger().info("Phone requested restoration to standby")
            elif kind == "heartbeat":
                self.state["last_input"] = time.monotonic()

    def _watch_mapping_start(self, process):
        """Reflect the deferred-start script result in the phone UI."""
        result = process.wait()
        with self.lock:
            self.state["mapping_state"] = "started" if result == 0 else "failed"
            detail = self._mapping_progress()
            self.state["mapping_detail"] = detail or (
                "建图导航已启动。" if result == 0 else "启动失败，请查看电脑端日志。"
            )
        if result == 0:
            self.node.get_logger().info("Mapping/navigation startup submitted successfully")
        else:
            self.node.get_logger().error(
                f"Mapping/navigation startup script exited with {result}; "
                "see /tmp/start_real_mapping_navigation.log"
            )

    # ---------------- web ----------------

    def _register_routes(self):
        app = self.app

        @app.get("/")
        async def index():
            return HTMLResponse(HTML_PAGE)

        @app.get("/monitor")
        async def monitor():
            return HTMLResponse(MONITOR_PAGE)

        @app.websocket("/ws")
        async def ws_endpoint(websocket: WebSocket):
            supplied_token = websocket.query_params.get("token", "")
            if self.access_token and not secrets.compare_digest(
                supplied_token, self.access_token
            ):
                await websocket.close(code=1008, reason="invalid access token")
                return
            await websocket.accept()
            self.connections.append(websocket)
            with self.lock:
                self.state["clients"] += 1
                self.state["last_input"] = time.monotonic()
            try:
                while True:
                    text = await websocket.receive_text()
                    self.handle_message(text)
            except WebSocketDisconnect:
                pass
            except Exception:
                pass
            finally:
                if websocket in self.connections:
                    self.connections.remove(websocket)
                with self.lock:
                    self.state["clients"] = max(0, self.state["clients"] - 1)
                    self.state["last_disconnect"] = time.monotonic()
                    self.state["nx"] = 0.0
                    self.state["ny"] = 0.0
                    self.state["spin_left"] = False
                    self.state["spin_right"] = False

        @app.websocket("/monitor/ws")
        async def monitor_ws_endpoint(websocket: WebSocket):
            supplied_token = websocket.query_params.get("token", "")
            if self.access_token and not secrets.compare_digest(
                supplied_token, self.access_token
            ):
                await websocket.close(code=1008, reason="invalid access token")
                return
            await websocket.accept()
            connection = {"websocket": websocket, "versions": {}}
            self.monitor_connections.append(connection)
            try:
                # The monitor is read-only; keep the socket open until the page
                # closes without accepting any control messages on this route.
                while True:
                    await websocket.receive_text()
            except WebSocketDisconnect:
                pass
            except Exception:
                pass
            finally:
                if connection in self.monitor_connections:
                    self.monitor_connections.remove(connection)

    @asynccontextmanager
    async def _lifespan(self, app):
        task = asyncio.create_task(self._status_broadcaster())
        monitor_task = asyncio.create_task(self._monitor_broadcaster())
        try:
            yield
        finally:
            task.cancel()
            monitor_task.cancel()

    async def _status_broadcaster(self):
        while True:
            await asyncio.sleep(0.5)
            with self.lock:
                payload = dict(self.state["last_published"])
                payload["mapping_state"] = self.state["mapping_state"]
                if self.state["mapping_state"] == "starting":
                    progress = self._mapping_progress()
                    if progress:
                        self.state["mapping_detail"] = progress
                payload["mapping_detail"] = self.state["mapping_detail"]
                payload["reset_state"] = self.state["reset_state"]
            text = json.dumps({"t": "status", **payload})
            for websocket in list(self.connections):
                try:
                    await websocket.send_text(text)
                except Exception:
                    pass

    async def _monitor_broadcaster(self):
        """Send deltas only; high-volume map/cloud snapshots are never resent."""
        while True:
            await asyncio.sleep(0.5)
            with self.lock:
                pose = self.monitor["pose"]
                versions = dict(self.monitor["versions"])
                cloud = self.monitor["cloud"]
                grids = dict(self.monitor["grids"])
                plan = self.monitor["plan"]
                lio = self.monitor["lio"]
                candidates = self.monitor["candidates"]
            for connection in list(self.monitor_connections):
                previous = connection["versions"]
                payload = {"t": "monitor", "pose": pose}
                if versions["cloud"] != previous.get("cloud") and cloud is not None:
                    payload["cloud"] = cloud
                changed_grids = {
                    name: grid for name, grid in grids.items()
                    if versions[name] != previous.get(name)
                }
                if changed_grids:
                    payload["grids"] = changed_grids
                for name, value in (("plan", plan), ("lio", lio), ("candidates", candidates)):
                    if versions[name] != previous.get(name) and value is not None:
                        payload[name] = value
                try:
                    await connection["websocket"].send_text(
                        json.dumps(payload, separators=(",", ":"))
                    )
                    connection["versions"] = versions
                except Exception:
                    if connection in self.monitor_connections:
                        self.monitor_connections.remove(connection)


def _local_ipv4_addresses():
    addresses = []
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith("127.") and ip not in addresses:
                addresses.append(ip)
    except OSError:
        pass
    return addresses


def main():
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass
    rclpy.init()
    server = WebJoystickServer()

    publisher_thread = threading.Thread(target=server.publisher_loop, daemon=True)
    publisher_thread.start()

    spin_thread = threading.Thread(target=lambda: rclpy.spin(server.node), daemon=True)
    spin_thread.start()

    print("Web joystick server 已启动，发布话题: " + server.command_topic)
    for ip in _local_ipv4_addresses():
        suffix = f"?token={server.access_token}" if server.access_token else ""
        print(f"  手机/笔记本打开: http://{ip}:{server.port}/{suffix}")

    try:
        uvicorn.run(server.app, host="0.0.0.0", port=server.port, log_level="info")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
