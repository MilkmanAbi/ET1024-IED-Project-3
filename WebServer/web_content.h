/*
 * web_content.h - Web Dashboard HTML/CSS/JS stored in PROGMEM
 *
 * Dark-themed, mobile-first responsive UI for Urban Farming Robot.
 * Single page: sensor dashboard + D-pad controller.
 * WebSocket connection with auto-reconnect.
 */

#ifndef WEB_CONTENT_H
#define WEB_CONTENT_H

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>UrbanFarmBot</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
:root{
  --bg:#1a1a2e;--bg2:#16213e;--card:#0f3460;
  --green:#2ecc71;--orange:#e67e22;--red:#e74c3c;
  --text:#ecf0f1;--dim:#7f8c8d;--accent:#3498db;
}
body{
  font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;
  background:var(--bg);color:var(--text);
  min-height:100vh;overflow-x:hidden;
}

/* Top Bar */
.topbar{
  background:var(--bg2);padding:12px 16px;
  display:flex;align-items:center;justify-content:space-between;
  border-bottom:1px solid #ffffff10;
}
.topbar h1{font-size:18px;font-weight:600}
.status{display:flex;align-items:center;gap:6px;font-size:13px}
.dot{width:10px;height:10px;border-radius:50%;background:var(--red);transition:background .3s}
.dot.on{background:var(--green)}

/* Layout */
.container{
  display:grid;grid-template-columns:1fr 1fr;gap:16px;
  padding:16px;max-width:900px;margin:0 auto;
}

/* Cards */
.card{
  background:var(--card);border-radius:12px;padding:16px;
}
.card h2{font-size:14px;color:var(--dim);margin-bottom:10px;text-transform:uppercase;letter-spacing:1px}

/* Mode Section */
.mode-section{margin-bottom:12px}
.mode-status{
  display:flex;align-items:center;gap:8px;margin-bottom:10px;
  font-size:15px;
}
.mode-controls{display:flex;gap:8px}
.mode-btn{
  flex:1;padding:12px 8px;border:none;border-radius:10px;
  font-size:14px;font-weight:600;cursor:pointer;color:#fff;
  transition:all .15s;
}
.mode-btn:active{transform:scale(0.95)}
.mode-btn:disabled{opacity:0.3;cursor:not-allowed}
.start-btn{background:var(--green)}
.stop-btn{background:var(--red)}

/* Sensor Grid */
.sensors{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.sensor{
  background:#ffffff08;border-radius:8px;padding:12px;text-align:center;
}
.sensor .icon{font-size:24px;margin-bottom:4px}
.sensor .val{font-size:22px;font-weight:700}
.sensor .lbl{font-size:11px;color:var(--dim);margin-top:2px}
.sensor.warn{border:1px solid var(--orange)}
.sensor.danger{border:1px solid var(--red)}

/* Badges */
.badge{
  padding:3px 10px;border-radius:12px;font-size:12px;font-weight:600;
  text-transform:uppercase;
}
.badge.man{background:#3498db33;color:var(--accent)}
.badge.search{background:#e67e2233;color:var(--orange)}
.badge.lf{background:#2ecc7133;color:var(--green)}
.badge.up{background:#2ecc7133;color:var(--green)}
.badge.down{background:#e67e2233;color:var(--orange)}
.badge.moving{background:#3498db33;color:var(--accent)}

/* Actuator Badge */
.actuator-status{
  display:flex;align-items:center;justify-content:center;
  gap:8px;margin-top:10px;padding:8px;
  background:#ffffff08;border-radius:8px;font-size:14px;
}

/* D-Pad */
.dpad{
  display:grid;
  grid-template-areas:". up ." "left stop right" ". down .";
  grid-template-columns:repeat(3,70px);
  grid-template-rows:repeat(3,70px);
  gap:6px;justify-content:center;margin:10px 0;
}
.dpad button{
  border:none;border-radius:50%;font-size:22px;font-weight:700;
  cursor:pointer;color:#fff;transition:all .15s;
  display:flex;align-items:center;justify-content:center;
  -webkit-user-select:none;user-select:none;
  touch-action:none;
}
.dpad button:active:not(:disabled){transform:scale(0.92)}
.btn-up{grid-area:up;background:var(--accent)}
.btn-down{grid-area:down;background:var(--accent)}
.btn-left{grid-area:left;background:var(--accent)}
.btn-right{grid-area:right;background:var(--accent)}
.btn-stop{grid-area:stop;background:var(--red);font-size:14px}
.dpad button:disabled{opacity:0.3;cursor:not-allowed}

/* Actuator Controls */
.act-controls{
  display:flex;gap:10px;justify-content:center;margin-top:14px;
}
.act-btn{
  flex:1;padding:14px;border:none;border-radius:10px;
  font-size:14px;font-weight:600;cursor:pointer;color:#fff;
  transition:all .15s;max-width:160px;
}
.act-btn:active:not(:disabled){transform:scale(0.95)}
.act-btn:disabled{opacity:0.3;cursor:not-allowed}
.act-down{background:var(--orange)}
.act-up{background:var(--green)}

/* Obstacle Warning */
.obstacle-warn{
  display:none;padding:10px;margin-top:10px;
  background:#e74c3c33;border:1px solid var(--red);border-radius:8px;
  text-align:center;font-weight:600;color:var(--red);
  animation:pulse 1s infinite;
}
.obstacle-warn.show{display:block}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:0.6}}

/* Mobile layout */
@media(max-width:640px){
  .container{grid-template-columns:1fr;gap:12px;padding:12px}
  .dpad{
    grid-template-columns:repeat(3,76px);
    grid-template-rows:repeat(3,76px);
  }
}
</style>
</head>
<body>

<div class="topbar">
  <h1>UrbanFarmBot</h1>
  <div class="status">
    <span id="connText">Disconnected</span>
    <span class="dot" id="connDot"></span>
  </div>
</div>

<div class="container">
  <!-- Left Panel: Dashboard -->
  <div class="card">
    <h2>Dashboard</h2>

    <div class="mode-section">
      <div class="mode-status">
        Mode: <span class="badge man" id="modeBadge">MANUAL</span>
      </div>
      <div class="mode-controls">
        <button class="mode-btn start-btn" id="btnStart">Start Line Follow</button>
        <button class="mode-btn stop-btn" id="btnStop" disabled>Stop</button>
      </div>
    </div>

    <div class="sensors">
      <div class="sensor" id="sTemp">
        <div class="icon">&#x1F321;</div>
        <div class="val" id="valTemp">--</div>
        <div class="lbl">Temp &deg;C</div>
      </div>
      <div class="sensor" id="sHum">
        <div class="icon">&#x1F4A7;</div>
        <div class="val" id="valHum">--</div>
        <div class="lbl">Humidity %</div>
      </div>
      <div class="sensor" id="sMoist">
        <div class="icon">&#x1F331;</div>
        <div class="val" id="valMoist">--</div>
        <div class="lbl">Moisture</div>
      </div>
      <div class="sensor" id="sDist">
        <div class="icon">&#x1F4CF;</div>
        <div class="val" id="valDist">--</div>
        <div class="lbl">Distance cm</div>
      </div>
    </div>

    <div class="actuator-status">
      Actuator: <span class="badge up" id="actBadge">UP</span>
    </div>

    <div class="obstacle-warn" id="obstacleWarn">
      &#x26A0; OBSTACLE DETECTED
    </div>
  </div>

  <!-- Right Panel: Controller -->
  <div class="card">
    <h2>Controller</h2>

    <div class="dpad" id="dpad">
      <button class="btn-up" data-cmd="F">&#x25B2;</button>
      <button class="btn-left" data-cmd="L">&#x25C0;</button>
      <button class="btn-stop" data-cmd="S">STOP</button>
      <button class="btn-right" data-cmd="R">&#x25B6;</button>
      <button class="btn-down" data-cmd="B">&#x25BC;</button>
    </div>

    <div class="act-controls">
      <button class="act-btn act-down" id="btnActDown">Push Down</button>
      <button class="act-btn act-up" id="btnActUp" disabled>Pull Up</button>
    </div>
  </div>
</div>

<script>
// State
var ws = null;
var currentMode = 'MAN';  // MAN, SEARCH, LF
var actState = 'UP';
var reconnTimer = null;

// WebSocket
function connect() {
  var host = location.hostname || '192.168.4.1';
  ws = new WebSocket('ws://' + host + '/ws');

  ws.onopen = function() {
    document.getElementById('connDot').classList.add('on');
    document.getElementById('connText').textContent = 'Connected';
    if (reconnTimer) { clearTimeout(reconnTimer); reconnTimer = null; }
  };

  ws.onclose = function() {
    document.getElementById('connDot').classList.remove('on');
    document.getElementById('connText').textContent = 'Reconnecting...';
    reconnTimer = setTimeout(connect, 2000);
  };

  ws.onerror = function() { ws.close(); };

  ws.onmessage = function(evt) {
    handleMessage(evt.data);
  };
}

function sendCmd(cmd) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(cmd);
  }
}

// Parse messages from Arduino
function handleMessage(msg) {
  msg = msg.trim();

  // Telemetry: T:25.5,H:60.2,M:1,D:15  (M: 1=moist 0=dry)
  if (msg.indexOf('T:') === 0) {
    var parts = msg.split(',');
    for (var i = 0; i < parts.length; i++) {
      var kv = parts[i].split(':');
      if (kv.length === 2) {
        var k = kv[0], v = kv[1];
        if (k === 'T') document.getElementById('valTemp').textContent = v;
        else if (k === 'H') document.getElementById('valHum').textContent = v;
        else if (k === 'M') {
          var wet = (v === '1');
          document.getElementById('valMoist').textContent = wet ? 'MOIST' : 'DRY';
          var sm = document.getElementById('sMoist');
          if (wet) { sm.classList.add('warn'); } else { sm.classList.remove('warn'); }
        }
        else if (k === 'D') {
          document.getElementById('valDist').textContent = v;
          updateObstacle(parseFloat(v));
        }
      }
    }
    return;
  }

  // Actuator state
  if (msg.indexOf('A:') === 0) {
    actState = msg.substring(2);
    updateActuator(actState);
    return;
  }

  // Mode
  if (msg.indexOf('MODE:') === 0) {
    currentMode = msg.substring(5);
    updateModeUI();
    return;
  }

  // Obstacle stop
  if (msg === 'K:OBSTACLE') {
    updateObstacle(0);
  }
}

function updateObstacle(dist) {
  var el = document.getElementById('obstacleWarn');
  var sd = document.getElementById('sDist');
  if (dist < 20) {
    el.classList.add('show');
    sd.classList.add('danger');
    sd.classList.remove('warn');
  } else if (dist < 40) {
    el.classList.remove('show');
    sd.classList.add('warn');
    sd.classList.remove('danger');
  } else {
    el.classList.remove('show');
    sd.classList.remove('warn');
    sd.classList.remove('danger');
  }
}

function updateActuator(state) {
  var badge = document.getElementById('actBadge');
  badge.textContent = state;
  badge.className = 'badge ' + state.toLowerCase();

  document.getElementById('btnActDown').disabled = (state !== 'UP');
  document.getElementById('btnActUp').disabled = (state !== 'DOWN');
}

function updateModeUI() {
  var badge = document.getElementById('modeBadge');
  var btnStart = document.getElementById('btnStart');
  var btnStop = document.getElementById('btnStop');
  var isManual = (currentMode === 'MAN');

  if (currentMode === 'MAN') {
    badge.textContent = 'MANUAL';
    badge.className = 'badge man';
    btnStart.disabled = false;
    btnStop.disabled = true;
  } else if (currentMode === 'SEARCH') {
    badge.textContent = 'SEARCHING';
    badge.className = 'badge search';
    btnStart.disabled = true;
    btnStop.disabled = false;
  } else {
    badge.textContent = 'LINE FOLLOW';
    badge.className = 'badge lf';
    btnStart.disabled = true;
    btnStop.disabled = false;
  }

  // D-pad only works in manual mode
  var btns = document.querySelectorAll('.dpad button');
  for (var i = 0; i < btns.length; i++) {
    btns[i].disabled = !isManual;
  }
}

// Start / Stop buttons
document.getElementById('btnStart').addEventListener('click', function() {
  sendCmd('G');
});
document.getElementById('btnStop').addEventListener('click', function() {
  sendCmd('X');
});

// Actuator buttons
document.getElementById('btnActDown').addEventListener('click', function() {
  sendCmd('D:1');
});
document.getElementById('btnActUp').addEventListener('click', function() {
  sendCmd('D:0');
});

// D-Pad: hold-to-move
(function() {
  var btns = document.querySelectorAll('.dpad button[data-cmd]');
  for (var i = 0; i < btns.length; i++) {
    (function(btn) {
      var cmd = btn.getAttribute('data-cmd');

      function start(e) {
        e.preventDefault();
        if (btn.disabled) return;
        sendCmd(cmd);
      }
      function stop(e) {
        e.preventDefault();
        if (btn.disabled) return;
        if (cmd !== 'S') sendCmd('S');
      }

      btn.addEventListener('mousedown', start);
      btn.addEventListener('mouseup', stop);
      btn.addEventListener('mouseleave', stop);
      btn.addEventListener('touchstart', start, {passive:false});
      btn.addEventListener('touchend', stop, {passive:false});
      btn.addEventListener('touchcancel', stop, {passive:false});
    })(btns[i]);
  }
})();

// Init
connect();
updateModeUI();
</script>
</body>
</html>
)rawliteral";

#endif // WEB_CONTENT_H
