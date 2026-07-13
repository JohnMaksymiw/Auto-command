// Browser-based control panel using the Web Serial API.
// Each board keeps its own SerialPort connection directly between the
// browser and the USB device — the server only serves these static files.

const boardState = {}; // id -> { connected, port, reader, writer, keepReading, logEl, statusEl, connectBtn }

const BOOT_SETTLE_MS = 2500; // time to wait after port.open() for Arduino DTR-reset boot
const sleep = ms => new Promise(r => setTimeout(r, ms));

// ---- Camera / overlay state ----
const camState = {
  open: false,
  stream: null,
  recording: false,
  recorder: null,
  chunks: [],
  csvRows: [],
  latest: { time: '--', viscosity: '--', series: '--', load: '--' },
  animFrame: null
};

// ============================================================
// Utilities
// ============================================================

function el(tag, cls, text) {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text !== undefined) e.textContent = text;
  return e;
}

function timestamp() {
  return new Date().toTimeString().slice(0, 8);
}

function fileStamp() {
  return new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
}

function downloadBlob(blob, filename) {
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 5000);
}

// ============================================================
// Serial monitor helpers
// ============================================================

function appendLog(boardId, text, kind) {
  const st = boardState[boardId];
  if (!st) return;
  const line = el('div', 'logline' + (kind ? ' ' + kind : ''));
  line.textContent = `[${timestamp()}] ${text}`;
  st.logEl.appendChild(line);
  st.logEl.scrollTop = st.logEl.scrollHeight;
  while (st.logEl.childNodes.length > 2000) {
    st.logEl.removeChild(st.logEl.firstChild);
  }
}

// Parse DATA lines from any board and feed them to the camera overlay.
// Expected format: DATA,<time_s>,<viscosity>,<series>,<load>
// Also parses SWEEP lines from the mixer board.
function checkDataLine(boardId, line) {
  if (line.startsWith('DATA,')) {
    const parts = line.split(',');
    if (parts.length !== 5) return;
    const [, t, viscosity, series, load] = parts;
    camState.latest = { time: t, viscosity, series, load };
    const statusEl = document.getElementById('cam-status');
    if (statusEl) statusEl.textContent = `t=${t}s  visc=${viscosity}  ser=${series}  load=${load}`;
    if (camState.recording) {
      camState.csvRows.push([new Date().toISOString(), t, viscosity, series, load]);
    }
    return;
  }

  if (line.startsWith('SWEEP,') && boardId === 'mixer') {
    const parts = line.split(',');
    if (parts.length !== 8) return;
    const [, n, timeSec, intercept, slope, rmse, maxStep, status] = parts;
    const sweep = {
      n: parseInt(n), timeSec: parseFloat(timeSec),
      intercept: parseFloat(intercept), slope: parseFloat(slope),
      rmse: parseFloat(rmse), maxStep: parseFloat(maxStep),
      status: status.trim()
    };
    const st = boardState[boardId];
    if (!st) return;
    st.sweepHistory = st.sweepHistory || [];
    st.sweepHistory.push(sweep);
    if (st.rheoTbody) appendSweepRow(st.rheoTbody, sweep, st.sweepHistory);
  }
}

function appendSweepRow(tbody, sweep, history) {
  const tr = document.createElement('tr');
  const statusClass = {
    OK: 'rheo-ok', TRANSITION: 'rheo-transition',
    STARTUP: 'rheo-startup', SUSPECT: 'rheo-suspect'
  }[sweep.status] || 'rheo-ok';
  tr.className = statusClass;

  // Intercept delta vs last OK sweep
  let deltaStr = '—';
  const prevOK = [...history].reverse().find(
    s => s.n !== sweep.n && s.status === 'OK'
  );
  if (prevOK && sweep.status === 'OK') {
    const d = sweep.intercept - prevOK.intercept;
    deltaStr = (d >= 0 ? '+' : '') + d.toFixed(1);
  }

  const mins = (sweep.timeSec / 60).toFixed(1);
  const cells = [
    sweep.n,
    mins + ' min',
    sweep.intercept.toFixed(1),
    deltaStr,
    sweep.slope.toFixed(3),
    sweep.rmse.toFixed(1),
    sweep.maxStep.toFixed(1),
    `<span class="sbadge ${statusClass}">${sweep.status}</span>`
  ];
  tr.innerHTML = cells.map(c => `<td>${c}</td>`).join('');
  tbody.appendChild(tr);
  tbody.parentElement.scrollTop = tbody.parentElement.scrollHeight;
}

function buildRheoPanel() {
  const panel = el('div', 'rheo-panel');

  const head = el('div', 'rheo-head');
  head.appendChild(el('span', null, 'Rheology Results'));
  const dlBtn = el('button', 'btn small', 'CSV');
  dlBtn.title = 'Download sweep results as CSV';
  dlBtn.addEventListener('click', () => downloadSweepCsv());
  const clrBtn = el('button', 'btn small', 'Clear');
  clrBtn.addEventListener('click', () => {
    const st = boardState['mixer'];
    if (st) { st.sweepHistory = []; if (st.rheoTbody) st.rheoTbody.innerHTML = ''; }
  });
  const btnRow = el('div', 'rheo-btn-row');
  btnRow.append(dlBtn, clrBtn);
  head.appendChild(btnRow);
  panel.appendChild(head);

  const wrap = el('div', 'rheo-table-wrap');
  const table = document.createElement('table');
  table.className = 'rheo-table';
  const thead = document.createElement('thead');
  thead.innerHTML = '<tr><th>M</th><th>Time</th><th>Intercept</th><th>Δic</th><th>Slope</th><th>RMSE</th><th>MaxStep</th><th>Status</th></tr>';
  const tbody = document.createElement('tbody');
  table.append(thead, tbody);
  wrap.appendChild(table);
  panel.appendChild(wrap);

  return { panel, tbody };
}

function downloadSweepCsv() {
  const st = boardState['mixer'];
  if (!st || !st.sweepHistory || !st.sweepHistory.length) return;
  const header = 'M,time_min,intercept,slope,rmse,max_step,status';
  const rows = st.sweepHistory.map(s =>
    [s.n, (s.timeSec/60).toFixed(2), s.intercept.toFixed(2),
     s.slope.toFixed(4), s.rmse.toFixed(2), s.maxStep.toFixed(2), s.status].join(',')
  );
  const csv = [header, ...rows].join('\n');
  downloadBlob(new Blob([csv], { type: 'text/csv' }), `sweeps_${fileStamp()}.csv`);
}

function setConnected(boardId, connected) {
  const st = boardState[boardId];
  if (!st) return;
  st.connected = connected;
  st.statusEl.textContent = connected ? 'Connected' : 'Disconnected';
  st.statusEl.className = 'status ' + (connected ? 'on' : 'off');
  st.connectBtn.textContent = connected ? 'Disconnect' : 'Connect';
  st.connectBtn.className = 'btn ' + (connected ? 'danger' : 'primary');
  updateDot(boardId, connected);
}

// ============================================================
// Serial connection
// ============================================================

async function connect(board) {
  const st = boardState[board.id];
  let port;
  try {
    port = await navigator.serial.requestPort();
  } catch (e) {
    appendLog(board.id, 'Port selection cancelled', 'sys');
    return;
  }
  try {
    await port.open({ baudRate: board.baud });
  } catch (e) {
    appendLog(board.id, 'Connect failed: ' + e.message, 'err');
    return;
  }

  st.port = port;
  st.keepReading = true;
  st.writer = port.writable.getWriter();
  appendLog(board.id, `Connected @ ${board.baud} baud — waiting for board to boot…`, 'sys');
  readLoop(board.id);
  await sleep(BOOT_SETTLE_MS);
  setConnected(board.id, true);
  appendLog(board.id, 'Board ready.', 'sys');
}

async function readLoop(boardId) {
  const st = boardState[boardId];
  const decoder = new TextDecoder();
  let buffer = '';
  try {
    while (st.keepReading && st.port && st.port.readable) {
      const reader = st.port.readable.getReader();
      st.reader = reader;
      try {
        while (true) {
          const { value, done } = await reader.read();
          if (done) break;
          buffer += decoder.decode(value, { stream: true });
          let idx;
          while ((idx = buffer.indexOf('\n')) >= 0) {
            const lineText = buffer.slice(0, idx).replace(/\r$/, '');
            buffer = buffer.slice(idx + 1);
            if (lineText.length) {
              appendLog(boardId, lineText, 'rx');
              checkDataLine(boardId, lineText);
            }
          }
        }
      } catch (e) {
        if (st.keepReading) appendLog(boardId, 'Read error: ' + e.message, 'err');
      } finally {
        try { reader.releaseLock(); } catch (_) {}
      }
    }
  } catch (e) {
    appendLog(boardId, 'Serial error: ' + e.message, 'err');
  }
}

async function disconnect(boardId) {
  const st = boardState[boardId];
  st.keepReading = false;
  try { if (st.reader) await st.reader.cancel(); } catch (_) {}
  try { if (st.writer) st.writer.releaseLock(); } catch (_) {}
  try { if (st.port) await st.port.close(); } catch (_) {}
  st.port = null;
  st.reader = null;
  st.writer = null;
  setConnected(boardId, false);
  appendLog(boardId, 'Disconnected', 'sys');
}

async function toggleConnection(board) {
  const st = boardState[board.id];
  if (st.connected) await disconnect(board.id);
  else await connect(board);
}

async function sendCommand(boardId, cmd) {
  const st = boardState[boardId];
  if (!st.connected || !st.writer) {
    appendLog(boardId, 'Not connected — cannot send "' + cmd + '"', 'err');
    return;
  }
  try {
    await st.writer.write(new TextEncoder().encode(cmd + '\n'));
    appendLog(boardId, '> ' + cmd, 'tx');
    // Auto-open camera when dispensing starts on Linear
    if (boardId === 'linear' && cmd === 'start') openCamera();
  } catch (e) {
    appendLog(boardId, 'Send failed: ' + e.message, 'err');
  }
}

// ============================================================
// Camera overlay
// ============================================================

async function openCamera() {
  if (camState.open) {
    document.getElementById('camera-overlay').classList.remove('hidden');
    return;
  }
  try {
    const stream = await navigator.mediaDevices.getUserMedia({ video: true, audio: false });
    camState.stream = stream;

    const video = document.getElementById('cam-video');
    video.srcObject = stream;
    await video.play();

    // Wait for actual dimensions
    await new Promise(res => {
      if (video.videoWidth) { res(); return; }
      video.addEventListener('loadedmetadata', res, { once: true });
    });

    const canvas = document.getElementById('cam-canvas');
    canvas.width = video.videoWidth || 640;
    canvas.height = video.videoHeight || 480;
    camState.canvas = canvas;
    camState.ctx = canvas.getContext('2d');
    camState.video = video;
    camState.open = true;

    document.getElementById('camera-overlay').classList.remove('hidden');
    drawFrame();
  } catch (e) {
    alert('Could not open camera: ' + e.message);
  }
}

function closeCamera() {
  if (camState.recording) stopRecording();
  document.getElementById('camera-overlay').classList.add('hidden');
  if (camState.animFrame) { cancelAnimationFrame(camState.animFrame); camState.animFrame = null; }
  if (camState.stream) { camState.stream.getTracks().forEach(t => t.stop()); camState.stream = null; }
  camState.open = false;
}

function drawFrame() {
  if (!camState.open || !camState.ctx) return;
  const ctx = camState.ctx;
  const canvas = camState.canvas;
  const video = camState.video;

  ctx.drawImage(video, 0, 0, canvas.width, canvas.height);

  const lines = [
    `Time:       ${camState.latest.time} s`,
    `Viscosity:  ${camState.latest.viscosity}`,
    `Series:     ${camState.latest.series}`,
    `Load:       ${camState.latest.load}`
  ];

  ctx.font = 'bold 22px "SF Mono", Menlo, monospace';
  lines.forEach((text, i) => {
    const x = 20, y = 42 + i * 38;
    ctx.fillStyle = 'rgba(0,0,0,0.55)';
    ctx.fillText(text, x + 2, y + 2);
    ctx.fillStyle = '#ffffff';
    ctx.fillText(text, x, y);
  });

  // REC indicator
  if (camState.recording) {
    ctx.fillStyle = 'rgba(0,0,0,0.5)';
    ctx.fillRect(canvas.width - 80, 10, 70, 28);
    ctx.fillStyle = '#ef4444';
    ctx.beginPath();
    ctx.arc(canvas.width - 65, 24, 7, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = '#fff';
    ctx.font = 'bold 14px sans-serif';
    ctx.fillText('REC', canvas.width - 55, 29);
  }

  camState.animFrame = requestAnimationFrame(drawFrame);
}

function startRecording() {
  if (camState.recording || !camState.canvas) return;
  const canvasStream = camState.canvas.captureStream(30);
  const mimeType = MediaRecorder.isTypeSupported('video/webm;codecs=vp9')
    ? 'video/webm;codecs=vp9'
    : 'video/webm';
  camState.recorder = new MediaRecorder(canvasStream, { mimeType });
  camState.chunks = [];
  camState.csvRows = [['pc_time', 'test_time_s', 'viscosity', 'series', 'load']];

  camState.recorder.ondataavailable = e => {
    if (e.data.size > 0) camState.chunks.push(e.data);
  };

  camState.recorder.onstop = () => {
    const stamp = fileStamp();
    const blob = new Blob(camState.chunks, { type: 'video/webm' });
    downloadBlob(blob, `test_${stamp}.webm`);
    const csv = camState.csvRows.map(r => r.join(',')).join('\n');
    downloadBlob(new Blob([csv], { type: 'text/csv' }), `test_${stamp}.csv`);
    camState.chunks = [];
    camState.csvRows = [];
    updateRecordButtons(false);
  };

  camState.recorder.start(100);
  camState.recording = true;
  updateRecordButtons(true);
}

function stopRecording() {
  if (!camState.recording || !camState.recorder) return;
  camState.recorder.stop();
  camState.recording = false;
}

function updateRecordButtons(recording) {
  const rec = document.getElementById('cam-record-btn');
  const stop = document.getElementById('cam-stop-btn');
  if (!rec || !stop) return;
  rec.disabled = recording;
  stop.disabled = !recording;
}

// ============================================================
// UI builder
// ============================================================

function buildBoardPage(board) {
  const page = el('section', 'board-page');
  page.id = 'page-' + board.id;
  page.style.display = 'none';

  const bar = el('div', 'connbar');
  const connectBtn = el('button', 'btn primary', 'Connect');
  const status = el('span', 'status off', 'Disconnected');
  bar.append(connectBtn, status);
  page.appendChild(bar);

  const isMixer = board.id === 'mixer';
  const body = el('div', 'board-body' + (isMixer ? ' mixer-body' : ''));

  const cmdPanel = el('div', 'cmd-panel');
  board.groups.forEach(group => {
    const g = el('div', 'cmd-group');
    g.appendChild(el('h3', 'group-title', group.title));
    const grid = el('div', 'btn-grid');
    group.commands.forEach(c => {
      let cls = 'btn cmd';
      if (c.danger) cls += ' danger';
      else if (c.accent) cls += ' accent';
      const b = el('button', cls, c.label);
      b.title = c.cmd;
      b.addEventListener('click', () => sendCommand(board.id, c.cmd));
      grid.appendChild(b);
    });
    g.appendChild(grid);
    cmdPanel.appendChild(g);
  });

  const serialPanel = el('div', 'serial-panel');
  const serialHead = el('div', 'serial-head');
  serialHead.appendChild(el('span', null, 'Serial Monitor'));
  const clearBtn = el('button', 'btn small', 'Clear');
  serialHead.appendChild(clearBtn);
  const log = el('div', 'serial-log');

  const inputRow = el('div', 'input-row');
  const manual = el('input', 'manual-input');
  manual.type = 'text';
  manual.placeholder = 'Type a command and press Enter…';
  const sendBtn = el('button', 'btn primary', 'Send');
  inputRow.append(manual, sendBtn);

  serialPanel.append(serialHead, log, inputRow);

  let rheoTbody = null;
  if (isMixer) {
    const { panel: rheoPanel, tbody } = buildRheoPanel();
    rheoTbody = tbody;
    body.append(cmdPanel, rheoPanel, serialPanel);
  } else {
    body.append(cmdPanel, serialPanel);
  }
  page.appendChild(body);

  boardState[board.id] = {
    connected: false,
    port: null,
    reader: null,
    writer: null,
    keepReading: false,
    logEl: log,
    statusEl: status,
    connectBtn,
    sweepHistory: [],
    rheoTbody
  };

  connectBtn.addEventListener('click', () => toggleConnection(board));
  clearBtn.addEventListener('click', () => { log.innerHTML = ''; });

  const doManualSend = () => {
    const v = manual.value.trim();
    if (v) { sendCommand(board.id, v); manual.value = ''; }
  };
  sendBtn.addEventListener('click', doManualSend);
  manual.addEventListener('keydown', e => { if (e.key === 'Enter') doManualSend(); });

  return page;
}

function showBoard(id) {
  document.querySelectorAll('.board-page').forEach(p => {
    p.style.display = (p.id === 'page-' + id) ? 'flex' : 'none';
  });
  document.querySelectorAll('.tab').forEach(t => {
    t.classList.toggle('active', t.dataset.board === id);
  });
}

// ============================================================
// Home tab — connect all boards at once
// ============================================================

// Hardcoded port hints shown to the user during sequential picking
const PORT_HINTS = {
  mixer:   'usbmodem13301',
  bowl:    'usbmodem11101',
  linear2: 'usbmodem132201',
  linear:  'usbmodem13401'
};

let homeStatusEl = null;

function buildHomePage() {
  const page = el('section', 'board-page home-page');
  page.id = 'page-home';
  page.style.display = 'none';

  const inner = el('div', 'home-inner');

  const logoImg = el('img', 'home-logo');
  logoImg.src = 'logo-nobg.png';
  logoImg.alt = 'Auto Command';

  const portList = el('ul', 'port-list');
  [
    ['Mixer',  PORT_HINTS.mixer],
    ['Bowl',   PORT_HINTS.bowl],
    ['Linear', PORT_HINTS.linear]
  ].forEach(([name, hint]) => {
    const li = el('li', 'port-item');
    li.innerHTML = `<span class="port-name">${name}</span><span class="port-path">${hint}</span>`;
    portList.appendChild(li);
  });

  const connectBtn = el('button', 'btn primary home-connect-btn', 'Connect All');
  const disconnectBtn = el('button', 'btn danger home-connect-btn', 'Disconnect All');
  homeStatusEl = el('div', 'home-status', '');

  const btnRow = el('div', 'home-btn-row');
  btnRow.append(connectBtn, disconnectBtn);

  // Per-board status dots
  const dotRow = el('div', 'dot-row');
  window.BOARDS.forEach(board => {
    const dot = el('div', 'dot-item');
    dot.id = `dot-${board.id}`;
    dot.innerHTML = `<span class="dot off"></span><span class="dot-label">${board.name}</span>`;
    dotRow.appendChild(dot);
  });

  connectBtn.addEventListener('click', () => connectAll(connectBtn));
  disconnectBtn.addEventListener('click', () => disconnectAll());

  inner.append(logoImg, portList, btnRow, homeStatusEl, dotRow);
  page.appendChild(inner);
  return page;
}

function updateDot(boardId, connected) {
  const dot = document.querySelector(`#dot-${boardId} .dot`);
  if (dot) {
    dot.className = 'dot ' + (connected ? 'on' : 'off');
  }
}


async function disconnectAll() {
  for (const board of window.BOARDS) {
    if (boardState[board.id] && boardState[board.id].connected) {
      await disconnect(board.id);
    }
  }
  homeStatusEl.textContent = 'All boards disconnected';
}

async function connectAll(btn) {
  btn.disabled = true;
  for (const board of window.BOARDS) {
    const st = boardState[board.id];
    if (st && st.connected) {
      homeStatusEl.textContent = `${board.name} already connected — skipping`;
      continue;
    }

    homeStatusEl.textContent = `Select the ${board.name} port (${PORT_HINTS[board.id]})…`;

    let port;
    try {
      port = await navigator.serial.requestPort();
    } catch (e) {
      homeStatusEl.textContent = `${board.name} selection cancelled`;
      continue;
    }

    try {
      await port.open({ baudRate: board.baud });
    } catch (e) {
      homeStatusEl.textContent = `${board.name} connect failed: ${e.message}`;
      continue;
    }

    const st2 = boardState[board.id];
    st2.port = port;
    st2.keepReading = true;
    st2.writer = port.writable.getWriter();
    appendLog(board.id, `Connected @ ${board.baud} baud — waiting for board to boot…`, 'sys');
    readLoop(board.id);
    homeStatusEl.textContent = `${board.name} — waiting for board to boot…`;
    await sleep(BOOT_SETTLE_MS);
    setConnected(board.id, true);
    appendLog(board.id, 'Board ready.', 'sys');
    homeStatusEl.textContent = `${board.name} connected ✓`;

    // Persist port identity so auto-connect can find it next time
    savePortInfo(board.id, port);
  }

  const allConnected = window.BOARDS.every(b => boardState[b.id] && boardState[b.id].connected);
  homeStatusEl.textContent = allConnected ? 'All boards connected' : 'Done — check individual tabs for any skipped boards';
  btn.disabled = false;
}

// Persist enough info about a port to re-identify it via getPorts() later
async function savePortInfo(boardId, port) {
  try {
    const allPorts = await navigator.serial.getPorts();
    const idx = allPorts.indexOf(port);
    const info = port.getInfo();
    const saved = JSON.parse(localStorage.getItem('boardPortInfo') || '{}');
    saved[boardId] = { usbVendorId: info.usbVendorId, usbProductId: info.usbProductId, index: idx };
    localStorage.setItem('boardPortInfo', JSON.stringify(saved));
  } catch (_) {}
}

// On page load: silently connect to previously-granted ports without any dialog
async function tryAutoConnect() {
  const ports = await navigator.serial.getPorts();
  if (!ports.length) return false;

  const saved = JSON.parse(localStorage.getItem('boardPortInfo') || '{}');
  const claimed = new Set();
  let anyConnected = false;

  for (const board of window.BOARDS) {
    const st = boardState[board.id];
    if (!st || st.connected) continue;

    const info = saved[board.id];
    let port = null;

    // Use saved index first — most reliable when all boards share the same VID/PID
    if (info && info.index !== undefined && info.index < ports.length) {
      const candidate = ports[info.index];
      if (!claimed.has(candidate)) port = candidate;
    }

    // Fall back to VID/PID match only if no saved index (works when boards have distinct chip types)
    if (!port && info && info.usbVendorId !== undefined) {
      port = ports.find(p => {
        if (claimed.has(p)) return false;
        const pi = p.getInfo();
        return pi.usbVendorId === info.usbVendorId && pi.usbProductId === info.usbProductId;
      });
    }

    // Last resort: next unclaimed port in order
    if (!port) port = ports.find(p => !claimed.has(p));
    if (!port) continue;

    claimed.add(port);

    try {
      await port.open({ baudRate: board.baud });
      st.port = port;
      st.keepReading = true;
      st.writer = port.writable.getWriter();
      appendLog(board.id, `Auto-connected @ ${board.baud} baud — waiting for board to boot…`, 'sys');
      readLoop(board.id);
      await sleep(BOOT_SETTLE_MS);
      setConnected(board.id, true);
      appendLog(board.id, 'Board ready.', 'sys');
      anyConnected = true;
    } catch (_) {
      // Port may already be open or unavailable — ignore silently
    }
  }

  return anyConnected;
}

// ============================================================
// Init
// ============================================================

function init() {
  if (!('serial' in navigator)) {
    document.getElementById('unsupported').style.display = 'block';
    document.getElementById('boards').style.display = 'none';
    return;
  }

  const tabs = document.getElementById('tabs');
  const boardsContainer = document.getElementById('boards');

  // Home tab — first
  const homeTab = el('button', 'tab active', 'Home');
  homeTab.dataset.board = 'home';
  homeTab.addEventListener('click', () => showBoard('home'));
  tabs.appendChild(homeTab);
  boardsContainer.appendChild(buildHomePage());

  // Board tabs
  window.BOARDS.forEach(board => {
    const tab = el('button', 'tab', board.name);
    tab.dataset.board = board.id;
    tab.addEventListener('click', () => showBoard(board.id));
    tabs.appendChild(tab);
    boardsContainer.appendChild(buildBoardPage(board));
  });

  showBoard('home');

  // Auto-connect on load if ports were previously granted
  homeStatusEl.textContent = 'Connecting…';
  tryAutoConnect().then(anyConnected => {
    const allConnected = window.BOARDS.every(b => boardState[b.id] && boardState[b.id].connected);
    if (allConnected) {
      homeStatusEl.textContent = 'All boards auto-connected';
    } else if (anyConnected) {
      homeStatusEl.textContent = 'Some boards auto-connected — click Connect to add remaining';
    } else {
      homeStatusEl.textContent = '';
    }
  });

  // Camera button in topbar
  document.getElementById('cam-toggle-btn').addEventListener('click', openCamera);
  document.getElementById('cam-close-btn').addEventListener('click', closeCamera);
  document.getElementById('cam-record-btn').addEventListener('click', startRecording);
  document.getElementById('cam-stop-btn').addEventListener('click', stopRecording);
}

window.addEventListener('DOMContentLoaded', init);
