"use strict";

const FACE_ORDER = ["U", "R", "F", "D", "L", "B"];
const stateEl = document.getElementById("state");
const detectionEl = document.getElementById("detection");
const faceGridEl = document.getElementById("face-grid");
const tokensEl = document.getElementById("tokens");
const currentTokenEl = document.getElementById("current-token");
const faultLogEl = document.getElementById("fault-log");
const manualResultEl = document.getElementById("manual-result");
const ctrlStatusEl = document.getElementById("ctrl-status");

let solutionTokens = [];

function stateClass(state) {
  if (!state) return "state-idle";
  const s = state.toUpperCase();
  if (s.startsWith("DETECT")) return "state-detect";
  if (s.startsWith("PICKUP")) return "state-pickup";
  if (s.startsWith("ROTATE")) return "state-perceive";
  if (s.startsWith("PERCEIVE")) return "state-perceive";
  if (s.startsWith("SOLVE")) return "state-solve";
  if (s.startsWith("PLACE")) return "state-place";
  if (s.startsWith("EXECUTE")) return "state-execute";
  if (s.startsWith("GO_HOME")) return "state-home";
  if (s === "DONE") return "state-done";
  if (s === "FAULT") return "state-fault";
  return "state-idle";
}

function renderState(state) {
  stateEl.textContent = state || "IDLE";
  stateEl.className = "badge " + stateClass(state);
}

function renderDetection(d) {
  if (!d) { detectionEl.textContent = "대기 중…"; return; }
  detectionEl.innerHTML =
    `frame: ${d.frame_id || "?"}<br>` +
    `pos:  x=${d.x.toFixed(3)}  y=${d.y.toFixed(3)}  z=${d.z.toFixed(3)}<br>` +
    `quat: ${d.qx.toFixed(2)}, ${d.qy.toFixed(2)}, ${d.qz.toFixed(2)}, ${d.qw.toFixed(2)}`;
}

function renderFaces(faceColors) {
  faceGridEl.innerHTML = "";
  for (const face of FACE_ORDER) {
    const tile = document.createElement("div");
    tile.className = "face-tile";
    const h4 = document.createElement("h4");
    h4.textContent = face;
    tile.appendChild(h4);
    const cells = document.createElement("div");
    cells.className = "face-cells";
    const colors = faceColors[face] || "?????????";
    for (let i = 0; i < 9; i++) {
      const c = colors[i] || "?";
      const cell = document.createElement("div");
      cell.className = "cell " + (c.match(/[WYROBG]/) ? c : "Q");
      cell.textContent = c;
      cells.appendChild(cell);
    }
    tile.appendChild(cells);
    faceGridEl.appendChild(tile);
  }
}

function renderTokens(tokens, currentLabel) {
  solutionTokens = tokens || [];
  tokensEl.innerHTML = "";
  const activeIdx = parseCurrentTokenIndex(currentLabel);
  solutionTokens.forEach((t, i) => {
    const span = document.createElement("span");
    span.className = "tok";
    span.textContent = t;
    if (i === activeIdx) span.classList.add("active");
    else if (activeIdx >= 0 && i < activeIdx) span.classList.add("done");
    tokensEl.appendChild(span);
  });
}

function parseCurrentTokenIndex(label) {
  if (!label) return -1;
  const m = label.match(/^(\d+)\/(\d+):/);
  if (!m) return -1;
  return parseInt(m[1], 10) - 1;
}

function renderCurrentToken(label) {
  currentTokenEl.textContent = label || "-";
  if (solutionTokens.length) renderTokens(solutionTokens, label);
}

function pushFault(entry) {
  const li = document.createElement("li");
  const time = document.createElement("time");
  const ts = entry.ts ? new Date(entry.ts * 1000) : new Date();
  time.textContent = ts.toLocaleTimeString();
  li.appendChild(time);
  li.appendChild(document.createTextNode(" " + (entry.reason || "")));
  faultLogEl.insertBefore(li, faultLogEl.firstChild);
}

function applySnapshot(snap) {
  renderState(snap.state);
  renderDetection(snap.last_detection);
  renderFaces(snap.face_colors || {});
  renderTokens(snap.solution || [], snap.current_token);
  renderCurrentToken(snap.current_token);
  faultLogEl.innerHTML = "";
  (snap.fault_history || []).slice().reverse().forEach(pushFault);
}

function applyEvent(msg) {
  switch (msg.type) {
    case "snapshot": applySnapshot(msg.data); break;
    case "state": renderState(msg.data); break;
    case "solution": renderTokens(msg.data, currentTokenEl.textContent); break;
    case "current_token": renderCurrentToken(msg.data); break;
    case "face_colors": {
      const cur = {};
      FACE_ORDER.forEach((f, fi) => {
        const tile = faceGridEl.children[fi];
        if (tile) {
          const cells = tile.querySelectorAll(".cell");
          cur[f] = Array.from(cells).map((c) => c.textContent || "?").join("");
        }
      });
      cur[msg.face] = msg.colors;
      renderFaces(cur);
      break;
    }
    case "last_detection": renderDetection(msg.data); break;
    case "fault": pushFault(msg.data); break;
  }
}

function setCtrlStatus(text, isError) {
  ctrlStatusEl.textContent = text;
  ctrlStatusEl.style.color = isError ? "#e74c3c" : "#27ae60";
}

async function fetchSnapshot() {
  try {
    const res = await fetch("/api/snapshot");
    if (res.ok) applySnapshot(await res.json());
  } catch (e) {
    console.warn("snapshot fetch failed", e);
  }
}

function connectWS() {
  const wsUrl = (location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/ws/events";
  const ws = new WebSocket(wsUrl);
  ws.onmessage = (ev) => {
    try { applyEvent(JSON.parse(ev.data)); } catch (e) { console.warn("ws parse", e); }
  };
  ws.onclose = () => setTimeout(connectWS, 1500);
  ws.onerror = () => ws.close();
  setInterval(() => { if (ws.readyState === WebSocket.OPEN) ws.send("ping"); }, 15000);
}

// ---- button handlers ----

document.getElementById("btn-scan").addEventListener("click", async () => {
  setCtrlStatus("스캔 중… (완료까지 대기)", false);
  try {
    const res = await fetch("/api/start_scan", { method: "POST" });
    const data = await res.json();
    if (data.success) {
      setCtrlStatus(`스캔 완료 ✓  solution: ${data.solution || "(없음)"}`, false);
      // solution 입력란에 자동 채워줌 (사용자가 확인 후 Solve 가능)
      const solInput = document.getElementById("manual-solution");
      if (solInput && data.solution) solInput.value = data.solution;
    } else {
      setCtrlStatus("스캔 실패: " + (data.message || ""), true);
    }
  } catch (e) {
    setCtrlStatus("스캔 오류: " + e, true);
  }
});

document.getElementById("btn-solve").addEventListener("click", async () => {
  setCtrlStatus("풀이 실행 중… (완료까지 대기)", false);
  try {
    const res = await fetch("/api/start_solve", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ solution: "" }),  // 빈 값 = 캐시 사용
    });
    const data = await res.json();
    setCtrlStatus(
      data.success ? "풀이 완료 ✓" : "풀이 실패: " + (data.message || ""),
      !data.success,
    );
  } catch (e) {
    setCtrlStatus("풀이 오류: " + e, true);
  }
});

document.getElementById("btn-run").addEventListener("click", async () => {
  try {
    const res = await fetch("/api/start_run", { method: "POST" });
    const data = await res.json();
    setCtrlStatus("start_run: " + (data.message || ""), !data.success);
  } catch (e) {
    setCtrlStatus("오류: " + e, true);
  }
});

document.getElementById("btn-cancel").addEventListener("click", async () => {
  try {
    const res = await fetch("/api/cancel", { method: "POST" });
    const data = await res.json();
    setCtrlStatus("cancel: " + (data.message || ""), !data.success);
  } catch (e) {
    setCtrlStatus("오류: " + e, true);
  }
});

document.getElementById("btn-estop").addEventListener("click", async () => {
  if (!confirm("⚠️ 비상정지를 실행합니다. 진행 중인 모든 모션이 즉시 정지됩니다. 계속할까요?")) {
    return;
  }
  try {
    setCtrlStatus("🛑 비상정지 요청 중…", true);
    const res = await fetch("/api/emergency_stop", { method: "POST" });
    const data = await res.json();
    setCtrlStatus("emergency_stop: " + (data.message || ""), !data.success);
  } catch (e) {
    setCtrlStatus("오류: " + e, true);
  }
});

// Solve with manual solution string
document.getElementById("manual-solve-form").addEventListener("submit", async (e) => {
  e.preventDefault();
  const solution = document.getElementById("manual-solution").value.trim();
  setCtrlStatus("풀이 실행 중…", false);
  try {
    const res = await fetch("/api/start_solve", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ solution }),
    });
    const data = await res.json();
    setCtrlStatus(
      data.success ? "풀이 완료 ✓" : "풀이 실패: " + (data.message || ""),
      !data.success,
    );
  } catch (e) {
    setCtrlStatus("오류: " + e, true);
  }
});

// Manual single token
document.getElementById("manual-form").addEventListener("submit", async (e) => {
  e.preventDefault();
  const tok = document.getElementById("manual-token").value.trim();
  if (!tok) return;
  manualResultEl.textContent = "보내는 중…";
  try {
    const res = await fetch("/api/token", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ token: tok }),
    });
    const data = await res.json();
    manualResultEl.textContent = `${data.success ? "✓" : "✗"} ${data.message || ""}`;
  } catch (e) {
    manualResultEl.textContent = "오류: " + e;
  }
});

fetchSnapshot();
connectWS();
