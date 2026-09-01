/*
 * GRS GroundStation dashboard client.
 *
 * Expected incoming WebSocket message (one JSON object per UAV update):
 *
* UAV ("type": "uav" or omitted):
 * {
 *   "id": "UAV-01",
 *   "connected": true,
 *   "armed": true,
 *   "mode": "OFFBOARD",
 *   "airspeed": 15.32,
 *   "groundspeed": 12.32,
 *   "altitude": 125.4,
 *   "roll": 8,
 *   "pitch": 6,
 *   "rpm": 6400,
 *   "cl": 1.12,
 *   "battery": 95,
 *   "gpsHdop": 0.8,
 *   "gpsFix": "3D Fix (10)",
 *   "satellites": 18,
 *   "rcSignal": 92,
 *   "linkQuality": "Excellent",
 *   "health": {"imu": "ok", "baro": "ok", "compass": "ok", "gps": "ok", "battery": "warn", "rc": "ok"}
 * }
 *
 * Payload ("type": "payload") -- fewer fields, no mode/armed/attitude:
 * {
 *   "type": "payload",
 *   "id": "PAYLOAD",
 *   "connected": true,
 *   "altitude": 118.2,
 *   "battery": 91,
 *   "gpsHdop": 0.9,
 *   "mass": 2.35,        // estimator output, omit if not available yet
 *   "wind": 3.1,          // estimator output, omit if not available yet
 *   "gpsFix": "3D Fix (9)",
 *   "satellites": 15,
 *   "health": { "imu": "ok", "gps": "ok", "battery": "ok" }
 * }
 *
 * Launcher ("type": "launcher") -- one per catapult, decoupled from any UAV:
 * {
 *   "type": "launcher",
 *   "id": "1",
 *   "state": "Armed",
 *   "connected": "yes",
 *   "cocked": "yes",
 *   "armed": "yes",
 *   "countdown": "no",
 *   "lowBattery": "no",
 *   "safetyPinIn": "no",
 *   "gcsTimeout": "no"
 * }
 *
 * NMPC controller debug panel ("type": "nmpc") -- one for the whole
 * controller, not per-UAV:
 * {
 *   "type": "nmpc",
 *   "launched": true,
 *   "inFlight": true,
 *   "endedTraj": false,
 *   "violation": false,
 *   "lastSolveMs": 3.42,
 *   "trackingNumber": 128,
 *   "trajectoryIndex": 340,
 *   "trajectoryTotal": 1000
 * }
 *
 * Adapt WebSocketServer.cpp / DashboardServer.cpp to emit this shape,
 * or edit `applyUpdate()` below to match whatever schema you send instead.
 */

const WS_URL = "ws://localhost:8080/ws";

const uavGrid = document.getElementById("uav-grid");
const payloadContainer = document.getElementById("payload-panel");
const launcherGrid = document.getElementById("launcher-grid");
const nmpcContainer = document.getElementById("nmpc-panel");

const templateUav = document.getElementById("uav-panel-template");
const payloadTemplate = document.getElementById("payload-panel-template");
const launcherTemplate = document.getElementById("launcher-panel-template");
const nmpcTemplate = document.getElementById("nmpc-panel-template");
const uavCountEl = document.getElementById("uav-count");
const clockEl = document.getElementById("clock");

const uavPanels = new Map();
const launcherPanels = new Map();
let payloadPanel = null;
let nmpcPanel = null;

function getOrCreatePanel(id) {
    if (uavPanels.has(id)) return uavPanels.get(id);

    const node = templateUav.content.firstElementChild.cloneNode(true);
    node.querySelector(".uav-name").textContent = id;
    uavGrid.appendChild(node);
    uavPanels.set(id, node);
    return node;
}

function setStat(panel, field, value, digits = null, suffix = "") {
    const card = panel.querySelector(`.stat-card[data-field="${field}"] .stat-value`);
    if (!card) return;

    if (value == null) {
        card.textContent = "--";
        return;
    }

    const formatted =
        (digits !== null && typeof value === "number")
            ? value.toFixed(digits)
            : value;

    card.textContent = `${formatted}${suffix}`;
}

function setInfo(panel, field, value) {
    const el = panel.querySelector(`[data-field="${field}"]`);
    if (el) el.textContent = value;
}

function setHealth(panel, field, status) {
    const el = panel.querySelector(`[data-field="health-${field}"]`);
    if (!el) return;
    el.textContent = status === "ok" ? "OK" : status === "warn" ? "Check" : "Fail";
    el.className = status === "ok" ? "health-ok" : status === "warn" ? "health-warn" : "health-fail";
}

function setStatus(panel, field, status, prefix = "launcher-") {
    const el = panel.querySelector(`[data-field="${prefix}${field}"]`);
    if (!el) return;
    el.textContent = status === "yes" ? "YES" : "NO";
    el.className = status === "yes" ? "status-yes" : "status-no";
}

function applyUpdate(data) {
    const panel = getOrCreatePanel(data.id);

    panel.querySelector(".status-dot").classList.toggle("on", !!data.connected);

    const connBadge = panel.querySelector(".badge-conn");
    connBadge.textContent = data.connected ? "CONNECTED" : "OFFLINE";
    connBadge.classList.toggle("on", !!data.connected);

    const armBadge = panel.querySelector(".badge-arm");
    armBadge.textContent = data.armed ? "ARMED" : "DISARMED";
    armBadge.classList.toggle("on", !!data.armed);

    setStat(panel, "airspeed", data.airspeed, 1);
    setStat(panel, "groundspeed", data.groundspeed, 1);
    setStat(panel, "altitude", data.altitude, 1);
    setStat(panel, "roll", data.roll, 1);
    setStat(panel, "pitch", data.pitch, 1);
    setStat(panel, "rpm", data.rpm, 0);
    setStat(panel, "cl", data.cl, 2);
    setStat(panel, "battery", data.battery, 0, "%");
    setStat(panel, "gpsHdop", data.gpsHdop, 1);

    setInfo(panel, "mode", data.mode ?? "--");
    setInfo(panel, "gpsFix", data.gpsFix ?? "--");
    setInfo(panel, "satellites", data.satellites ?? "--");
    setInfo(panel, "rcSignal", data.rcSignal !== undefined ? `${data.rcSignal}%` : "--");
    setInfo(panel, "linkQuality", data.linkQuality ?? "--");
    setInfo(panel, "lastUpdate", "just now");

    if (data.health) {
        for (const [key, status] of Object.entries(data.health)) {
            setHealth(panel, key, status);
        }
    }

    updateTopbar();
}

function updateTopbar() {
    const connected = [...uavPanels.values()].filter(p =>
        p.querySelector(".status-dot").classList.contains("on")).length;
    uavCountEl.textContent = `${connected} UAV${connected === 1 ? "" : "s"} Connected`;
}

function tickClock() {
    clockEl.textContent = new Date().toLocaleTimeString("en-GB", { hour12: false });
}
setInterval(tickClock, 500);
tickClock();

function createPayloadPanel() {

    if (payloadPanel !== null) return payloadPanel;

    const node = payloadTemplate.content.firstElementChild.cloneNode(true);

    payloadContainer.appendChild(node);
    payloadPanel = node;

    return payloadPanel;
}

function updatePayload(data) {
    const panel = createPayloadPanel();

    panel.querySelector(".status-dot").classList.toggle("on", !!data.connected);

    const connBadge = panel.querySelector(".badge-conn");
    connBadge.textContent = data.connected ? "CONNECTED" : "OFFLINE";
    connBadge.classList.toggle("on", !!data.connected);

    const armBadge = panel.querySelector(".badge-arm");
    armBadge.textContent = data.armed ? "ARMED" : "DISARMED";
    armBadge.classList.toggle("on", !!data.armed);

    setStat(panel, "groundspeed", data.groundspeed, 1);
    setStat(panel, "altitude", data.altitude, 1);
    setStat(panel, "battery", data.battery, 0, "%");
    setStat(panel, "gpsHdop", data.gpsHdop, 1);

    setInfo(panel, "mode", data.mode ?? "--");
    setInfo(panel, "gpsFix", data.gpsFix ?? "--");
    setInfo(panel, "satellites", data.satellites ?? "--");
    setInfo(panel, "linkQuality", data.linkQuality ?? "--");
    setInfo(panel, "lastUpdate", "just now");

    if (data.health) {
        for (const [key, status] of Object.entries(data.health)) {
            setHealth(panel, key, status);
        }
    }
}

/* ---------- Launcher panels (one per catapult, own section) ---------- */

function getOrCreateLauncherPanel(id) {
    if (launcherPanels.has(id)) return launcherPanels.get(id);

    const node = launcherTemplate.content.firstElementChild.cloneNode(true);
    node.querySelector(".uav-name").textContent = `Launcher ${id}`;
    launcherGrid.appendChild(node);
    launcherPanels.set(id, node);
    return node;
}

function updateLauncher(data) {
    const panel = getOrCreateLauncherPanel(data.id);

    const connected = data.connected === "yes";
    const armed = data.armed === "yes";

    panel.querySelector(".status-dot").classList.toggle("on", connected);

    const connBadge = panel.querySelector(".badge-conn");
    connBadge.textContent = connected ? "CONNECTED" : "OFFLINE";
    connBadge.classList.toggle("on", connected);

    const armBadge = panel.querySelector(".badge-arm");
    armBadge.textContent = armed ? "ARMED" : "DISARMED";
    armBadge.classList.toggle("on", armed);

    setInfo(panel, "state", data.state ?? "--");
    setStat(panel, "battery", data.battery, 0, "%");
    setStatus(panel, "cocked", data.cocked);
    setStatus(panel, "countdown", data.countdown);
    setStatus(panel, "safetyPinIn", data.safetyPinIn);
    setStatus(panel, "lowBattery", data.lowBattery);
    setStatus(panel, "gcsTimeout", data.gcsTimeout);
}

/* ---------- NMPC controller debug panel (one for the whole controller) ---------- */

function createNmpcPanel() {
    if (nmpcPanel !== null) return nmpcPanel;

    const node = nmpcTemplate.content.firstElementChild.cloneNode(true);
    nmpcContainer.appendChild(node);
    nmpcPanel = node;
    return nmpcPanel;
}

function updateNmpc(data) {
    const panel = createNmpcPanel();

    panel.querySelector(".status-dot").classList.toggle("on", !!data.launched);

    const launchedBadge = panel.querySelector('[data-field="nmpc-launched-badge"]');
    launchedBadge.textContent = data.launched ? "LAUNCHED" : "PRE-LAUNCH";
    launchedBadge.classList.toggle("on", !!data.launched);

    const violationBadge = panel.querySelector('[data-field="nmpc-violation-badge"]');
    violationBadge.textContent = data.violation ? "VIOLATION" : "OK";
    violationBadge.classList.toggle("on", !!data.violation);

    setStat(panel, "lastSolveMs", data.lastSolveMs, 2);
    setStat(panel, "trackingNumber", data.trackingNumber, 0);

    setInfo(panel, "nmpc-launched", data.launched ? "Yes" : "No");
    setInfo(panel, "nmpc-inFlight", data.inFlight ? "Yes" : "No");
    setInfo(panel, "nmpc-trajectory", `${data.trajectoryIndex ?? "--"} / ${data.trajectoryTotal ?? "--"}`);
    setInfo(panel, "nmpc-endedTraj", data.endedTraj ? "Yes" : "No");
    setInfo(panel, "nmpc-violation", data.violation ? "Yes" : "No");
}

/* ---------- WebSocket connection to DashboardServer / WebSocketServer ---------- */

let socket = null;
let demoMode = false;
let demoIntervalId = null;

// Distinct from "socket connected": an idle GCS with no UAVs linked yet has
// a perfectly healthy, open WebSocket that just never sends anything. Demo
// mode should stay up until the first real message actually arrives, not
// merely until the socket opens.
let hasReceivedRealData = false;

function connect() {
    socket = new WebSocket(WS_URL);

    socket.addEventListener("message", (event) => {
        try {
            const data = JSON.parse(event.data);

            if (!hasReceivedRealData) {
                hasReceivedRealData = true;
                stopDemo();
            }

            switch (data.type) {
                case "payload":  updatePayload(data);  break;
                case "launcher": updateLauncher(data); break;
                case "nmpc":     updateNmpc(data);      break;
                default:         applyUpdate(data);
            }
        } catch (err) {
            console.error("Bad message from GCS backend:", err);
        }
    });

    socket.addEventListener("close", () => {
        setTimeout(connect, 2000); // retry
        if (!hasReceivedRealData) maybeStartDemo();
    });

    socket.addEventListener("error", () => {
        socket.close();
    });
}

/* ---------- Demo/offline fallback so the page is viewable standalone ---------- */

function maybeStartDemo() {
    if (demoMode || hasReceivedRealData) return;
    demoMode = true;
    runDemo();
}

function stopDemo() {
    demoMode = false;
    if (demoIntervalId !== null) {
        clearInterval(demoIntervalId);
        demoIntervalId = null;
    }

    // Wipe demo-created panels so stale fake data can't linger next to (or
    // instead of) real ones once telemetry starts flowing.
    uavGrid.innerHTML = "";
    uavPanels.clear();
    launcherGrid.innerHTML = "";
    launcherPanels.clear();
    if (payloadPanel) { payloadPanel.remove(); payloadPanel = null; }
    if (nmpcPanel) { nmpcPanel.remove(); nmpcPanel = null; }
    updateTopbar();
}

function runDemo() {
    const ids = ["UAV-01", "UAV-02"];
    const state = {
        "UAV-01": { airspeed: 15.3, groundspeed: 18.2, altitude: 125, roll: 6, pitch: 4, cl: 0.82, battery: 75 },
        "UAV-02": { airspeed: 18.8, groundspeed: 22.2, altitude: 125, roll: 5, pitch: 2, cl: 0.79, battery: 68 },
    };
    const nmpcState = { lastSolveMs: 3.1, trackingNumber: 0, trajectoryIndex: 0, trajectoryTotal: 1000 };

    demoIntervalId = setInterval(() => {
        if (!demoMode) return;

        updateLauncher({
            id: "1", state: "Armed", connected: "yes", armed: "yes",
            cocked: "yes", countdown: "no", safetyPinIn: "no", lowBattery: "no", gcsTimeout: "no",battery: 88,
        });
        updateLauncher({
            id: "2", state: "Connected", connected: "yes", armed: "no",
            cocked: "no", countdown: "no", safetyPinIn: "yes", lowBattery: "no", gcsTimeout: "no",battery: 91,
        });

        nmpcState.trackingNumber += 1;
        nmpcState.trajectoryIndex = Math.min(nmpcState.trajectoryTotal, nmpcState.trajectoryIndex + 1);
        nmpcState.lastSolveMs = 2.5 + Math.random() * 2;
        updateNmpc({
            launched: true,
            inFlight: true,
            endedTraj: nmpcState.trajectoryIndex >= nmpcState.trajectoryTotal,
            violation: false,
            lastSolveMs: nmpcState.lastSolveMs,
            trackingNumber: nmpcState.trackingNumber,
            trajectoryIndex: nmpcState.trajectoryIndex,
            trajectoryTotal: nmpcState.trajectoryTotal,
        });

        updatePayload({
            connected: true,
            armed: false,
            groundspeed: 3.1,
            altitude: 25.2,
            battery: 82,
            gpsHdop: 0.8 + Math.random() * 0.5,
            mode: "AUTO",
            gpsFix: "3D Fix (10)",
            satellites: 18,
            rcSignal: 90,
            linkQuality: "Excellent",
            health: { imu: "ok", baro: "warn", compass: "ok", gps: "ok", battery: "ok", rc: "ok" },
        })

        ids.forEach((id) => {
            const s = state[id];
            s.airspeed = Math.max(0, s.airspeed + Math.random());
            s.groundspeed = Math.max(0, s.groundspeed + (Math.random() - 0.5));
            s.altitude += (Math.random() - 0.5) * 2;
            s.roll = (s.roll + (Math.random() - 0.5) * 3);
            s.pitch = (s.pitch + (Math.random() - 0.5) * 3);
            s.cl = (s.cl + (Math.random() * 0.1));
            s.battery = Math.max(0, s.battery - 0.01);

            applyUpdate({
                id,
                connected: true,
                armed: true,
                mode: id === "UAV-01" ? "OFFBOARD" : "AUTO",
                airspeed: s.airspeed,
                groundspeed: s.groundspeed,
                altitude: s.altitude,
                roll: s.roll,
                pitch: s.pitch,
                rpm: 6500 + Math.random() * 50,
                cl: s.cl,
                battery: s.battery,
                gpsHdop: 0.8 + Math.random() * 0.5,
                gpsFix: "3D Fix (10)",
                satellites: 18,
                linkQuality: "Excellent",
                health: { imu: "ok", baro: "warn", compass: "ok", gps: "ok", battery: "ok", rc: "ok" },
            });


        });
    }, 500);
}

connect();
// If the socket never connects, or it connects but the GCS is idle (no
// UAVs linked yet so nothing is ever sent), fall back to demo data so the
// page isn't just black. Cancels itself the moment real telemetry arrives.
setTimeout(() => {
    if (!hasReceivedRealData) maybeStartDemo();
}, 2000);