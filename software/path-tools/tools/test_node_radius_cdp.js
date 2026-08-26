'use strict';

const assert = require('assert');
const { spawn } = require('child_process');
const fs = require('fs');
const os = require('os');
const path = require('path');

const nodes = `0,0,2
0,325,0
0,1100,0
0,1800,0
0,2200,1
275,2200,0
550,2200,2
550,1525,0
550,850,2
1000,850,0
1600,850,0
2200,850,1
550,0,2
1000,0,2
1300,0,0
1600,0,2
1900,0,0
2200,0,2
2200,450,2
1600,450,1
1000,450,2
1000,2200,1
1000,1750,2
1600,1750,2
1300,1750,0
1600,1300,2
2200,1300,2
1900,1300,0
1900,1750,0
2200,1750,2
2200,2200,2
1480,2200,0
1870,2200,0`;

const links = `4-5
6-5
7-6
4-3
2-3
1-2
1-13
13-9
9-8
8-7
9-10
11-10
12-11
14-15
13-14
15-16
17-16
18-17
21-14
21-20
20-16
20-19
19-18
19-12
7-22
23-22
23-10
23-25
25-24
24-26
26-28
28-27
27-12
27-30
30-29
29-24
22-32
32-33
33-31
31-30`;

const chrome = 'C:\\Program Files\\Google\\Chrome\\Application\\chrome.exe';
const port = 9700 + Math.floor(Math.random() * 200);
const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'node-radius-cdp-'));
const page = `file:///${path.resolve(__dirname, 'node_coordinate_viewer.html').replace(/\\/g, '/')}`;
const browser = spawn(chrome, [
  '--headless=new', `--remote-debugging-port=${port}`, `--user-data-dir=${profile}`,
  '--disable-gpu', '--no-first-run', page
], { stdio: 'ignore' });
const delay = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

async function connect() {
  for (let attempt = 0; attempt < 60; attempt += 1) {
    try {
      const targets = await (await fetch(`http://127.0.0.1:${port}/json/list`)).json();
      const target = targets.find((item) => item.type === 'page' && item.url.startsWith('file:'));
      if (target) return new WebSocket(target.webSocketDebuggerUrl);
    } catch (_) { /* Chrome is still starting. */ }
    await delay(100);
  }
  throw new Error('Chrome DevTools did not start');
}

async function main() {
  const socket = await connect();
  if (socket.readyState !== WebSocket.OPEN) {
    await new Promise((resolve, reject) => {
      socket.addEventListener('open', resolve, { once: true });
      socket.addEventListener('error', reject, { once: true });
    });
  }
  let sequence = 0;
  const pending = new Map();
  socket.onmessage = (event) => {
    const message = JSON.parse(event.data);
    if (!message.id || !pending.has(message.id)) return;
    pending.get(message.id)(message);
    pending.delete(message.id);
  };
  const send = (method, params = {}) => new Promise((resolve) => {
    const id = ++sequence;
    pending.set(id, resolve);
    socket.send(JSON.stringify({ id, method, params }));
  });
  const evaluate = async (expression) => {
    const response = await send('Runtime.evaluate', { expression, returnByValue: true, awaitPromise: true });
    if (response.result.exceptionDetails) {
      const details = response.result.exceptionDetails;
      throw new Error(details.exception?.description || details.text);
    }
    return response.result.result.value;
  };
  await send('Runtime.enable');
  for (let attempt = 0; attempt < 50; attempt += 1) {
    const ready = await evaluate(`document.readyState === 'complete'
      && typeof els !== 'undefined'
      && typeof computeRoute === 'function'`);
    if (ready) break;
    if (attempt === 49) throw new Error('planner page did not finish initialization');
    await delay(100);
  }
  await evaluate(`(() => {
    els.bulkInput.value = ${JSON.stringify(nodes)};
    els.linkInput.value = ${JSON.stringify(links)};
    applyInput();
    els.startNodeSelect.value = '1';
    els.routeModeSelect.value = 'greedy';
    els.speedModelSelect.value = 'friction';
  })()`);

  const results = [];
  for (const radiusCm of [10, 15, 20, 25, 30, 35, 40]) {
    const result = await evaluate(`(async () => {
      els.curveRadiusInput.value = '${radiusCm}';
      await computeRoute();
      const route = state.lastRoute;
      if (!route) throw new Error('route missing: ' + els.errorBox.textContent + ' / ' + els.routeSummary.textContent);
      const radii = (route.speedPieces || []).map(piece => piece.radiusM).filter(Number.isFinite);
      const points = route.plannedPathPoints || [];
      const spacings = points.slice(1).map((point, index) => Math.hypot(
        point.x - points[index].x,
        point.y - points[index].y
      ));
      const start = nodeById(route.pathIds[0]);
      return {
        requestedRadiusCm: ${radiusCm},
        timeSeconds: route.timeSeconds,
        effectiveDistanceMm: route.effectiveDistanceMm,
        pathIds: route.pathIds,
        radiiCm: radii.map(value => value * 100),
        plannedPointCount: points.length,
        allPointsFinite: points.every(point => Number.isFinite(point.x) && Number.isFinite(point.y)),
        maxRegularSpacingErrorMm: Math.max(0, ...spacings.slice(0, -1).map(value => Math.abs(value - 5))),
        terminalSpacingMm: spacings.at(-1) || 0,
        closureErrorMm: points.length > 1 ? Math.hypot(points[0].x - points.at(-1).x, points[0].y - points.at(-1).y) : Infinity,
        firstDistanceToStartMm: points.length ? Math.hypot(points[0].x - start.x, points[0].y - start.y) : Infinity,
        minimumDistanceToStartMm: points.length ? Math.min(...points.map(point => Math.hypot(point.x - start.x, point.y - start.y))) : Infinity
      };
    })()`);
    assert(result && result.radiiCm.length, `no planned turns for ${radiusCm} cm`);
    assert(result.plannedPointCount > 100, `too few 5 mm points for ${radiusCm} cm`);
    assert(result.allPointsFinite, `non-finite planned point for ${radiusCm} cm`);
    assert(result.maxRegularSpacingErrorMm < 0.02, `planned spacing drift for ${radiusCm} cm`);
    assert(result.terminalSpacingMm > 0 && result.terminalSpacingMm <= 5.02, `invalid terminal spacing for ${radiusCm} cm`);
    assert(result.closureErrorMm < 0.02, `planned loop does not close for ${radiusCm} cm`);
    assert(Math.abs(result.firstDistanceToStartMm - result.minimumDistanceToStartMm) < 0.02, `planned path does not begin nearest the selected start for ${radiusCm} cm`);
    results.push(result);
  }
  const byRadius = new Map(results.map((result) => [result.requestedRadiusCm, result]));
  assert(Math.max(...byRadius.get(25).radiiCm) >= 24.9, '25 cm should be reached on long corridors');
  assert(Math.max(...byRadius.get(30).radiiCm) >= 29.9, '30 cm should be reached on long corridors');
  assert(Math.max(...byRadius.get(35).radiiCm) >= 34.9, '35 cm should be reached on long corridors');
  assert(Math.min(...byRadius.get(35).radiiCm) < 30, 'short corridors should still clip safely');
  assert(Math.abs(byRadius.get(25).effectiveDistanceMm - byRadius.get(35).effectiveDistanceMm) > 100, '25 cm and 35 cm plans should differ materially');
  const exactCandidates = await evaluate(`(async () => {
    els.routeModeSelect.value = 'exact';
    await computeRoute();
    return state.routeCandidates.map(route => ({
      alternatingTurnCount: route.alternatingTurnCount,
      alternatingTurnDistanceMm: route.alternatingTurnDistanceMm,
      distance: route.distance,
      timeSeconds: route.timeSeconds
    }));
  })()`);
  assert.strictEqual(exactCandidates.length, 10, '33-node map should produce ten exact candidates');
  assert(exactCandidates.every(candidate => Number.isFinite(candidate.timeSeconds)), 'exact candidates need speed-planned times');
  assert(exactCandidates.slice(1).every((candidate, index) => {
    const previous = exactCandidates[index];
    return candidate.alternatingTurnCount > previous.alternatingTurnCount
      || (candidate.alternatingTurnCount === previous.alternatingTurnCount
        && candidate.alternatingTurnDistanceMm < previous.alternatingTurnDistanceMm)
      || (candidate.alternatingTurnCount === previous.alternatingTurnCount
        && candidate.alternatingTurnDistanceMm === previous.alternatingTurnDistanceMm
        && candidate.distance >= previous.distance);
  }), '33-node exact candidates have invalid smoothness ordering');
  process.stdout.write(`${JSON.stringify(results, null, 2)}\n`);
  socket.close();
}

main().catch((error) => {
  console.error(error);
  process.exitCode = 1;
}).finally(() => browser.kill());
