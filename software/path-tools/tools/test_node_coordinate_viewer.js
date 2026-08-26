'use strict';

const assert = require('assert');
const { spawn } = require('child_process');
const fs = require('fs');
const os = require('os');
const path = require('path');

const browserExecutable = 'C:\\Program Files\\Google\\Chrome\\Application\\chrome.exe';
const port = 9800 + Math.floor(Math.random() * 100);
const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'node-planner-cdp-'));
const pageUrl = `file:///${path.resolve(__dirname, 'node_coordinate_viewer.html').replace(/\\/g, '/')}`;
const browser = spawn(browserExecutable, [
  '--headless=new', `--remote-debugging-port=${port}`, `--user-data-dir=${profile}`,
  '--disable-gpu', '--no-first-run', '--window-size=1440,900', pageUrl
], { stdio: 'ignore' });

const delay = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

async function targetInfo() {
  for (let attempt = 0; attempt < 50; attempt += 1) {
    try {
      const targets = await (await fetch(`http://127.0.0.1:${port}/json/list`)).json();
      const page = targets.find((target) => target.type === 'page' && target.url.startsWith('file:'));
      if (page) return page;
    } catch (_) {
      // Browser startup race.
    }
    await delay(100);
  }
  throw new Error('Chrome DevTools endpoint did not start');
}

async function main() {
  const target = await targetInfo();
  const ws = new WebSocket(target.webSocketDebuggerUrl);
  let nextId = 1;
  const pending = new Map();
  ws.onmessage = (event) => {
    const message = JSON.parse(event.data);
    if (!message.id || !pending.has(message.id)) return;
    const request = pending.get(message.id);
    pending.delete(message.id);
    if (message.error) request.reject(new Error(JSON.stringify(message.error)));
    else request.resolve(message.result);
  };
  await new Promise((resolve, reject) => { ws.onopen = resolve; ws.onerror = reject; });
  const send = (method, params = {}) => new Promise((resolve, reject) => {
    const id = nextId++;
    pending.set(id, { resolve, reject });
    ws.send(JSON.stringify({ id, method, params }));
  });
  const evaluate = async (expression) => {
    const result = await send('Runtime.evaluate', { expression, returnByValue: true, awaitPromise: true });
    if (result.exceptionDetails) throw new Error(JSON.stringify(result.exceptionDetails));
    return result.result.value;
  };

  await send('Runtime.enable');
  await send('Page.enable');
  for (let attempt = 0; attempt < 50; attempt += 1) {
    if (await evaluate('document.readyState === "complete" && Boolean(document.querySelector("#sampleBtn"))')) break;
    if (attempt === 49) throw new Error(`planner page did not load: ${await evaluate('location.href')}`);
    await delay(100);
  }
  await evaluate(`(() => {
    document.querySelector('#sampleBtn').click();
    const mode = document.querySelector('#routeModeSelect');
    mode.value = 'fastest';
    mode.dispatchEvent(new Event('change', { bubbles: true }));
    document.querySelector('#routeBtn').click();
  })()`);
  for (let attempt = 0; attempt < 200; attempt += 1) {
    const ready = await evaluate(`document.querySelector('#routeBtn').disabled === false &&
      JSON.parse(document.querySelector('#exportText').value).computed_shortest_route !== null`);
    if (ready) break;
    if (attempt === 199) throw new Error('route computation timed out');
    await delay(50);
  }
  const stopNodeId = await evaluate(`(() => {
    const laps = document.querySelector('#lapCountInput');
    laps.value = '3';
    laps.dispatchEvent(new Event('input', { bubbles: true }));
    const stop = document.querySelector('#stopNodeSelect');
    stop.selectedIndex = Math.min(1, stop.options.length - 1);
    stop.dispatchEvent(new Event('change', { bubbles: true }));
    const stopLap = document.querySelector('#stopLapInput');
    stopLap.value = '2';
    stopLap.dispatchEvent(new Event('input', { bubbles: true }));
    return Number(stop.value);
  })()`);
  const payload = await evaluate(`JSON.parse(document.querySelector('#exportText').value)`);
  const route = payload.computed_shortest_route;
  const coordinateEditAudit = await evaluate(`(() => {
    const originalPath = state.lastRoute.pathIds.slice();
    const originalCandidateCount = state.routeCandidates.length;
    const movedNode = state.nodes.find((node) => node.id !== originalPath[0]) || state.nodes[0];
    const originalX = movedNode.x;
    const input = document.querySelector('#bulkInput');
    input.value = state.nodes.map((node) => [
      node.name,
      node.x + (node.id === movedNode.id ? 10 : 0),
      node.y,
      node.type
    ].join(',')).join('\n');
    input.dispatchEvent(new Event('input', { bubbles: true }));
    return {
      pathPreserved: JSON.stringify(state.lastRoute.pathIds) === JSON.stringify(originalPath),
      candidatesPreserved: state.routeCandidates.length === originalCandidateCount,
      geometryRebuilt: state.nodes.find((node) => node.id === movedNode.id).x === originalX + 10,
      plannedPointsAvailable: state.lastRoute.plannedPathPoints.length > 10
    };
  })()`);
  assert.strictEqual(coordinateEditAudit.pathPreserved, true,
    'editing only a node coordinate must preserve the current route sequence');
  assert.strictEqual(coordinateEditAudit.candidatesPreserved, true,
    'editing only a node coordinate must preserve route candidates');
  assert.strictEqual(coordinateEditAudit.geometryRebuilt, true,
    'editing only a node coordinate must update the node geometry');
  assert.strictEqual(coordinateEditAudit.plannedPointsAvailable, true,
    'editing only a node coordinate must rebuild the planned path geometry');
  assert.strictEqual(payload.route_candidates.length, 10, `expected top 10 routes, got ${payload.route_candidates.length}`);
  assert.strictEqual(new Set(payload.route_candidates.slice(0, 10).map(
    (candidate) => candidate.full_path_node_ids.join('-')
  )).size, 10, 'top 10 routes must be distinct');
  assert.strictEqual(await evaluate(`document.querySelectorAll('#routeRanking button').length`), 10);
  assert.strictEqual(await evaluate(`(() => {
    document.querySelectorAll('#routeRanking button')[1].click();
    const selected = document.querySelector('#routeCandidateSelect').value;
    const secondActive = document.querySelectorAll('#routeRanking button')[1].classList.contains('active');
    document.querySelectorAll('#routeRanking button')[0].click();
    return selected === '1' && secondActive;
  })()`), true, 'ranking cards must select and highlight their route');
  assert.strictEqual(payload.planned_lap_count, 3);
  assert.strictEqual(payload.stop_lap, 2);
  assert.strictEqual(payload.end_node_id, stopNodeId);
  assert.strictEqual(route.end_node_id, stopNodeId);
  assert.strictEqual(route.full_path_node_ids.at(-1), stopNodeId);
  assert(route.planned_path_points.length > 10);
  assert(route.planned_path_points.every((point) => Number.isFinite(point.speed_mps)));
  const reverseRouteAudit = await evaluate(`(() => {
    const basePath = state.routeCandidates[state.selectedRouteIndex].pathIds.slice();
    const expectedLap = [basePath[0], ...basePath.slice(1, -1).reverse(), basePath[0]];
    const originalDirection = state.lastRoute.traversal.direction;
    const originalPath = buildExportPayload().computed_shortest_route.full_path_node_ids;
    document.querySelector('#reverseRouteBtn').click();
    const reversed = buildExportPayload();
    const reversedRoute = reversed.computed_shortest_route;
    const firstLap = reversedRoute.full_path_node_ids.slice(0, expectedLap.length);
    const reversedDirection = reversed.travel_direction;
    const reversedFiniteSpeeds = reversedRoute.planned_path_points.every(point => Number.isFinite(point.speed_mps));
    const reversedStart = reversedRoute.full_path_node_ids[0];
    const reversedEnd = reversedRoute.full_path_node_ids.at(-1);
    const reversedActualLaps = state.lastRoute.traversal.actualLapCount;
    const reversedEdges = reversedRoute.full_path_edge_lines.slice(0, expectedLap.length - 1);
    document.querySelector('#reverseRouteBtn').click();
    const restored = buildExportPayload();
    return {
      expectedLap,
      firstLap,
      originalDirection,
      reversedDirection,
      reversedFlag: reversed.route_reversed,
      nestedReversedFlag: reversedRoute.route_reversed,
      reversedFiniteSpeeds,
      reversedStart,
      reversedEnd,
      reversedActualLaps,
      reversedEdges,
      expectedEdges: routeEdgeLines(expectedLap),
      restoredFlag: restored.route_reversed,
      restoredPath: restored.computed_shortest_route.full_path_node_ids,
      originalPath
    };
  })()`);
  assert.deepStrictEqual(reverseRouteAudit.firstLap, reverseRouteAudit.expectedLap,
    'reverse must preserve the start and reverse one closed lap before lap expansion');
  assert.strictEqual(reverseRouteAudit.reversedStart, route.full_path_node_ids[0]);
  assert.strictEqual(reverseRouteAudit.reversedEnd, stopNodeId);
  assert.strictEqual(reverseRouteAudit.reversedActualLaps, 2);
  assert.strictEqual(reverseRouteAudit.reversedFlag, true);
  assert.strictEqual(reverseRouteAudit.nestedReversedFlag, true);
  assert.strictEqual(reverseRouteAudit.reversedFiniteSpeeds, true);
  assert.deepStrictEqual(reverseRouteAudit.reversedEdges, reverseRouteAudit.expectedEdges,
    'edge export must follow the reversed traversal order');
  if (['clockwise', 'counterclockwise'].includes(reverseRouteAudit.originalDirection)) {
    assert.notStrictEqual(reverseRouteAudit.reversedDirection, reverseRouteAudit.originalDirection,
      'closed route direction must flip after reversing');
  }
  assert.strictEqual(reverseRouteAudit.restoredFlag, false);
  assert.deepStrictEqual(reverseRouteAudit.restoredPath, reverseRouteAudit.originalPath,
    'second click must restore the original multi-lap traversal');
  const speedConstraintAudit = await evaluate(`(() => {
    const settings = state.lastRoute.speedSettings;
    const points = state.lastRoute.plannedPathPoints;
    let maximumAccelExcessMps2 = 0;
    let maximumDecelExcessMps2 = 0;
    for (let index = 1; index < points.length; index += 1) {
      const distanceM = pointDistance(points[index - 1], points[index]) / 1000;
      const speedSquaredDelta = points[index].speedMps ** 2 - points[index - 1].speedMps ** 2;
      if (speedSquaredDelta >= 0) {
        maximumAccelExcessMps2 = Math.max(maximumAccelExcessMps2,
          speedSquaredDelta - 2 * settings.effectiveAccelerationMps2 * distanceM);
      } else {
        maximumDecelExcessMps2 = Math.max(maximumDecelExcessMps2,
          -speedSquaredDelta - 2 * settings.effectiveDecelerationMps2 * distanceM);
      }
    }
    const synthetic = applyGlobalSpeedConstraints([
      { x: 0, y: 0, speedMps: 3 },
      { x: 10, y: 0, speedMps: 2.2 }
    ], 30, false);
    return {
      maximumAccelExcessMps2,
      maximumDecelExcessMps2,
      gripAccelerationMps2: settings.gripAccelerationMps2,
      accelerationMps2: settings.accelerationMps2,
      decelerationMps2: settings.decelerationMps2,
      effectiveAccelerationMps2: settings.effectiveAccelerationMps2,
      effectiveDecelerationMps2: settings.effectiveDecelerationMps2,
      startupDragTimeS: settings.startupDragTimeS,
      startupDragSpeedMps: settings.startupDragSpeedMps,
      startupDragDistanceMm: settings.startupDragDistanceMm,
      startupSpeeds: points
        .filter(point => point.sMm <= settings.startupDragDistanceMm)
        .map(point => point.speedMps),
      postStartupMaximumSpeedMps: Math.max(...points
        .filter(point => point.sMm > settings.startupDragDistanceMm + plannedPathSpacingMm)
        .map(point => point.speedMps)),
      plannedStartSpeedMps: points[0].speedMps,
      plannedEndSpeedMps: points.at(-1).speedMps,
      syntheticStartSpeedMps: synthetic[0].speedMps,
      syntheticEndSpeedMps: synthetic[1].speedMps
    };
  })()`);
  assert(speedConstraintAudit.maximumAccelExcessMps2 < 0.01,
    `planned speeds must obey the global acceleration limit: ${JSON.stringify(speedConstraintAudit)}`);
  assert(speedConstraintAudit.maximumDecelExcessMps2 < 0.01,
    `planned speeds must obey the global deceleration limit: ${JSON.stringify(speedConstraintAudit)}`);
  assert(Math.abs(speedConstraintAudit.gripAccelerationMps2 - 25) < 0.1,
    `default lateral acceleration must match the MCU 25 m/s2: ${JSON.stringify(speedConstraintAudit)}`);
  assert.strictEqual(speedConstraintAudit.accelerationMps2, 30);
  assert.strictEqual(speedConstraintAudit.decelerationMps2, 30);
  assert.strictEqual(speedConstraintAudit.effectiveAccelerationMps2, 30,
    `planning acceleration must not be clamped by the lateral friction limit: ${JSON.stringify(speedConstraintAudit)}`);
  assert.strictEqual(speedConstraintAudit.effectiveDecelerationMps2, 30,
    `planning deceleration must not be clamped by the lateral friction limit: ${JSON.stringify(speedConstraintAudit)}`);
  assert.strictEqual(speedConstraintAudit.startupDragTimeS, 0.4);
  assert.strictEqual(speedConstraintAudit.startupDragSpeedMps, 0.2);
  assert(Math.abs(speedConstraintAudit.startupDragDistanceMm - 80) < 1e-6);
  assert(speedConstraintAudit.startupSpeeds.length > 1
    && speedConstraintAudit.startupSpeeds.every(speed => speed <= 0.2 + 1e-6),
    `startup drag distance must remain at the MCU drag speed: ${JSON.stringify(speedConstraintAudit)}`);
  assert(speedConstraintAudit.postStartupMaximumSpeedMps > 0.2,
    `planned acceleration must begin after startup drag: ${JSON.stringify(speedConstraintAudit)}`);
  const asymmetricProfile = await evaluate(`(() => {
    const profile = motionProfileMeters(1, 0, 0, 10, 10, 30, 0.01);
    const peak = profile.samples.reduce((best, sample) => sample.speedMps > best.speedMps ? sample : best);
    return { peakDistanceM: peak.distanceM, peakSpeedMps: peak.speedMps };
  })()`);
  assert(asymmetricProfile.peakDistanceM > 0.7 && asymmetricProfile.peakDistanceM < 0.8,
    `stronger deceleration must move the triangular-profile peak later: ${JSON.stringify(asymmetricProfile)}`);
  assert(Math.abs(asymmetricProfile.peakSpeedMps ** 2 - 15) < 0.2,
    `asymmetric triangular-profile peak must use both limits: ${JSON.stringify(asymmetricProfile)}`);
  assert(speedConstraintAudit.plannedStartSpeedMps <= 0.2,
    `the startup drag segment must use the MCU drag speed: ${JSON.stringify(speedConstraintAudit)}`);
  assert.strictEqual(speedConstraintAudit.plannedEndSpeedMps, 0,
    'the finite route must end at zero speed');
  assert(speedConstraintAudit.syntheticStartSpeedMps < 2.34,
    'an impossible 3.0 to 2.2 m/s drop over 10 mm must propagate backward');
  assert.strictEqual(speedConstraintAudit.syntheticEndSpeedMps, 2.2);
  assert(['clockwise', 'counterclockwise', 'mixed_or_open'].includes(payload.travel_direction));
  assert(Number.isFinite(payload.start_heading_deg));
  await evaluate(`(() => {
    const mode = document.querySelector('#routeModeSelect');
    mode.value = 'exact';
    mode.dispatchEvent(new Event('change', { bubbles: true }));
    document.querySelector('#routeBtn').click();
  })()`);
  for (let attempt = 0; attempt < 200; attempt += 1) {
    const exactReady = await evaluate(`(() => {
      if (document.querySelector('#routeBtn').disabled) return false;
      const payload = JSON.parse(document.querySelector('#exportText').value);
      return payload.route_mode === 'exact' && payload.route_candidates.length >= 10;
    })()`);
    if (exactReady) break;
    if (attempt === 199) throw new Error('exact route computation timed out');
    await delay(50);
  }
  const exactPayload = await evaluate(`JSON.parse(document.querySelector('#exportText').value)`);
  assert(exactPayload.route_candidates.slice(0, 10).every((candidate) =>
    Number.isFinite(candidate.estimated_time_s) && Number.isFinite(candidate.effective_distance_mm)
  ), 'exact candidates must include the shared radius and speed plan');
  assert(exactPayload.computed_shortest_route.planned_path_points.length > 10);
  assert(exactPayload.computed_shortest_route.planned_path_points.every((point) => Number.isFinite(point.speed_mps)));
  const exactVisuals = await evaluate(`(() => {
    const labelsToggle = document.querySelector('#showLabels');
    const labelsPreviouslyEnabled = labelsToggle.checked;
    labelsToggle.checked = true;
    const originalColor = speedToHeatColor;
    const originalFillText = ctx.fillText;
    let colorCalls = 0;
    const labels = [];
    speedToHeatColor = (speed) => {
      colorCalls += 1;
      return originalColor(speed);
    };
    ctx.fillText = function(value, ...args) {
      labels.push(String(value));
      return originalFillText.call(ctx, value, ...args);
    };
    draw();
    labelsToggle.checked = labelsPreviouslyEnabled;
    speedToHeatColor = originalColor;
    ctx.fillText = originalFillText;
    return {
      colorCalls,
      arcPieces: (state.lastRoute.speedPieces || []).filter(piece => piece.type === 'arc').length,
      labels,
      radiusLabels: labels.filter(label => label.includes('· R ') && label.includes('m/s')).length,
      coordinateLabels: labels.filter(label => label.startsWith('X ') && label.includes('· Y ')).length
    };
  })()`);
  assert(exactVisuals.colorCalls > 10, 'exact route must use point-speed coloring');
  assert(exactVisuals.radiusLabels > 0, `exact route must draw radius and speed labels: ${JSON.stringify(exactVisuals)}`);
  assert(exactVisuals.coordinateLabels > 0, `exact route must draw turn coordinates: ${JSON.stringify(exactVisuals)}`);
  const exactCandidates = exactPayload.route_candidates.slice(0, 10);
  assert(exactCandidates.every((candidate) =>
    Number.isFinite(candidate.alternating_turn_count)
    && Number.isFinite(candidate.alternating_turn_distance_sum_mm)
    && candidate.left_to_right_count + candidate.right_to_left_count === candidate.alternating_turn_count
  ), 'exact candidates must expose consistent turn-alternation metrics');
  assert(exactCandidates.slice(1).every((candidate, index) => {
    const previous = exactCandidates[index];
    return candidate.alternating_turn_count > previous.alternating_turn_count
      || (candidate.alternating_turn_count === previous.alternating_turn_count
        && candidate.alternating_turn_distance_sum_mm < previous.alternating_turn_distance_sum_mm)
      || (candidate.alternating_turn_count === previous.alternating_turn_count
        && candidate.alternating_turn_distance_sum_mm === previous.alternating_turn_distance_sum_mm
        && candidate.distance_mm >= previous.distance_mm);
  }), 'exact candidates must use alternation count, spacing sum, then distance ordering');
  const optimizedAudit = await evaluate(`(async () => {
    state.optimizerCancelRequested = false;
    let progressUpdates = 0;
    const result = await findOptimizedRequiredRoutes(1, 1000, () => { progressUpdates += 1; });
    const candidates = result.candidates || [result];
    const requiredIds = new Set(state.nodes.filter(node => node.type === 0 || node.type === 1).map(node => node.id));
    return {
      ok: result.ok,
      progressUpdates,
      candidateCount: candidates.length,
      valid: candidates.every(route => {
        const visited = new Set(route.pathIds);
        const noImmediateBacktracking = route.pathIds.every((id, index, ids) => (
          index === 0 || index === ids.length - 1 || ids[index - 1] !== ids[index + 1]
        ));
        return route.pathIds[0] === 1
          && route.pathIds.at(-1) === 1
          && [...requiredIds].every(id => visited.has(id))
          && noImmediateBacktracking
          && route.plannedPathPoints.length > 0
          && route.plannedPathPoints.every(point => Number.isFinite(point.speedMps));
      })
    };
  })()`);
  assert(optimizedAudit.ok, 'deep optimizer must return a route');
  assert(optimizedAudit.progressUpdates > 0, 'deep optimizer must report progress while yielding');
  assert(optimizedAudit.candidateCount > 0 && optimizedAudit.candidateCount <= 10);
  assert(optimizedAudit.valid, `deep optimizer produced an invalid candidate: ${JSON.stringify(optimizedAudit)}`);
  const absoluteSeedAudit = await evaluate(`(async () => {
    const cases = [];
    for (const seedPath of [[1, 2, 3, 4], [2, 3, 4]]) {
      state.optimizerSeeds = [{ signature: seedPath.join('-'), pathIds: seedPath }];
      state.optimizerCancelRequested = false;
      const result = await findOptimizedRequiredRoutes(1, 1000, () => {});
      const candidates = result.candidates || [result];
      const containsSeed = (pathIds) => pathIds.some((id, index) => (
        seedPath.every((seedId, offset) => pathIds[index + offset] === seedId)
      ));
      cases.push({
        ok: result.ok,
        candidateCount: candidates.length,
        allLocked: candidates.every((route) => containsSeed(route.pathIds))
      });
    }
    state.optimizerSeeds = [];
    return cases;
  })()`);
  absoluteSeedAudit.forEach((audit) => {
    assert.strictEqual(audit.ok, true, 'absolute seed search must return a route');
    assert.strictEqual(audit.allLocked, true,
      'absolute optimizer seeds must remain contiguous and direction-locked in every candidate');
    assert(audit.candidateCount > 0 && audit.candidateCount <= 10,
      `absolute seed search returned an invalid candidate count: ${JSON.stringify(absoluteSeedAudit)}`);
  });
  const startCorridorSeedAudit = await evaluate(`(async () => {
    const originalNodes = state.nodes;
    const originalLinks = state.links;
    const originalSeeds = state.optimizerSeeds;
    state.nodes = [
      { id: 1, name: 'P1', x: 0, y: 0, type: 0 },
      { id: 2, name: 'P2', x: 100, y: 0, type: 2 },
      { id: 3, name: 'P3', x: 200, y: 0, type: 2 },
      { id: 4, name: 'P4', x: 300, y: 100, type: 0 },
      { id: 5, name: 'P5', x: 300, y: -100, type: 0 }
    ];
    state.links = [[1, 2], [2, 3], [3, 4], [4, 5], [5, 3]].map(([fromId, toId]) => ({
      key: makeLinkKey(fromId, toId), fromId, toId
    }));
    state.optimizerSeeds = [{ signature: '1-2-3', pathIds: [1, 2, 3] }];
    state.optimizerCancelRequested = false;
    const result = await findOptimizedRequiredRoutes(1, 1000, () => {});
    const candidates = result.candidates || [result];
    const audit = {
      ok: result.ok,
      candidateCount: candidates.length,
      allStartWithSeed: candidates.every(route => route.pathIds.slice(0, 3).join('-') === '1-2-3'),
      allReturnThroughCorridor: candidates.every(route => route.pathIds.slice(-3).join('-') === '3-2-1')
    };
    state.nodes = originalNodes;
    state.links = originalLinks;
    state.optimizerSeeds = originalSeeds;
    return audit;
  })()`);
  assert.strictEqual(startCorridorSeedAudit.ok, true,
    `a start-prefix seed must allow the closed route to reuse its corridor: ${JSON.stringify(startCorridorSeedAudit)}`);
  assert.strictEqual(startCorridorSeedAudit.allStartWithSeed, true,
    'a start-prefix absolute seed must be executed at departure');
  assert.strictEqual(startCorridorSeedAudit.allReturnThroughCorridor, true,
    'a closed route must be allowed to return through start-prefix seed nodes');
  const overlappingSeedAudit = await evaluate(`(async () => {
    const originalNodes = state.nodes;
    const originalLinks = state.links;
    const originalSeeds = state.optimizerSeeds;
    state.nodes = [
      { id: 1, name: 'P1', x: 0, y: 0, type: 0 },
      { id: 2, name: 'P2', x: 100, y: 0, type: 2 },
      { id: 3, name: 'P3', x: 200, y: 0, type: 0 },
      { id: 4, name: 'P4', x: 300, y: 0, type: 2 },
      { id: 5, name: 'P5', x: 400, y: 0, type: 0 },
      { id: 6, name: 'P6', x: 400, y: 100, type: 0 }
    ];
    state.links = [[1, 2], [2, 3], [3, 4], [4, 5], [5, 6], [6, 4]].map(([fromId, toId]) => ({
      key: makeLinkKey(fromId, toId), fromId, toId
    }));
    state.optimizerSeeds = [{ signature: '3-4-5', pathIds: [3, 4, 5] }];
    state.optimizerCancelRequested = false;
    const result = await findOptimizedRequiredRoutes(1, 1000, () => {});
    const candidates = result.candidates || [result];
    const containsSeed = pathIds => pathIds.some((id, index) => (
      [3, 4, 5].every((seedId, offset) => pathIds[index + offset] === seedId)
    ));
    const audit = {
      ok: result.ok,
      candidateCount: candidates.length,
      allContainSeed: candidates.every(route => containsSeed(route.pathIds)),
      allReuseSeedNodes: candidates.every(route => route.pathIds.filter(id => id === 4).length >= 2)
    };
    state.nodes = originalNodes;
    state.links = originalLinks;
    state.optimizerSeeds = originalSeeds;
    return audit;
  })()`);
  assert.strictEqual(overlappingSeedAudit.ok, true,
    `absolute seeds must not make shared roads exclusive: ${JSON.stringify(overlappingSeedAudit)}`);
  assert.strictEqual(overlappingSeedAudit.allContainSeed, true,
    'an overlapping absolute seed must still be executed contiguously and direction-locked');
  assert.strictEqual(overlappingSeedAudit.allReuseSeedNodes, true,
    'other route sections must be allowed to cross or reuse absolute seed roads');
  const livePreviewAudit = await evaluate(`(async () => {
    state.optimizerCancelRequested = false;
    state.optimizerPaused = false;
    let snapshotCount = 0;
    let runningSnapshot = false;
    let candidateCount = 0;
    const result = await findOptimizedRequiredRoutes(1, 5000, () => {}, async (routes, meta) => {
      snapshotCount += 1;
      runningSnapshot = runningSnapshot || meta.paused === false;
      candidateCount = routes.length;
      state.optimizerCancelRequested = true;
    }, 1000);
    const audit = {
      ok: result.ok,
      snapshotCount,
      runningSnapshot,
      candidateCount
    };
    state.optimizerCancelRequested = false;
    state.optimizerPaused = false;
    return audit;
  })()`);
  assert.strictEqual(livePreviewAudit.ok, true, 'live-preview optimizer must still return a route');
  assert.strictEqual(livePreviewAudit.snapshotCount, 1, 'live preview must publish while optimization is running');
  assert.strictEqual(livePreviewAudit.runningSnapshot, true, 'live preview must be identified as a running snapshot');
  assert(livePreviewAudit.candidateCount > 0 && livePreviewAudit.candidateCount <= 10,
    `live preview must expose the current top routes: ${JSON.stringify(livePreviewAudit)}`);
  assert.strictEqual(await evaluate(`(() => {
    const fasterWithMoreReversals = { timeSeconds: 5, alternatingTurnCount: 9, alternatingTurnDistanceMm: 100, distance: 2000 };
    const slowerWithFewerReversals = { timeSeconds: 6, alternatingTurnCount: 0, alternatingTurnDistanceMm: 1000, distance: 1000 };
    return compareOptimizedRoutes(fasterWithMoreReversals, slowerWithFewerReversals) < 0;
  })()`), true, 'deep optimizer must rank estimated time before reversal count');
  const pauseResumeAudit = await evaluate(`(async () => {
    state.optimizerCancelRequested = false;
    state.optimizerPaused = false;
    let requestedPause = false;
    let snapshotCount = 0;
    let resumedProgress = false;
    const result = await findOptimizedRequiredRoutes(1, 5000, ({ iterations }) => {
      if (!requestedPause && iterations > 0) {
        requestedPause = true;
        state.optimizerPaused = true;
      } else if (snapshotCount > 0 && iterations > 0) {
        resumedProgress = true;
        state.optimizerCancelRequested = true;
      }
    }, async (routes) => {
      snapshotCount += 1;
      if (!routes.length) throw new Error('paused optimizer did not expose current routes');
      state.optimizerPaused = false;
    }, 100000);
    const audit = {
      ok: result.ok,
      snapshotCount,
      resumedProgress,
      forcedStopReported: result.message.includes('强制停止')
    };
    state.optimizerCancelRequested = false;
    state.optimizerPaused = false;
    return audit;
  })()`);
  assert.deepStrictEqual(pauseResumeAudit, {
    ok: true,
    snapshotCount: 1,
    resumedProgress: true,
    forcedStopReported: true
  }, 'deep optimizer pause, snapshot, resume, and force-stop lifecycle must remain continuous');
  const manualRouteAudit = await evaluate(`(() => {
    const mode = document.querySelector('#routeModeSelect');
    mode.value = 'manual';
    startManualRoutePlanning();
    appendManualRouteTarget(2);
    appendManualRouteTarget(3);
    undoManualRouteSegment();
    appendManualRouteTarget(3);
    finishManualRoutePlanning();
    const route = state.lastRoute;
    const seedButtonEnabled = !document.querySelector('#saveOptimizerSeedBtn').disabled;
    document.querySelector('#saveOptimizerSeedBtn').click();
    const savedSeedCount = state.optimizerSeeds.length;
    document.querySelector('#saveOptimizerSeedBtn').click();
    const duplicateSeedReported = document.querySelector('#routeSummary').textContent.includes('已经保存');
    document.querySelector('#clearOptimizerSeedsBtn').click();
    const audit = {
      algorithm: route.algorithm,
      stops: route.manualStopIds,
      startsAt: route.pathIds[0],
      endsAt: route.pathIds.at(-1),
      planning: state.manualPlanning,
      finiteSpeeds: route.plannedPathPoints.every(point => Number.isFinite(point.speedMps)),
      edgeText: document.querySelector('#routeEdgeText').value,
      edgeCount: routeEdgeLines(route.pathIds).length,
      exportedEdges: buildExportPayload().computed_shortest_route.full_path_edge_lines,
      seedButtonEnabled,
      savedSeedCount,
      duplicateSeedReported,
      longDurations: ['3600', '21600'].every(value => (
        [...document.querySelector('#optimizerDurationSelect').options].some(option => option.value === value)
      ))
    };
    mode.value = 'exact';
    return audit;
  })()`);
  assert.deepStrictEqual(manualRouteAudit, {
    algorithm: 'manual',
    stops: [1, 2, 3],
    startsAt: 1,
    endsAt: 3,
    planning: false,
    finiteSpeeds: true,
    edgeText: '1-2\n2-3',
    edgeCount: 2,
    exportedEdges: ['1-2', '2-3'],
    seedButtonEnabled: true,
    savedSeedCount: 1,
    duplicateSeedReported: true,
    longDurations: true
  }, 'manual planning must preserve clicked stop order and an explicitly open endpoint');
  const largeOptimizerAudit = await evaluate(`(async () => {
    const originalNodes = state.nodes;
    const originalLinks = state.links;
    const count = 170;
    state.nodes = Array.from({ length: count }, (_, index) => {
      const angle = index / count * Math.PI * 2;
      return {
        id: index + 1,
        name: 'L' + (index + 1),
        x: Math.cos(angle) * 5000,
        y: Math.sin(angle) * 5000,
        type: 0,
        typeLabel: '必经元素节点'
      };
    });
    state.links = Array.from({ length: count }, (_, index) => {
      const fromId = index + 1;
      const toId = (index + 1) % count + 1;
      return { key: makeLinkKey(fromId, toId), fromId, toId };
    });
    state.optimizerCancelRequested = false;
    const result = await findOptimizedRequiredRoutes(1, 1000, () => {});
    const path = result.pathIds || [];
    const audit = {
      ok: result.ok,
      requiredCount: result.requiredIds?.length || 0,
      closed: path.length > 1 && path[0] === 1 && path.at(-1) === 1,
      covered: new Set(path).size >= count,
      noImmediateBacktracking: path.every((id, index, ids) => (
        index === 0 || index === ids.length - 1 || ids[index - 1] !== ids[index + 1]
      ))
    };
    state.nodes = originalNodes;
    state.links = originalLinks;
    return audit;
  })()`);
  assert.deepStrictEqual(largeOptimizerAudit, {
    ok: true,
    requiredCount: 170,
    closed: true,
    covered: true,
    noImmediateBacktracking: true
  }, 'deep optimizer must support 170 required nodes end to end');
  const thresholdMetrics = await evaluate(`(() => {
    const originalNodes = state.nodes;
    state.nodes = [
      { id: 1, x: 0, y: 0 },
      { id: 2, x: 100, y: 0 },
      { id: 3, x: 100, y: 100 },
      { id: 4, x: 1100, y: 100 },
      { id: 5, x: 1100, y: 1100 }
    ];
    const metrics = routeTurnAlternationMetrics([1, 2, 3, 4, 5]);
    state.nodes = originalNodes;
    return metrics;
  })()`);
  assert.strictEqual(thresholdMetrics.alternatingTurnCount, 1, 'turn reversals over 900 mm must not count');
  assert.strictEqual(thresholdMetrics.alternatingTurnDistanceMm, 100);
  const hairpinExtensionCases = await evaluate(`(() => {
    const originalNodes = state.nodes;
    const settings = {
      ...readSpeedSettings(),
      curveRadiusCm: 20,
      alternatingTurnOffsetEnabled: true,
      alternatingTurnThresholdMm: 900,
      alternatingTurnOffsetMm: 100
    };
    const run = (nodes, enabled = true, overrides = {}) => {
      state.nodes = nodes.map((node, index) => ({
        id: index + 1,
        type: node.type ?? ((index === 0 || index === nodes.length - 1) ? 0 : 2),
        ...node
      }));
      const coordinatesBeforePlanning = state.nodes.map(({ x, y }) => ({ x, y }));
      const pathIds = state.nodes.map((node) => node.id);
      const caseSettings = {
        ...settings,
        alternatingTurnOffsetEnabled: enabled,
        ...overrides
      };
      const plan = buildRouteTurnPlan(pathIds, caseSettings);
      const profile = buildRouteMotionProfile(pathIds, caseSettings);
      const spacings = profile.plannedPoints.slice(1).map((point, index) => pointDistance(point, profile.plannedPoints[index]));
      return {
        extensions: profile.alternatingTurnTransitions,
        planningNodes: plan.nodes.map(({ x, y }) => ({ x, y })),
        pointsFinite: profile.plannedPoints.every((point) => Number.isFinite(point.x) && Number.isFinite(point.y)),
        maxSpacingErrorMm: Math.max(0, ...spacings.slice(0, -1).map((spacing) => Math.abs(spacing - plannedPathSpacingMm))),
        realNodesUnchanged: JSON.stringify(coordinatesBeforePlanning)
          === JSON.stringify(state.nodes.map(({ x, y }) => ({ x, y })))
      };
    };
    const hairpinNodes = [
      { x: 0, y: 0 }, { x: 0, y: 600 }, { x: 500, y: 600 }, { x: 500, y: 0 }
    ];
    const result = {
      hairpin: run(hairpinNodes),
      baseline: run(hairpinNodes, false),
      mirrored: run(hairpinNodes.map((node) => ({ x: node.x, y: -node.y }))),
      sampledHead: run([
        { x: 0, y: 0 }, { x: 0, y: 600 }, { x: 250, y: 600 },
        { x: 500, y: 600 }, { x: 500, y: 0 }
      ]),
      sBend: run([
        { x: -300, y: 0 }, { x: 0, y: 0 }, { x: 0, y: 100 }, { x: 300, y: 100 }
      ]),
      tooWide: run([
        { x: 0, y: 0 }, { x: 0, y: 600 }, { x: 1000, y: 600 }, { x: 1000, y: 0 }
      ]),
      p24P26: run([
        { x: 2200, y: 1750, type: 2 }, { x: 1900, y: 1750, type: 0 },
        { x: 1600, y: 1750 }, { x: 1600, y: 1300 },
        { x: 1900, y: 1300, type: 0 }, { x: 2200, y: 1300, type: 2 }
      ], true, {
        alternatingTurnOffsetMm: 200
      }),
      p31P30: run([
        { x: 1870, y: 2200, type: 0 },
        { x: 2200, y: 2200, type: 2 },
        { x: 2200, y: 1750, type: 2 },
        { x: 1900, y: 1750, type: 0 }
      ], true, {
        alternatingTurnOffsetMm: 200
      }),
      actualRouteWindow: run([
        { x: 1000, y: 2200, type: 1 },
        { x: 1480, y: 2200, type: 0 },
        { x: 1870, y: 2200, type: 0 },
        { x: 2200, y: 2200, type: 2 },
        { x: 2200, y: 1750, type: 2 },
        { x: 1900, y: 1750, type: 0 },
        { x: 1600, y: 1750, type: 2 },
        { x: 1600, y: 1300, type: 2 },
        { x: 1900, y: 1300, type: 0 },
        { x: 2200, y: 1300, type: 2 },
        { x: 2200, y: 850, type: 1 }
      ], true, {
        alternatingTurnOffsetMm: 200
      }),
      atThreshold: run(hairpinNodes, true, {
        alternatingTurnThresholdMm: 500
      }),
      notReversing: run([
        { x: 0, y: 0 }, { x: 0, y: 600 },
        { x: 500, y: 600 }, { x: 1000, y: 0 }
      ])
    };
    state.nodes = originalNodes;
    return result;
  })()`);
  assert.strictEqual(hairpinExtensionCases.hairpin.extensions.length, 1, 'a short rectangular hairpin must extend its head');
  assert.strictEqual(hairpinExtensionCases.mirrored.extensions.length, 1, 'a mirrored hairpin must extend away from its baseline');
  assert.strictEqual(hairpinExtensionCases.sBend.extensions.length, 0, 'an S bend is not a rectangular hairpin');
  assert.strictEqual(hairpinExtensionCases.tooWide.extensions.length, 0, 'a hairpin beyond the turn-pair spacing threshold must not extend');
  const extension = hairpinExtensionCases.hairpin.extensions[0];
  assert.strictEqual(extension.originalDepthMm, 600);
  assert.strictEqual(extension.addedDepthMm, 100);
  assert.strictEqual(extension.virtualDepthMm, 700);
  assert.strictEqual(extension.headWidthMm, 500);
  assert.deepStrictEqual(hairpinExtensionCases.hairpin.planningNodes, [
    { x: 0, y: 0 }, { x: 0, y: 700 }, { x: 500, y: 700 }, { x: 500, y: 0 }
  ], 'the entire head must move while both bottom anchors remain fixed');
  assert.strictEqual(hairpinExtensionCases.sampledHead.planningNodes[2].y, 700,
    'straight sampling points on the head must move with both head corners');
  assert.strictEqual(hairpinExtensionCases.p24P26.extensions.length, 1);
  assert.deepStrictEqual(hairpinExtensionCases.p24P26.planningNodes, [
    { x: 2200, y: 1750 }, { x: 1900, y: 1750 },
    { x: 1400, y: 1750 }, { x: 1400, y: 1300 },
    { x: 1900, y: 1300 }, { x: 2200, y: 1300 }
  ], 'P24/P26 virtual points must move left together while all surrounding real points remain fixed');
  assert.strictEqual(hairpinExtensionCases.p24P26.extensions[0].headWidthMm, 450);
  assert.strictEqual(hairpinExtensionCases.p24P26.extensions[0].elementConnectionDistanceMm, 450);
  assert.strictEqual(hairpinExtensionCases.p31P30.extensions.length, 0,
    'P31/P30 is asymmetric and must not be treated as the P24/P26 extension structure');
  assert.deepStrictEqual(
    hairpinExtensionCases.actualRouteWindow.extensions.map((item) => [item.firstNodeId, item.secondNodeId]),
    [[7, 8]],
    'the actual route window must select only P24/P26 and reject P31/P30'
  );
  assert.strictEqual(hairpinExtensionCases.atThreshold.extensions.length, 0,
    'turn-pair spacing equal to the limit must be ignored');
  assert.strictEqual(hairpinExtensionCases.notReversing.extensions.length, 0,
    'short geometry without an opposing entry/exit heading must be ignored');
  assert(hairpinExtensionCases.hairpin.realNodesUnchanged, 'planning must not mutate authoritative node coordinates');
  assert(hairpinExtensionCases.hairpin.pointsFinite && hairpinExtensionCases.mirrored.pointsFinite);
  assert(hairpinExtensionCases.hairpin.maxSpacingErrorMm < 0.02, 'extended hairpin must retain 5 mm sampling');
  const stretchVisualization = await evaluate(`(() => {
    state.nodes = [
      { id: 30, name: 'P30', x: 2200, y: 1750, type: 2, typeLabel: '弯道节点' },
      { id: 29, name: 'P29', x: 1900, y: 1750, type: 0, typeLabel: '直道元素节点' },
      { id: 24, name: 'P24', x: 1600, y: 1750, type: 2, typeLabel: '弯道节点' },
      { id: 26, name: 'P26', x: 1600, y: 1300, type: 2, typeLabel: '弯道节点' },
      { id: 28, name: 'P28', x: 1900, y: 1300, type: 0, typeLabel: '直道元素节点' },
      { id: 27, name: 'P27', x: 2200, y: 1300, type: 2, typeLabel: '弯道节点' }
    ];
    const settings = {
      ...readSpeedSettings(), curveRadiusCm: 20,
      alternatingTurnOffsetEnabled: true, alternatingTurnThresholdMm: 900,
      alternatingTurnOffsetMm: 200
    };
    const pathIds = [30, 29, 24, 26, 28, 27];
    const profile = buildRouteMotionProfile(pathIds, settings);
    state.lastRoute = {
      ok: true, pathIds, speedSettings: settings,
      speedPieces: profile.pieces, plannedPathPoints: profile.plannedPoints,
      alternatingTurnTransitions: profile.alternatingTurnTransitions
    };
    draw();
    return {
      stretches: profile.alternatingTurnTransitions.length,
      summary: alternatingTurnStretchSummary(state.lastRoute)
    };
  })()`);
  assert.strictEqual(stretchVisualization.stretches, 1);
  assert(stretchVisualization.summary.includes('P29↔P28 元素连线 450 mm'), stretchVisualization.summary);
  assert(stretchVisualization.summary.includes('虚拟 P24→P26'), stretchVisualization.summary);
  const segmentOffsetAudit = await evaluate(`(() => {
    state.lastRoute = null;
    state.routeCandidates = [];
    state.manualPlanning = false;
    state.nodes = [
      { id: 1, name: 'P1', x: 0, y: 0, type: 0, typeLabel: '直道元素节点' },
      { id: 2, name: 'P2', x: 50, y: 1, type: 2, typeLabel: '弯道节点' },
      { id: 3, name: 'P3', x: 100, y: 0, type: 1, typeLabel: '弯道元素节点' },
      { id: 4, name: 'P4', x: 50, y: 30, type: 2, typeLabel: '弯道节点' }
    ];
    state.links = [
      { key: '1-2', fromId: 1, toId: 2, fromName: 'P1', toName: 'P2' },
      { key: '2-3', fromId: 2, toId: 3, fromName: 'P2', toName: 'P3' }
    ];
    const linksBefore = JSON.stringify(state.links);
    state.segmentOffset.active = true;
    state.segmentOffset.startId = 1;
    state.segmentOffset.endId = 3;
    state.segmentOffset.undoCoordinates = null;
    els.segmentOffsetTolerance.value = '5';
    els.segmentOffsetX.value = '10';
    els.segmentOffsetY.value = '-20';
    refreshSegmentOffsetSelection();
    const selectedIds = [...state.segmentOffset.selectedIds];
    applySegmentOffset();
    const moved = state.nodes.map(({ id, x, y }) => ({ id, x, y }));
    const inputAfterMove = els.bulkInput.value;
    const linksUnchanged = JSON.stringify(state.links) === linksBefore;
    undoSegmentOffset();
    return {
      selectedIds,
      moved,
      inputAfterMove,
      linksUnchanged,
      restored: state.nodes.map(({ id, x, y }) => ({ id, x, y })),
      undoDisabled: els.segmentOffsetUndoBtn.disabled
    };
  })()`);
  assert.deepStrictEqual(segmentOffsetAudit.selectedIds, [1, 2, 3],
    'segment offset must select only nodes near the finite selected segment');
  assert.deepStrictEqual(segmentOffsetAudit.moved, [
    { id: 1, x: 10, y: -20 }, { id: 2, x: 60, y: -19 },
    { id: 3, x: 110, y: -20 }, { id: 4, x: 50, y: 30 }
  ]);
  assert(segmentOffsetAudit.inputAfterMove.includes('60,-19,2'), 'bulk input must reflect shifted coordinates');
  assert.strictEqual(segmentOffsetAudit.linksUnchanged, true, 'segment offset must preserve graph links');
  assert.deepStrictEqual(segmentOffsetAudit.restored, [
    { id: 1, x: 0, y: 0 }, { id: 2, x: 50, y: 1 },
    { id: 3, x: 100, y: 0 }, { id: 4, x: 50, y: 30 }
  ], 'undo must restore every shifted coordinate');
  assert.strictEqual(segmentOffsetAudit.undoDisabled, true, 'undo must be single-use after restoring coordinates');
  console.log(JSON.stringify({
    points: route.planned_path_points.length,
    candidates: payload.route_candidates.length,
    exactCandidates: exactPayload.route_candidates.length,
    laps: payload.planned_lap_count,
    stopLap: payload.stop_lap,
    stopNodeId,
    direction: payload.travel_direction
  }));
  const screenshot = await send('Page.captureScreenshot', { format: 'png', fromSurface: true });
  fs.writeFileSync(path.join(os.tmpdir(), 'node-planner-ui.png'), Buffer.from(screenshot.data, 'base64'));
  await send('Browser.close');
}

main().finally(() => {
  if (!browser.killed) browser.kill();
});
