'use strict';

const assert = require('assert');
const { spawn } = require('child_process');
const fs = require('fs');
const os = require('os');
const path = require('path');

const browserExecutable = 'C:\\Program Files\\Google\\Chrome\\Application\\chrome.exe';
const port = 9400 + Math.floor(Math.random() * 400);
const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'pathcapture-cdp-'));
const outputDir = process.argv[3] || path.join(os.tmpdir(), 'pathcapture-ui');
let replayImportFile = null;
function createSyntheticImport() {
  const points = [];
  for (let x = 0; x <= 1000; x += 5) points.push({ x, y: 0 });
  for (let angle = -Math.PI / 2; angle <= Math.PI / 2; angle += 5 / 200) {
    points.push({ x: 1000 + Math.cos(angle) * 200, y: 200 + Math.sin(angle) * 200 });
  }
  for (let x = 995; x >= 0; x -= 5) points.push({ x, y: 400 });
  for (let angle = Math.PI / 2; angle <= Math.PI * 1.5; angle += 5 / 200) {
    points.push({ x: Math.cos(angle) * 200, y: 200 + Math.sin(angle) * 200 });
  }
  const replayPoints = [
    { x: 100, y: 0, theta_deg: 0, elapsed_s: 0 },
    { x: 500, y: 0, theta_deg: 0, elapsed_s: 0.2 },
    { x: 900, y: 0, theta_deg: 0, elapsed_s: 0.4 },
    { x: 1000, y: 50, theta_deg: -90, elapsed_s: 0.6 }
  ];
  const filename = path.join(profile, 'synthetic-path.json');
  fs.writeFileSync(filename, JSON.stringify({ version: '3.0', predefined: points, replay_runs: [] }));
  replayImportFile = path.join(profile, 'synthetic-replay.json');
  fs.writeFileSync(replayImportFile, JSON.stringify({
    version: '3.0',
    predefined: points,
    replay_runs: [{ id: 'vehicle-sim', status: 'imported', points: replayPoints, lookahead: [] }]
  }));
  return filename;
}
const importFile = process.argv[2] || createSyntheticImport();
const externalFile = process.argv[4] || path.join(__dirname, 'fixtures', 'external-node-route.json');
fs.mkdirSync(outputDir, { recursive: true });

const browser = spawn(browserExecutable, [
  '--headless=new', `--remote-debugging-port=${port}`, `--user-data-dir=${profile}`,
  '--disable-gpu', '--hide-scrollbars', '--no-first-run', '--window-size=1440,900',
  'http://localhost:5000/'
], { stdio: 'ignore' });

const delay = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

async function targetInfo() {
  for (let attempt = 0; attempt < 50; attempt += 1) {
    try {
      const targets = await (await fetch(`http://127.0.0.1:${port}/json/list`)).json();
      const page = targets.find((target) =>
        target.type === 'page' && target.url.startsWith('http://localhost:5000/')
      );
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
    const { resolve, reject } = pending.get(message.id);
    pending.delete(message.id);
    if (message.error) reject(new Error(JSON.stringify(message.error)));
    else resolve(message.result);
  };
  await new Promise((resolve, reject) => {
    ws.onopen = resolve;
    ws.onerror = reject;
  });
  const send = (method, params = {}) => new Promise((resolve, reject) => {
    const id = nextId++;
    pending.set(id, { resolve, reject });
    ws.send(JSON.stringify({ id, method, params }));
  });
  const evaluate = async (expression) => {
    const result = await send('Runtime.evaluate', { expression, returnByValue: true, awaitPromise: true });
    if (result.exceptionDetails) throw new Error(result.exceptionDetails.text);
    return result.result.value;
  };

  await send('Runtime.enable');
  await send('DOM.enable');
  await send('Page.enable');
  for (let attempt = 0; attempt < 50; attempt += 1) {
    const state = await evaluate('({url: location.href, ready: document.readyState})');
    if (state.url.startsWith('http://localhost:5000/') && state.ready === 'complete') break;
    await delay(100);
    if (attempt === 49) throw new Error(`application page did not load: ${JSON.stringify(state)}`);
  }
  await delay(300);

  if (importFile) {
    const fileInput = await send('Runtime.evaluate', {
      expression: 'document.querySelector("#file-input")'
    });
    if (!fileInput.result.objectId) {
      const pageState = await evaluate('({url: location.href, ready: document.readyState, title: document.title, html: document.body.innerHTML.slice(0, 200)})');
      throw new Error(`file input was not found: ${JSON.stringify(pageState)}`);
    }
    await send('DOM.setFileInputFiles', {
      objectId: fileInput.result.objectId,
      files: [path.resolve(importFile)]
    });
    await evaluate(`(() => {
      const input = document.querySelector('#file-input');
      if (input.files.length) input.dispatchEvent(new Event('change', {bubbles: true}));
    })()`);
    await delay(1500);
  }
  await evaluate('document.querySelector("#geometry-validation").click()');
  await delay(700);

  const desktop = await evaluate(`({
    options: [...document.querySelector('#geometry-algorithm').options].map(o => o.textContent),
    selected: document.querySelector('#geometry-algorithm').value,
    summary: document.querySelector('#geometry-summary').textContent,
    chartHidden: document.querySelector('#curvature-pane').hidden,
    chartSize: [document.querySelector('#curvature-canvas').width, document.querySelector('#curvature-canvas').height],
    mapSize: [document.querySelector('#canvas').width, document.querySelector('#canvas').height],
    radiusLimitChecked: document.querySelector('#curvature-limit').checked,
    requestedRadius: document.querySelector('#minimum-radius-mm').value,
    workspace: document.querySelector('.workspace-tab.active').dataset.workspace,
    planningToolsHidden: document.querySelector('#planning-tools').hidden,
    speedChartHidden: document.querySelector('#speed-pane').hidden,
    speedChartSize: [document.querySelector('#speed-canvas').width, document.querySelector('#speed-canvas').height],
    planningUploadVisible: !document.querySelector('#planning-upload-btn').hidden,
    planningUploadDisabled: document.querySelector('#planning-upload-btn').disabled,
    planningUploadStatus: document.querySelector('#planning-upload-status').textContent,
    overflow: document.documentElement.scrollWidth - document.documentElement.clientWidth
  })`);
  assert.strictEqual(desktop.selected, 'global');
  assert(desktop.options[0].includes('全局曲率样条'));
  assert(desktop.options[1].includes('分段回旋圆弧'));
  assert(desktop.summary.includes('全局曲率样条'), desktop.summary);
  assert(desktop.summary.includes('半径限制 ≥150mm：已满足'), desktop.summary);
  assert.strictEqual(desktop.radiusLimitChecked, true);
  assert.strictEqual(desktop.requestedRadius, '150');
  assert.strictEqual(desktop.chartHidden, false);
  assert.strictEqual(desktop.workspace, 'planning');
  assert.strictEqual(desktop.planningToolsHidden, false);
  assert.strictEqual(desktop.speedChartHidden, false);
  assert.strictEqual(desktop.planningUploadVisible, true);
  assert.strictEqual(desktop.planningUploadDisabled, true);
  assert(desktop.planningUploadStatus.includes('连接串口'), desktop.planningUploadStatus);
  assert(desktop.speedChartSize[0] > 100 && desktop.speedChartSize[1] > 50);
  assert(desktop.chartSize[0] > 100 && desktop.chartSize[1] > 50);
  assert.strictEqual(desktop.overflow, 0);

  const mapPixelCounts = await evaluate(`(() => {
    const canvas = document.querySelector('#canvas');
    const data = canvas.getContext('2d').getImageData(0, 0, canvas.width, canvas.height).data;
    const counts = {blue: 0, orange: 0, red: 0, green: 0};
    for (let index = 0; index < data.length; index += 4) {
      const key = data[index] + ',' + data[index + 1] + ',' + data[index + 2];
      if (key === '25,118,210') counts.blue += 1;
      else if (key === '245,124,0') counts.orange += 1;
      else if (key === '229,57,53') counts.red += 1;
      else if (key === '46,155,81') counts.green += 1;
    }
    return counts;
  })()`);
  assert(mapPixelCounts.blue > 100, JSON.stringify(mapPixelCounts));
  assert(mapPixelCounts.orange > 10, JSON.stringify(mapPixelCounts));
  assert.strictEqual(mapPixelCounts.red, 0, JSON.stringify(mapPixelCounts));
  assert.strictEqual(mapPixelCounts.green, 0, JSON.stringify(mapPixelCounts));

  const chartPixelCounts = await evaluate(`(() => {
    const canvas = document.querySelector('#curvature-canvas');
    const data = canvas.getContext('2d').getImageData(0, 0, canvas.width, canvas.height).data;
    const counts = {blue: 0, orange: 0, limit: 0};
    for (let index = 0; index < data.length; index += 4) {
      const key = data[index] + ',' + data[index + 1] + ',' + data[index + 2];
      if (key === '25,118,210') counts.blue += 1;
      else if (key === '245,124,0') counts.orange += 1;
      else if (data[index] > 150 && data[index + 1] < 100 && data[index + 2] < 100) counts.limit += 1;
    }
    return counts;
  })()`);
  assert(chartPixelCounts.blue > 0, JSON.stringify(chartPixelCounts));
  assert(chartPixelCounts.orange > 100, JSON.stringify(chartPixelCounts));
  assert(chartPixelCounts.limit > 100, JSON.stringify(chartPixelCounts));

  const speedPixelCounts = await evaluate(`(() => {
    const canvas = document.querySelector('#speed-canvas');
    const data = canvas.getContext('2d').getImageData(0, 0, canvas.width, canvas.height).data;
    const counts = {curve: 0, wheel: 0, final: 0};
    for (let index = 0; index < data.length; index += 4) {
      const key = data[index] + ',' + data[index + 1] + ',' + data[index + 2];
      if (key === '0,131,143') counts.curve += 1;
      else if (key === '194,24,91') counts.wheel += 1;
      else if (key === '46,125,50') counts.final += 1;
    }
    return counts;
  })()`);
  assert(speedPixelCounts.final > 10, JSON.stringify(speedPixelCounts));

  const hoverText = await evaluate(`(() => {
    const canvas = document.querySelector('#canvas');
    const context = canvas.getContext('2d');
    const data = context.getImageData(0, 0, canvas.width, canvas.height).data;
    const pixels = [];
    for (let index = 0; index < data.length; index += 4) {
      if (data[index] === 245 && data[index + 1] === 124 && data[index + 2] === 0) {
        pixels.push(index / 4);
      }
    }
    if (!pixels.length) return '';
    const rect = canvas.getBoundingClientRect();
    const stride = Math.max(1, Math.floor(pixels.length / 80));
    let lastText = '';
    for (let index = 0; index < pixels.length; index += stride) {
      const pixel = pixels[index];
      const x = (pixel % canvas.width) / (canvas.width / rect.width);
      const y = Math.floor(pixel / canvas.width) / (canvas.height / rect.height);
      window.dispatchEvent(new MouseEvent('mousemove', {
        bubbles: true, clientX: rect.left + x, clientY: rect.top + y
      }));
      lastText = document.querySelector('#point-tooltip').textContent;
      if (lastText.includes('曲率半径')) return lastText;
    }
    return lastText;
  })()`);
  assert(hoverText.includes('X') && hoverText.includes('Y'), hoverText);

  await evaluate(`(() => {
    const radius = document.querySelector('#minimum-radius-mm');
    radius.value = '180';
    document.querySelector('#corridor-optimizer').click();
  })()`);
  await delay(300);
  const corridorSummary = await evaluate('document.querySelector("#geometry-summary").textContent');
  assert(corridorSummary.includes('横向偏移优化'), corridorSummary);
  assert(corridorSummary.includes('半径限制 ≥180mm：已满足'), corridorSummary);

  const desktopShot = await send('Page.captureScreenshot', { format: 'png', fromSurface: true });
  fs.writeFileSync(path.join(outputDir, 'geometry-desktop.png'), Buffer.from(desktopShot.data, 'base64'));

  await evaluate('document.querySelector("#geometry-algorithm").value="legacy"; document.querySelector("#geometry-algorithm").dispatchEvent(new Event("change", {bubbles:true}))');
  await delay(300);
  const legacySummary = await evaluate('document.querySelector("#geometry-summary").textContent');
  assert(legacySummary.includes('分段回旋圆弧'), legacySummary);

  let externalState = null;
  if (externalFile) {
    if (replayImportFile) {
      const replayInput = await send('Runtime.evaluate', { expression: 'document.querySelector("#file-input")' });
      await send('DOM.setFileInputFiles', {
        objectId: replayInput.result.objectId,
        files: [path.resolve(replayImportFile)]
      });
      await evaluate(`document.querySelector('#file-input').dispatchEvent(new Event('change', {bubbles: true}))`);
      await delay(500);
      const planningProgressSamples = [];
      for (const time of [0, 0.4]) {
        await evaluate(`(() => {
          const slider = document.querySelector('#playback-slider');
          slider.value = '${time}';
          slider.dispatchEvent(new Event('input', {bubbles: true}));
        })()`);
        await delay(100);
        planningProgressSamples.push(await evaluate(
          `Number(document.querySelector('#planning-progress').value)`
        ));
      }
      assert(planningProgressSamples[1] > planningProgressSamples[0] + 500,
        `planning lookaheads did not follow replay: ${planningProgressSamples.join(',')}`);
      await evaluate(`(() => {
        const slider = document.querySelector('#playback-slider');
        slider.value = slider.max;
        slider.dispatchEvent(new Event('input', {bubbles: true}));
      })()`);
      await delay(100);
    }
    const pastedDocument = fs.readFileSync(path.resolve(externalFile), 'utf8');
    await evaluate(`(() => {
      document.querySelector('[data-workspace="external"]').click();
      document.querySelector('#external-paste-btn').click();
      document.querySelector('#external-paste-text').value = ${JSON.stringify(pastedDocument)};
      document.querySelector('#external-paste-import-btn').click();
    })()`);
    await delay(400);
    const pasteState = await evaluate(`({
      dialogOpen: document.querySelector('#external-paste-dialog').open,
      origins: document.querySelector('#external-origin').options.length,
      offsetX: document.querySelector('#external-offset-x').value,
      offsetY: document.querySelector('#external-offset-y').value,
      summary: document.querySelector('#external-summary').textContent
    })`);
    assert.strictEqual(pasteState.dialogOpen, false);
    assert.strictEqual(pasteState.origins, 4);
    assert.strictEqual(pasteState.offsetX, '0');
    assert.strictEqual(pasteState.offsetY, '0');
    assert(pasteState.summary.includes('5mm 折线 511 点'), pasteState.summary);
    assert(pasteState.summary.includes('5mm 切弯 475 点'), pasteState.summary);
    assert(pasteState.summary.includes('起点 (0, 0) mm'), pasteState.summary);
    const offsetState = await evaluate(`(() => {
      const x = document.querySelector('#external-offset-x');
      const y = document.querySelector('#external-offset-y');
      x.value = '20';
      y.value = '-30';
      x.dispatchEvent(new Event('input', {bubbles: true}));
      y.dispatchEvent(new Event('input', {bubbles: true}));
      return document.querySelector('#external-summary').textContent;
    })()`);
    assert(offsetState.includes('起点 (20, -30) mm'), offsetState);
    assert(offsetState.includes('偏置 (20, -30) mm'), offsetState);

    const externalInput = await send('Runtime.evaluate', { expression: 'document.querySelector("#external-file-input")' });
    await send('DOM.setFileInputFiles', {
      objectId: externalInput.result.objectId,
      files: [path.resolve(externalFile)]
    });
    await evaluate(`(() => {
      const input = document.querySelector('#external-file-input');
      input.dispatchEvent(new Event('change', {bubbles: true}));
    })()`);
    await delay(500);
    await evaluate(`(() => {
      const rotation = document.querySelector('#external-rotation');
      rotation.value = '-90';
      rotation.dispatchEvent(new Event('change', {bubbles: true}));
    })()`);
    await delay(300);
    externalState = await evaluate(`({
      workspace: document.querySelector('.workspace-tab.active').dataset.workspace,
      toolsHidden: document.querySelector('#external-tools').hidden,
      origins: document.querySelector('#external-origin').options.length,
      summary: document.querySelector('#external-summary').textContent,
      allLayerSwitchesVisible: [
        'show-predefined', 'show-geometry-plan', 'show-firmware-plan',
        'show-external-route', 'show-external-network', 'show-external-segments', 'show-realtime', 'show-lookahead'
      ].every(id => !document.querySelector('#' + id).closest('label').hidden),
      graphitePixels: (() => {
        const canvas = document.querySelector('#canvas');
        const pixels = canvas.getContext('2d').getImageData(0, 0, canvas.width, canvas.height).data;
        let count = 0;
        for (let index = 0; index < pixels.length; index += 4) {
          if (pixels[index] === 69 && pixels[index + 1] === 90 && pixels[index + 2] === 100) count += 1;
        }
        return count;
      })(),
    uploadVisible: !document.querySelector('#external-upload-btn').hidden,
    uploadDisabled: document.querySelector('#external-upload-btn').disabled,
    uploadStatus: document.querySelector('#external-upload-status').textContent,
    overflow: document.documentElement.scrollWidth - document.documentElement.clientWidth
    })`);
    assert.strictEqual(externalState.workspace, 'external');
    assert.strictEqual(externalState.toolsHidden, false);
    assert.strictEqual(externalState.origins, 4);
    assert.strictEqual(externalState.allLayerSwitchesVisible, true);
    assert.strictEqual(externalState.uploadVisible, true);
    assert.strictEqual(externalState.uploadDisabled, true);
    assert(externalState.uploadStatus.includes('连接串口'), externalState.uploadStatus);
    assert(externalState.graphitePixels > 10, JSON.stringify(externalState));
    assert(externalState.summary.includes('5mm 折线 511 点'), externalState.summary);
    assert(externalState.summary.includes('路肩最小净距 ≥150 mm · 宽 40 mm'), externalState.summary);
    assert.strictEqual(externalState.overflow, 0);
    const vehicleDefaults = await evaluate(`({
      length: document.querySelector('#vehicle-length-mm').value,
      width: document.querySelector('#vehicle-width-mm').value,
      pivot: document.querySelector('#vehicle-pivot-mm').value,
      panelHidden: document.querySelector('#vehicle-sim-panel').hidden,
      state: document.querySelector('#vehicle-sim-state').textContent
    })`);
    assert.deepStrictEqual(
      [vehicleDefaults.length, vehicleDefaults.width, vehicleDefaults.pivot],
      ['150', '140', '50']
    );
    assert.strictEqual(vehicleDefaults.panelHidden, true);
    const simulationLookaheadDefaults = await evaluate(`({
      open: document.querySelector('#simulation-lookahead-config').open,
      enabled: document.querySelector('#show-simulation-lookahead').checked,
      steering: document.querySelector('#simulation-steering-lookahead-mm').value,
      tangent: document.querySelector('#simulation-tangent-distance-mm').value,
      speed: document.querySelector('#simulation-speed-preview-mm').value,
      source: document.querySelector('#simulation-lookahead-source').textContent
    })`);
    assert.strictEqual(simulationLookaheadDefaults.open, false);
    assert.strictEqual(simulationLookaheadDefaults.enabled, true);
    assert.deepStrictEqual(
      [simulationLookaheadDefaults.steering, simulationLookaheadDefaults.tangent, simulationLookaheadDefaults.speed],
      ['400', '50', '100']
    );
    assert(simulationLookaheadDefaults.source.includes('模拟'), JSON.stringify(simulationLookaheadDefaults));
    await evaluate(`(() => {
      const width = document.querySelector('#vehicle-width-mm');
      width.value = '840';
      width.dispatchEvent(new Event('change', {bubbles: true}));
      const enabled = document.querySelector('#vehicle-sim-enabled');
      enabled.checked = true;
      enabled.dispatchEvent(new Event('change', {bubbles: true}));
    })()`);
    await delay(150);
    const collisionState = await evaluate(`({
      state: document.querySelector('#vehicle-sim-state').textContent,
      events: document.querySelectorAll('#vehicle-collision-events li').length,
      detail: document.querySelector('#vehicle-sim-detail').textContent
    })`);
    if (collisionState.state === '碰撞路肩') {
      assert(collisionState.events >= 1, JSON.stringify(collisionState));
      assert(collisionState.detail.includes('路肩重叠'), collisionState.detail);
    } else {
      assert.strictEqual(collisionState.state, '仿真暂停');
      assert(collisionState.detail.includes('车体范围安全'), collisionState.detail);
    }
    await evaluate(`(() => {
      const width = document.querySelector('#vehicle-width-mm');
      width.value = '140';
      width.dispatchEvent(new Event('change', {bubbles: true}));
      document.querySelector('#playback-reset-btn').click();
      document.querySelector('#playback-toggle-btn').click();
    })()`);
    await delay(250);
    const movingState = await evaluate(`({
      enabled: document.querySelector('#vehicle-sim-enabled').checked,
      playback: document.querySelector('#playback-toggle-btn').textContent,
      detail: document.querySelector('#vehicle-sim-detail').textContent
    })`);
    assert.strictEqual(movingState.enabled, true);
    assert.strictEqual(movingState.playback, '暂停');
    assert(!movingState.detail.includes('· 00:00.000 /'), movingState.detail);
    await evaluate(`document.querySelector('#playback-toggle-btn').click()`);
    await evaluate(`(() => {
      const source = document.querySelector('#vehicle-sim-source');
      source.value = 'external-segments';
      source.dispatchEvent(new Event('change', {bubbles: true}));
      document.querySelector('#vehicle-sim-use-speed').checked = true;
      document.querySelector('#vehicle-sim-fixed-speed').value = '777';
      document.querySelector('#vehicle-sim-play').click();
    })()`);
    await delay(250);
    const plannedPathSimulation = await evaluate(`(() => {
      const slider = document.querySelector('#vehicle-sim-progress');
      const detail = document.querySelector('#vehicle-sim-detail').textContent;
      const canvas = document.querySelector('#canvas');
      const pixels = canvas.getContext('2d').getImageData(0, 0, canvas.width, canvas.height).data;
      const colors = {green: 0, orange: 0, purple: 0};
      for (let index = 0; index < pixels.length; index += 4) {
        if (pixels[index + 1] > pixels[index] * 1.45 &&
            pixels[index + 1] > pixels[index + 2] * 1.15 &&
            pixels[index + 1] > 85) colors.green += 1;
        if (pixels[index] === 245 && pixels[index + 1] === 124 && pixels[index + 2] === 0) colors.orange += 1;
        if (pixels[index] === 126 && pixels[index + 1] === 87 && pixels[index + 2] === 194) colors.purple += 1;
      }
      return {
        detail,
        disabled: slider.disabled,
        max: Number(slider.max),
        value: Number(slider.value),
        label: document.querySelector('#vehicle-sim-progress-label').textContent,
        colors
      };
    })()`);
    assert(plannedPathSimulation.detail.includes('外部切弯路径'), plannedPathSimulation.detail);
    assert(/1[25]00 mm\/s/.test(plannedPathSimulation.detail), plannedPathSimulation.detail);
    assert(plannedPathSimulation.max > 0, JSON.stringify(plannedPathSimulation));
    assert.strictEqual(plannedPathSimulation.disabled, false);
    assert(plannedPathSimulation.label.includes(' / '), plannedPathSimulation.label);
    assert(plannedPathSimulation.colors.green > 0, JSON.stringify(plannedPathSimulation.colors));
    assert(plannedPathSimulation.colors.orange > 0, JSON.stringify(plannedPathSimulation.colors));
    assert(plannedPathSimulation.colors.purple > 0, JSON.stringify(plannedPathSimulation.colors));
    const editedSimulationLookahead = await evaluate(`(() => {
      const details = document.querySelector('#simulation-lookahead-config');
      details.open = true;
      const steering = document.querySelector('#simulation-steering-lookahead-mm');
      steering.value = '450';
      steering.dispatchEvent(new Event('input', {bubbles: true}));
      return {
        open: details.open,
        source: document.querySelector('#simulation-lookahead-source').textContent
      };
    })()`);
    assert.strictEqual(editedSimulationLookahead.open, true);
    assert(editedSimulationLookahead.source.includes('450/50/100'), JSON.stringify(editedSimulationLookahead));
    const hiddenSimulationLookahead = await evaluate(`(() => {
      const toggle = document.querySelector('#show-simulation-lookahead');
      toggle.checked = false;
      toggle.dispatchEvent(new Event('change', {bubbles: true}));
      return toggle.checked;
    })()`);
    assert.strictEqual(hiddenSimulationLookahead, false);
    await delay(100);
    await evaluate(`(() => {
      const toggle = document.querySelector('#show-simulation-lookahead');
      toggle.checked = true;
      toggle.dispatchEvent(new Event('change', {bubbles: true}));
    })()`);
    const seekState = await evaluate(`(() => {
      const slider = document.querySelector('#vehicle-sim-progress');
      slider.value = String(Number(slider.max) * 0.5);
      slider.dispatchEvent(new Event('input', {bubbles: true}));
      return {
        value: Number(slider.value),
        label: document.querySelector('#vehicle-sim-progress-label').textContent,
        detail: document.querySelector('#vehicle-sim-detail').textContent,
        button: document.querySelector('#vehicle-sim-play').textContent
      };
    })()`);
    assert(Math.abs(seekState.value - plannedPathSimulation.max * 0.5) < 0.01, JSON.stringify(seekState));
    assert(seekState.detail.includes(' / '), seekState.detail);
    assert.strictEqual(seekState.button, '继续');
    await evaluate(`(() => {
      document.querySelector('#vehicle-sim-play').click();
      const source = document.querySelector('#vehicle-sim-source');
      source.value = 'external-nodes';
      source.dispatchEvent(new Event('change', {bubbles: true}));
      document.querySelector('#vehicle-sim-play').click();
    })()`);
    await delay(200);
    const fallbackPathSimulation = await evaluate(`document.querySelector('#vehicle-sim-detail').textContent`);
    assert(fallbackPathSimulation.includes('外部节点路径'), fallbackPathSimulation);
    assert(fallbackPathSimulation.includes('777 mm/s'), fallbackPathSimulation);
    await evaluate(`(() => {
      document.querySelector('#vehicle-sim-play').click();
      const source = document.querySelector('#vehicle-sim-source');
      source.value = 'replay';
      source.dispatchEvent(new Event('change', {bubbles: true}));
    })()`);
    await evaluate(`(() => {
      ['show-geometry-plan', 'show-firmware-plan', 'show-external-route',
       'show-external-segments', 'show-external-shoulders', 'show-realtime', 'show-lookahead']
        .forEach(id => {
          const input = document.querySelector('#' + id);
          input.checked = false;
          input.dispatchEvent(new Event('change', {bubbles: true}));
        });
    })()`);
    const hoverByWorkspace = {};
    for (const mode of ['live', 'planning', 'external']) {
      await evaluate(`document.querySelector('[data-workspace="${mode}"]').click()`);
      await delay(250);
      await evaluate('document.querySelector("#auto-fit-btn").click()');
      await delay(300);
      hoverByWorkspace[mode] = await evaluate(`(() => {
        const canvas = document.querySelector('#canvas');
        const context = canvas.getContext('2d');
        const data = context.getImageData(0, 0, canvas.width, canvas.height).data;
        const rect = canvas.getBoundingClientRect();
        for (let index = 0; index < data.length; index += 4) {
          if (data[index] !== 25 || data[index + 1] !== 118 || data[index + 2] !== 210) continue;
          const pixel = index / 4;
          const x = (pixel % canvas.width) / (canvas.width / rect.width);
          const y = Math.floor(pixel / canvas.width) / (canvas.height / rect.height);
          window.dispatchEvent(new MouseEvent('mousemove', {
            bubbles: true, clientX: rect.left + x, clientY: rect.top + y
          }));
          const text = document.querySelector('#point-tooltip').textContent;
          if (text.includes('录制路径点')) return text;
        }
        return '';
      })()`);
      assert(hoverByWorkspace[mode].includes('X') && hoverByWorkspace[mode].includes('Y'),
        `${mode}: ${hoverByWorkspace[mode]}`);
    }
    await evaluate(`(() => {
      ['show-geometry-plan', 'show-firmware-plan', 'show-external-route',
       'show-external-segments', 'show-external-shoulders', 'show-realtime', 'show-lookahead']
        .forEach(id => {
          const input = document.querySelector('#' + id);
          input.checked = true;
          input.dispatchEvent(new Event('change', {bubbles: true}));
        });
      document.querySelector('[data-workspace="external"]').click();
    })()`);
    await delay(300);
    const changedOrigin = await evaluate(`(() => {
      const origin = document.querySelector('#external-origin');
      origin.value = '2';
      origin.dispatchEvent(new Event('change', {bubbles: true}));
      return {
        x: document.querySelector('#external-offset-x').value,
        y: document.querySelector('#external-offset-y').value,
        summary: document.querySelector('#external-summary').textContent
      };
    })()`);
    assert.strictEqual(changedOrigin.x, '0');
    assert.strictEqual(changedOrigin.y, '0');
    assert(changedOrigin.summary.includes('起点 (0, 0) mm'), changedOrigin.summary);
    await evaluate(`(() => {
      const origin = document.querySelector('#external-origin');
      origin.value = '1';
      origin.dispatchEvent(new Event('change', {bubbles: true}));
    })()`);
    await delay(200);
    const recordedVisibility = await evaluate(`(() => {
      const canvas = document.querySelector('#canvas');
      const countBlue = () => {
        const data = canvas.getContext('2d').getImageData(0, 0, canvas.width, canvas.height).data;
        let count = 0;
        for (let index = 0; index < data.length; index += 4) {
          if (data[index] === 25 && data[index + 1] === 118 && data[index + 2] === 210) count += 1;
        }
        return count;
      };
      const checkbox = document.querySelector('#show-predefined');
      const geometry = document.querySelector('#show-geometry-plan');
      const firmware = document.querySelector('#show-firmware-plan');
      geometry.checked = false;
      firmware.checked = false;
      geometry.dispatchEvent(new Event('change', {bubbles: true}));
      firmware.dispatchEvent(new Event('change', {bubbles: true}));
      return new Promise((resolve) => requestAnimationFrame(() => requestAnimationFrame(() => {
        const shown = countBlue();
        checkbox.checked = false;
        checkbox.dispatchEvent(new Event('change', {bubbles: true}));
        requestAnimationFrame(() => requestAnimationFrame(() => {
          const hidden = countBlue();
          checkbox.checked = true;
          geometry.checked = true;
          firmware.checked = true;
          checkbox.dispatchEvent(new Event('change', {bubbles: true}));
          geometry.dispatchEvent(new Event('change', {bubbles: true}));
          firmware.dispatchEvent(new Event('change', {bubbles: true}));
          resolve({shown, hidden});
        }));
      })));
    })()`);
    assert(recordedVisibility.shown > recordedVisibility.hidden, JSON.stringify(recordedVisibility));
    await delay(100);
    const externalShot = await send('Page.captureScreenshot', { format: 'png', fromSurface: true });
    fs.writeFileSync(path.join(outputDir, 'external-desktop.png'), Buffer.from(externalShot.data, 'base64'));
    await evaluate('document.querySelector("#external-use-raw-btn").click()');
    await delay(700);
    const liveRoundTrip = await evaluate(`(() => {
      document.querySelector('[data-workspace="live"]').click();
      return {
        workspace: document.querySelector('.workspace-tab.active').dataset.workspace,
        geometryChecked: document.querySelector('#geometry-validation').checked,
        chartHidden: document.querySelector('#curvature-pane').hidden
      };
    })()`);
    assert.deepStrictEqual(liveRoundTrip, { workspace: 'live', geometryChecked: false, chartHidden: true });
    await evaluate('document.querySelector(\'[data-workspace="planning"]\').click()');
    await delay(500);
  }

  await send('Emulation.setDeviceMetricsOverride', {
    width: 390, height: 844, deviceScaleFactor: 1, mobile: true
  });
  await delay(300);
  await evaluate('document.querySelector("#geometry-algorithm").value="global"; document.querySelector("#geometry-algorithm").dispatchEvent(new Event("change", {bubbles:true}))');
  await delay(700);
  await evaluate('document.querySelector("#auto-fit-btn").click()');
  await delay(200);
  const mobile = await evaluate(`({
    overflow: document.documentElement.scrollWidth - document.documentElement.clientWidth,
    selectorRight: document.querySelector('#planning-algorithm').getBoundingClientRect().right,
    viewport: innerWidth,
    chart: document.querySelector('#curvature-pane').getBoundingClientRect().toJSON(),
    map: document.querySelector('#canvas-container').getBoundingClientRect().toJSON()
  })`);
  assert.strictEqual(mobile.overflow, 0);
  assert(mobile.selectorRight <= mobile.viewport + 0.5);
  assert(mobile.chart.height > 90, JSON.stringify(mobile));
  assert(mobile.map.height > 250, JSON.stringify(mobile));
  const mobileShot = await send('Page.captureScreenshot', { format: 'png', fromSurface: true });
  fs.writeFileSync(path.join(outputDir, 'geometry-mobile.png'), Buffer.from(mobileShot.data, 'base64'));

  console.log(JSON.stringify({
    desktop, mapPixelCounts, chartPixelCounts, speedPixelCounts, hoverText, corridorSummary,
    legacySummary, externalState, mobile, outputDir
  }, null, 2));
  await send('Browser.close');
}

main().finally(() => {
  browser.kill();
}).catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
