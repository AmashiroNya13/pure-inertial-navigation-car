'use strict';

const assert = require('assert');
const { spawn } = require('child_process');
const fs = require('fs');
const os = require('os');
const path = require('path');

const browserExecutable = 'C:\\Program Files\\Google\\Chrome\\Application\\chrome.exe';
const port = 9900 + Math.floor(Math.random() * 80);
const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'amr-path-viewer-'));
const pageUrl = `file:///${path.resolve(__dirname, 'amr_path_viewer.html').replace(/\\/g, '/')}`;
const browser = spawn(browserExecutable, [
  '--headless=new', `--remote-debugging-port=${port}`, `--user-data-dir=${profile}`,
  '--disable-gpu', '--no-first-run', '--window-size=1280,900', pageUrl,
], { stdio: 'ignore' });

const delay = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

async function main() {
  let target;
  for (let attempt = 0; attempt < 50; attempt += 1) {
    try {
      const targets = await (await fetch(`http://127.0.0.1:${port}/json/list`)).json();
      target = targets.find((item) => item.type === 'page' && item.url.startsWith('file:'));
      if (target) break;
    } catch (_) {
      await delay(100);
    }
  }
  assert(target, 'Chrome DevTools endpoint did not start');
  const socket = new WebSocket(target.webSocketDebuggerUrl);
  const pending = new Map();
  let nextId = 1;
  socket.onmessage = (event) => {
    const message = JSON.parse(event.data);
    if (!message.id || !pending.has(message.id)) return;
    const request = pending.get(message.id);
    pending.delete(message.id);
    if (message.error) request.reject(new Error(JSON.stringify(message.error)));
    else request.resolve(message.result);
  };
  await new Promise((resolve, reject) => { socket.onopen = resolve; socket.onerror = reject; });
  const send = (method, params = {}) => new Promise((resolve, reject) => {
    const id = nextId++;
    pending.set(id, { resolve, reject });
    socket.send(JSON.stringify({ id, method, params }));
  });
  const evaluate = async (expression) => {
    const response = await send('Runtime.evaluate', { expression, returnByValue: true, awaitPromise: true });
    if (response.exceptionDetails) throw new Error(JSON.stringify(response.exceptionDetails));
    return response.result.value;
  };
  await send('Runtime.enable');
  for (let attempt = 0; attempt < 50; attempt += 1) {
    if (await evaluate('document.readyState === "complete" && typeof buildDataSets === "function"')) break;
    if (attempt === 49) throw new Error('viewer page did not finish loading');
    await delay(100);
  }
  const result = await evaluate(`(() => {
    const log = [
      '{path}0,0,0,0,0,0,0,0,0,0,0,0,0,0,0',
      '{path}5,0,0,1,5,5,5,1,1,0,0,1,1,1,1',
      '{path}10,0,0,2,10,10,10,2,2,0,0,2,2,2,2',
      '{vslipevt}1,1,5,0,0.70,10,7,3,3',
      '{vslipacc}1,1,2,10,0,10000,5000,0.5',
      '{ptcorr}1,1,1,1,0,0.9,100,5,0,1,5,0,0,0,0,1,2,1,1,3,0,0,2,5,1,0,5,0,1,1,1,1',
      '{ptcorrseg}1,2,5,0,10,0,1.5,0,0.4,8,0.9',
    ].join('\\n');
    const parsed = buildDataSets(log, 'synthetic.dat');
    state.fileName = parsed.fileName;
    state.dataSets = parsed.dataSets;
    state.phototube = parsed.phototube;
    state.events = parsed.events;
    state.selectedKey = '';
    render();
    const initial = document.querySelectorAll('#eventLayers circle').length;
    document.querySelector('#showTurnSlip').click();
    const withoutTurn = document.querySelectorAll('#eventLayers circle').length;
    document.querySelector('#showAccelSlip').click();
    const onlyPhoto = document.querySelectorAll('#eventLayers circle').length;
    const photoSegments = document.querySelectorAll('#phototubeCorrectionSegments line').length;
    const photoLabels = document.querySelectorAll('#phototubeCorrectionSegments text').length;
    return { initial, withoutTurn, onlyPhoto, photoSegments, photoLabels, counts: Object.fromEntries(Object.entries(parsed.events).map(([key, value]) => [key, value.length])) };
  })()`);
  assert.deepStrictEqual(result.counts, { turnSlip: 1, accelSlip: 1, phototube: 2 });
  assert.strictEqual(result.initial, 4);
  assert.strictEqual(result.withoutTurn, 3);
  assert.strictEqual(result.onlyPhoto, 2);
  assert.strictEqual(result.photoSegments, 1);
  assert.strictEqual(result.photoLabels, 1);
  socket.close();
}

async function cleanup() {
  browser.kill();
  await delay(300);
  try { fs.rmSync(profile, { recursive: true, force: true, maxRetries: 5, retryDelay: 100 }); } catch (_) { /* OS cleanup only. */ }
}

main()
  .then(cleanup)
  .catch(async (error) => { await cleanup(); console.error(error); process.exitCode = 1; });
