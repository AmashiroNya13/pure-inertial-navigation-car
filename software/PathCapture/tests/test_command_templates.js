'use strict';

const assert = require('assert');
const fs = require('fs');
const path = require('path');

const html = fs.readFileSync(path.join(__dirname, '..', 'static', 'index.html'), 'utf8');

for (const command of [
  'vrspd 0.8',
  'vpath lookahead 280',
  'vpath tangent 50',
  'vpath speedlookahead 700',
  'vangpid 1500 0 40',
  'vpid 5 10 0',
  'vpid l 5 10 0',
  'vpid r 5 10 0',
  'vpid status',
  'vload 3000',
  'vload 3000 3000',
  'vload stop',
  'vload ff on',
  'vload ff off',
  'vload ff status',
  'vspd 3000',
  'vspd 0',
  'vaff 0 0 3000',
  'vaff status',
  'vang2d 10 40 12 25',
  'vang2d status',
  'vang2d on',
  'vang2d off',
  'vpath mark on',
  'vpath mark off',
  'vpath mark status',
  'ptcal drive 0.5',
  'ptcal drive status',
  'ptcal drive abort'
]) {
  assert(html.includes(`value="${command}"`), `missing command template: ${command}`);
}

console.log('command template checks passed');
