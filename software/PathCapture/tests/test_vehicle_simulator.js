'use strict';

const assert = require('assert');
const simulator = require('../static/vehicle-simulator.js');

const path = simulator.preparePath([{ x: 0, y: 0 }, { x: 1000, y: 0 }]);
assert.strictEqual(path.totalMm, 1000);
const pose = simulator.poseAtDistance(path, 400);
assert.strictEqual(pose.x, 400);
assert.strictEqual(pose.y, 0);
assert(Math.abs(pose.headingRad) < 1e-9);
assert.strictEqual(path.totalSec, 1000 / 1200);
assert(Math.abs(simulator.poseAtTime(path, path.totalSec / 2).x - 500) < 1e-9);
assert.strictEqual(simulator.poseAtTime(path, path.totalSec).x, 1000);
assert(Math.abs(simulator.pointAtTime(path, path.totalSec / 2).s_mm - 500) < 1e-9);

const straightWithCorner = simulator.preparePath([
  { x: 0, y: 0 }, { x: 200, y: 0 }, { x: 400, y: 0 }, { x: 600, y: 0 },
  { x: 800, y: 0 }, { x: 800, y: 200 }, { x: 800, y: 400 }
]);
const straightSections = simulator.straightSections(straightWithCorner, {
  windowMm: 30,
  minimumLengthMm: 100
});
assert(straightSections.length >= 1);
assert.strictEqual(straightSections[0].startMm, 0);
assert(straightSections[0].endMm < straightWithCorner.totalMm);

const cornerThenStraight = simulator.preparePath([
  { x: 0, y: 0 }, { x: 50, y: 10 }, { x: 90, y: 40 }, { x: 110, y: 90 },
  { x: 110, y: 290 }, { x: 110, y: 490 }, { x: 110, y: 690 }
]);
const laterStraight = simulator.straightSections(cornerThenStraight, {
  windowMm: 30,
  minimumLengthMm: 100
});
assert(laterStraight.length >= 1);
assert(laterStraight[0].startMm > 0);

const speedPath = simulator.preparePath([
  { x: 0, y: 0, planned_speed_mm_s: 500 },
  { x: 500, y: 0, planned_speed_mm_s: 500 },
  { x: 1000, y: 0, planned_speed_mm_s: 1500 }
], { usePlannedSpeed: true, fixedSpeedMmS: 900 });
assert.strictEqual(speedPath.hasPlannedSpeed, true);
assert(Math.abs(speedPath.totalSec - 1.5) < 1e-9);
const fixedPath = simulator.preparePath(speedPath.points, { usePlannedSpeed: false, fixedSpeedMmS: 1000 });
assert.strictEqual(fixedPath.hasPlannedSpeed, false);
assert(Math.abs(fixedPath.totalSec - 1) < 1e-9);

const timedReplay = simulator.preparePath([
  { x: 0, y: 0, elapsed_s: 12.5 },
  { x: 500, y: 0, elapsed_s: 12.7 },
  { x: 1000, y: 0, elapsed_s: 13.0 }
], { useElapsedTime: true });
assert.strictEqual(timedReplay.timedBySource, true);
assert(Math.abs(timedReplay.totalSec - 0.5) < 1e-9);
assert(Math.abs(simulator.poseAtTime(timedReplay, 0.2).x - 500) < 1e-9);

const body = simulator.footprint(pose, {
  lengthMm: 150,
  widthMm: 140,
  pivotFromRearMm: 50
});
assert.deepStrictEqual(body.map((point) => [point.x, point.y]), [
  [500, 70], [500, -70], [350, -70], [350, 70]
]);
assert.strictEqual(
  body.reduce((sum, point) => sum + point.x, 0) / body.length,
  pose.x + 25,
  '150 mm body with a 50 mm rear pivot must place its geometric center 25 mm ahead of the motion center'
);

const track = [{ from: { x: 0, y: 0 }, to: { x: 1000, y: 0 } }];
const safe = simulator.collision(pose, { lengthMm: 150, widthMm: 140, pivotFromRearMm: 50 }, track, 150);
assert.strictEqual(safe.collided, false);
assert(Math.abs(safe.maxDistanceMm - 70) < 1e-9);
const offsetPose = { ...pose, y: 90 };
const hit = simulator.collision(offsetPose, { lengthMm: 150, widthMm: 140, pivotFromRearMm: 50 }, track, 150);
assert.strictEqual(hit.collided, true);
assert(Math.abs(hit.intrusionMm - 10) < 1e-9);
assert.strictEqual(simulator.pointSegmentDistance({ x: 1100, y: 0 }, track[0]), 100);

const shoulder = [{ from: { x: 0, y: 100 }, to: { x: 1000, y: 100 } }];
const shoulderHit = simulator.collisionWithShoulders(
  pose,
  { lengthMm: 150, widthMm: 140, pivotFromRearMm: 50 },
  shoulder,
  40
);
assert.strictEqual(shoulderHit.collided, false, '30 mm body-to-center gap must clear a 20 mm half-width shoulder');
const touchingPose = { ...pose, y: 15 };
assert.strictEqual(
  simulator.collisionWithShoulders(
    touchingPose,
    { lengthMm: 150, widthMm: 140, pivotFromRearMm: 50 },
    shoulder,
    40
  ).collided,
  true
);

console.log(JSON.stringify({ safe: safe.maxDistanceMm, intrusion: hit.intrusionMm }));
