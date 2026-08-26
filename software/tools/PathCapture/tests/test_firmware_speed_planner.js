'use strict';

const assert = require('assert');
const planner = require('../static/firmware-speed-planner.js');

const straight = Array.from({ length: 201 }, (_, index) => ({ x: index * 5, y: 0, curvature_per_mm: 0 }));
const result = planner.plan(straight);
assert.strictEqual(result.points.length, straight.length);
assert.strictEqual(result.totalDistanceMm, 1000);
assert.strictEqual(result.points[0].planned_speed_mm_s, 500);
assert.strictEqual(result.points.at(-1).planned_speed_mm_s, 0);
assert(result.points[20].planned_speed_mm_s <= 500);
assert(result.points[100].planned_speed_mm_s > 2000);
assert(result.points[170].planned_speed_mm_s < result.points[150].planned_speed_mm_s);

const radiusMm = 150;
const arcStepRad = 5 / radiusMm;
const curve = Array.from({ length: 121 }, (_, index) => ({
  x: radiusMm * Math.sin(index * arcStepRad),
  y: radiusMm * (1 - Math.cos(index * arcStepRad)),
  // The MCU derives curvature from coordinates and must ignore external annotations.
  curvature_per_mm: 0
}));
const curveResult = planner.plan(curve);
const center = curveResult.points[60];
assert(Math.abs(center.curvature_per_mm - 1 / radiusMm) < 1e-9);
assert(Math.abs(center.curvature_speed_cap_mm_s - Math.sqrt(25000 * radiusMm)) < 1e-6);
assert(center.wheel_speed_cap_mm_s < 2600);
assert(center.planned_speed_mm_s <= center.wheel_speed_cap_mm_s);
assert(curveResult.series.finalSpeedMmS.every(Number.isFinite));
assert(planner.availableLongitudinalAccel(2000, 1 / 200, false, planner.DEFAULTS) < 25000);
assert.strictEqual(planner.endScale(10, planner.DEFAULTS), 0);
assert.strictEqual(planner.endScale(300, planner.DEFAULTS), 1);

const previewCurve = planner.normalizePath(curve);
assert(Math.abs(previewCurve[60].curvature_per_mm - planner.curvatureAt(previewCurve, 60, 20)) < 1e-12);
assert.strictEqual(planner.DEFAULTS.curvatureGapPoints, 20);

const malformed = planner.plan([{ x: 0, y: 0 }, { x: 'bad', y: 2 }, { x: 5, y: 0 }]);
assert.strictEqual(malformed.points.length, 2);

console.log(JSON.stringify({ straight: result.points.length, curveSpeed: center.planned_speed_mm_s }));
