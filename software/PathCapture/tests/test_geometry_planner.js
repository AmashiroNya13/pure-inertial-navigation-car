'use strict';

const assert = require('assert');
const planner = require('../static/geometry-planner.js');

function noisyPoint(x, y, index) {
  const noise = Math.sin(index * 1.73) * 1.2;
  return { x: x + noise * 0.35, y: y + noise };
}

function syntheticRightAngle() {
  const points = [];
  let index = 0;
  for (let x = 0; x <= 350; x += 5) points.push(noisyPoint(x, 0, index++));
  for (let angle = -Math.PI / 2; angle <= 0.0001; angle += 5 / 150) {
    points.push(noisyPoint(350 + 150 * Math.cos(angle), 150 + 150 * Math.sin(angle), index++));
  }
  for (let y = 155; y <= 650; y += 5) points.push(noisyPoint(500, y, index++));
  return points;
}

function integrateCurvature(profile, lengthMm = 1100, stepMm = 5) {
  const points = [{ x: 0, y: 0 }];
  let heading = 0;
  for (let distance = stepMm; distance <= lengthMm; distance += stepMm) {
    const curvature = profile(distance - stepMm * 0.5, lengthMm);
    heading += curvature * stepMm;
    const previous = points[points.length - 1];
    points.push(noisyPoint(
      previous.x + Math.cos(heading) * stepMm,
      previous.y + Math.sin(heading) * stepMm,
      points.length
    ));
  }
  return points;
}

function curvatureStats(points) {
  const values = points.map((point) => point.curvature_per_mm).filter(Number.isFinite);
  let maxJump = 0;
  for (let index = 1; index < values.length; index += 1) {
    maxJump = Math.max(maxJump, Math.abs(values[index] - values[index - 1]));
  }
  return { maxJump, maximum: Math.max(...values), minimum: Math.min(...values) };
}

function assertGlobalResult(name, input, options = {}) {
  const result = planner.planGlobal(input, options);
  assert.strictEqual(result.algorithm, 'global_spline');
  assert.strictEqual(result.points.length, result.source.length, `${name}: output length changed`);
  assert(result.points.every((point) => Number.isFinite(point.x) && Number.isFinite(point.y)),
    `${name}: non-finite point`);
  assert(result.points.every((point) => Number.isFinite(point.curvature_per_mm)),
    `${name}: non-finite curvature`);
  assert(result.stats.maxOffsetMm <= result.options.maxGeometryOffsetMm + 1e-6,
    `${name}: offset ${result.stats.maxOffsetMm} exceeds guard`);
  assert(result.stats.solverResidual < 1e-4, `${name}: solver residual ${result.stats.solverResidual}`);
  assert(result.stats.iterations <= result.options.globalMaxIterations);
  assert(Math.hypot(
    result.points[0].x - result.source[0].x,
    result.points[0].y - result.source[0].y
  ) < 1, `${name}: start point moved`);
  assert(Math.hypot(
    result.points.at(-1).x - result.source.at(-1).x,
    result.points.at(-1).y - result.source.at(-1).y
  ) < 1, `${name}: end point moved`);
  return result;
}

const result = planner.plan(syntheticRightAngle());
assert(result.stats.lineCount >= 2, `expected two lines, got ${result.stats.lineCount}`);
assert(result.stats.cornerCount >= 1, `expected a reconstructed corner, got ${result.stats.cornerCount}`);
assert(result.points.some((point) => point.geometryType === 'clothoid_in'));
assert(result.points.some((point) => point.geometryType === 'arc'));
assert(result.points.some((point) => point.geometryType === 'clothoid_out'));
const corner = result.corners.find((item) => item.valid);
assert(corner.radiusMm >= result.options.minRadiusMm);
assert(Math.abs(corner.turnAngleDeg - 90) < 5, `unexpected angle ${corner.turnAngleDeg}`);
assert(corner.endpointErrorMm < 1);
assert(result.stats.maxOffsetMm < 20, `unexpected synthetic offset ${result.stats.maxOffsetMm}`);

const globalRightAngle = assertGlobalResult('right angle', syntheticRightAngle());
assert(globalRightAngle.stats.lineCount >= 2);

const compoundBend = integrateCurvature((distance) => {
  if (distance < 220 || distance > 900) return 0;
  const phase = (distance - 220) / 680;
  return 0.004 + 0.0025 * Math.sin(phase * Math.PI * 2);
});
const globalCompound = assertGlobalResult('compound bend', compoundBend);
assert(curvatureStats(globalCompound.points).minimum > -0.003,
  'compound bend should remain predominantly one direction');

const sBend = integrateCurvature((distance) => {
  if (distance < 180 || distance > 920) return 0;
  return 0.006 * Math.sin(((distance - 180) / 740) * Math.PI * 2);
});
const globalS = assertGlobalResult('S bend', sBend);
const sStats = curvatureStats(globalS.points);
assert(sStats.minimum < -0.001 && sStats.maximum > 0.001,
  `S bend lost a turn direction: ${JSON.stringify(sStats)}`);
assert(sStats.maxJump < 0.002, `S bend curvature jump is too large: ${sStats.maxJump}`);

const guarded = planner.planGlobal(syntheticRightAngle(), { maxGeometryOffsetMm: 0.05 });
assert.strictEqual(guarded.stats.rejected, true, 'strict offset guard should reject the spline');
assert(guarded.stats.rejectedOffsetMm > guarded.options.maxGeometryOffsetMm);
assert(guarded.warnings.some((warning) => warning.includes('已回退到原路径')));
assert(guarded.points.every((point) => point.geometryType === 'fallback'));
assert(guarded.stats.maxOffsetMm < 1e-6);

const radiusLimited = planner.planGlobal(syntheticRightAngle(), { minimumRadiusMm: 150 });
assert.strictEqual(radiusLimited.stats.curvatureLimitMet, true);
assert(radiusLimited.stats.achievedMinimumRadiusMm >= 150);
assert(radiusLimited.stats.maxOffsetMm <= radiusLimited.options.maxGeometryOffsetMm);
const minimumRangeRadius = planner.planGlobal(syntheticRightAngle(), { minimumRadiusMm: 100 });
assert.strictEqual(minimumRangeRadius.stats.curvatureLimitMet, true);
assert(minimumRangeRadius.stats.achievedMinimumRadiusMm >= 100);
const maximumRangeRadius = planner.planGlobal(syntheticRightAngle(), { minimumRadiusMm: 350 });
assert.strictEqual(maximumRangeRadius.stats.curvatureLimitMet, true,
  `350 mm radius should fit the 120 mm offset guard; achieved ${maximumRangeRadius.stats.achievedMinimumRadiusMm}`);
assert(maximumRangeRadius.stats.achievedMinimumRadiusMm >= 350);
assert(maximumRangeRadius.stats.achievedMinimumRadiusMm < 370,
  `350 mm request should not overshoot excessively: ${maximumRangeRadius.stats.achievedMinimumRadiusMm}`);
assert(maximumRangeRadius.stats.maxOffsetMm <= 120);
const impossibleRadius = planner.planGlobal(syntheticRightAngle(), { minimumRadiusMm: 500 });
assert.strictEqual(impossibleRadius.stats.curvatureLimitMet, false);
assert(impossibleRadius.warnings.some((warning) => warning.includes('不可满足')));
const corridorOptimized = planner.planCorridor(syntheticRightAngle(), { minimumRadiusMm: 180 });
assert.strictEqual(corridorOptimized.algorithm, 'corridor_optimizer');
assert.strictEqual(corridorOptimized.stats.curvatureLimitMet, true);
assert(corridorOptimized.stats.achievedMinimumRadiusMm >= 180);
assert(corridorOptimized.stats.maxOffsetMm <= corridorOptimized.options.maxGeometryOffsetMm);
assert(corridorOptimized.points.every((point) => point.geometryType === 'corridor_optimized'));

console.log(JSON.stringify({
  legacy: result.stats,
  radiusMm: corner.radiusMm,
  globalRightAngle: globalRightAngle.stats,
  globalCompound: globalCompound.stats,
  globalS: globalS.stats,
  globalSCurvature: sStats,
  guarded: guarded.stats,
  radiusLimited: radiusLimited.stats,
  minimumRangeRadius: minimumRangeRadius.stats,
  maximumRangeRadius: maximumRangeRadius.stats,
  impossibleRadius: impossibleRadius.stats,
  corridorOptimized: corridorOptimized.stats
}, null, 2));
