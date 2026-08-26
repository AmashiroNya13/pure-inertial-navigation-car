'use strict';

const assert = require('assert');
const PathUpload = require('../static/path-upload.js');

const straight = PathUpload.prepare([{ x: 0, y: 0 }, { x: 12, y: 0 }]);
assert.deepStrictEqual(straight.points.map((point) => point.s_mm), [0, 5, 10, 12]);
assert.strictEqual(straight.points[0].x_mm, 0);
assert.strictEqual(straight.points[straight.points.length - 1].x_mm, 12);
assert.strictEqual(straight.points[1].theta_rad, 0);
assert.strictEqual(straight.points[straight.points.length - 1].speed_mm_s, 0);
assert.deepStrictEqual(straight.markers, []);

const speed = PathUpload.prepare([
  { x: 0, y: 0, planned_speed_mm_s: 500 },
  { x: 10, y: 0, planned_speed_mm_s: 1500 }
]);
assert.strictEqual(speed.points[1].speed_mm_s, 1000);

const externalSpeed = PathUpload.prepare([
  { x: 0, y: 0, external_speed_mps: 1.0 },
  { x: 10, y: 0, external_speed_mps: 2.0 }
], { stopAtEnd: false });
assert.strictEqual(externalSpeed.points[1].speed_mm_s, 1500);
assert.strictEqual(externalSpeed.points[2].speed_mm_s, 2000);

const corner = PathUpload.prepare([{ x: 0, y: 0 }, { x: 10, y: 0 }, { x: 10, y: 10 }]);
assert.strictEqual(corner.points.length, 5);
assert(Math.abs(corner.points[2].theta_rad - Math.PI / 4) < 1e-9);
for (let index = 1; index < corner.points.length; index += 1) {
  assert(Math.abs(corner.points[index].theta_rad - corner.points[index - 1].theta_rad) <= Math.PI);
}

const roundedPath = [{ x: 0, y: 0 }, { x: 300, y: 0 }];
for (let step = 1; step <= 24; step += 1) {
  const angle = -Math.PI / 2 + (Math.PI / 2) * step / 24;
  roundedPath.push({ x: 300 + 200 * Math.cos(angle), y: 200 + 200 * Math.sin(angle) });
}
roundedPath.push({ x: 800, y: 200 });
const rounded = PathUpload.prepare(roundedPath, { stopAtEnd: false });
assert.strictEqual(rounded.markers.length, 2);
assert.strictEqual(rounded.markers[0][1], 'in');
assert.strictEqual(rounded.markers[1][1], 'out');
assert(rounded.points[rounded.markers[0][0]].s_mm > 250);
assert(rounded.points[rounded.markers[0][0]].s_mm < 360);
assert(rounded.points[rounded.markers[1][0]].s_mm > 560);
assert(rounded.points[rounded.markers[1][0]].s_mm < 680);

const zonePoints = Array.from({ length: 61 }, (_, index) => ({
  x_mm: index * 5 - 150,
  y_mm: 0
}));
assert.deepStrictEqual(PathUpload.derivePhototubeZones(zonePoints, [{ x_mm: 0, y_mm: 0 }]), [
  [12, 50]
]);

const overlappingZones = PathUpload.derivePhototubeZones(zonePoints, [
  { x_mm: -50, y_mm: 0 },
  { x_mm: 50, y_mm: 0 }
]);
assert.deepStrictEqual(overlappingZones, [[2, 60]]);

const repeatedPass = [
  { x_mm: -150, y_mm: 0 }, { x_mm: 0, y_mm: 0 }, { x_mm: 150, y_mm: 0 },
  { x_mm: 0, y_mm: 0 }, { x_mm: -150, y_mm: 0 }
];
assert.deepStrictEqual(PathUpload.derivePhototubeZones(repeatedPass, [{ x: 0, y: 0 }]), [
  [1, 1], [3, 3]
]);

const preparedZones = PathUpload.prepare([{ x: -150, y: 0 }, { x: 150, y: 0 }], {
  stopAtEnd: false,
  phototubeNodes: [{ x_mm: 0, y_mm: 0 }]
});
assert.deepStrictEqual(preparedZones.phototubeZones, [[12, 50]]);

assert.throws(() => PathUpload.prepare([{ x: 0, y: 0 }, { x: NaN, y: 1 }]), /无效坐标/);
assert.throws(() => PathUpload.prepare([{ x: 0, y: 0 }]), /至少需要两个/);

console.log('path upload tests passed');
