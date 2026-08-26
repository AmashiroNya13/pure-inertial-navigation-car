'use strict';

const assert = require('assert');
const external = require('../static/external-path.js');
const pathUpload = require('../static/path-upload.js');

const fixture = {
  unit: 'mm',
  start_node_id: 1,
  nodes: [
    { id: 1, name: 'A', x_mm: 100, y_mm: 200 },
    { id: 2, name: 'B', x_mm: 100, y_mm: 300 },
    { id: 3, name: 'C', x_mm: 200, y_mm: 300 }
  ],
  computed_shortest_route: {
    full_path_node_ids: [1, 2, 3, 1],
    speed_segments: [
      { turn_node_id: 2, speed_mps: 1.2, points_mm: [{ x: 100, y: 200 }, { x: 100, y: 250 }] },
      { turn_node_id: 3, speed_mps: 0.8, points_mm: [{ x: 100, y: 250 }, { x: 200, y: 300 }] }
    ]
  }
};

const document = external.parseDocument(fixture);
assert.strictEqual(document.nodes.length, 3);
assert.strictEqual(external.nodeRoute(document).length, 4, 'closed route endpoint must be retained');
assert.strictEqual(external.segmentRoute(document).length, 3);

const traversalDocument = external.parseDocument({
  ...fixture,
  end_node_id: 3,
  travel_direction: 'clockwise',
  start_heading_deg: 90,
  planned_lap_count: 4,
  stop_lap: 3
});
assert.strictEqual(traversalDocument.endNodeId, '3');
assert.strictEqual(traversalDocument.travelDirection, 'clockwise');
assert.strictEqual(traversalDocument.startHeadingDeg, 90);
assert.strictEqual(traversalDocument.plannedLapCount, 4);
assert.strictEqual(traversalDocument.stopLap, 3);
assert.strictEqual(traversalDocument.hasTraversalMetadata, true);

const plannedDocument = external.parseDocument({
  ...fixture,
  computed_shortest_route: {
    ...fixture.computed_shortest_route,
    planned_path_points: [
      { x_mm: 100, y_mm: 200, s_mm: 0, speed_mps: 1.2 },
      { x_mm: 105, y_mm: 200, s_mm: 5, speed_mps: 1.25 },
      { x_mm: 110, y_mm: 200, s_mm: 10, speed_mps: 1.3 }
    ]
  }
});
const plannedRoute = external.segmentRoute(plannedDocument);
assert.strictEqual(plannedRoute.length, 3, 'planned 5 mm points must take priority over legacy segments');
assert.strictEqual(plannedRoute[1].x, 105);
assert.strictEqual(plannedRoute[1].external_distance_mm, 5);
assert.strictEqual(plannedRoute[1].external_speed_mps, 1.25);

const aligned = external.route(document, 'nodes', { alignFirstSegment: true });
assert(Math.abs(aligned[0].x) < 1e-9 && Math.abs(aligned[0].y) < 1e-9);
assert(aligned[1].x > 99.9 && Math.abs(aligned[1].y) < 1e-8);
const clockwise = external.route(document, 'nodes', { rotationDeg: -90 });
assert(clockwise[1].x > 99.9 && Math.abs(clockwise[1].y) < 1e-8);
assert.strictEqual(clockwise[1].x_mm, clockwise[1].x);
assert.strictEqual(clockwise[1].y_mm, clockwise[1].y);
const clockwiseResampled = pathUpload.prepare(clockwise, {
  spacingMm: 5,
  defaultSpeedMmS: 1200,
  stopAtEnd: false
}).points;
assert(clockwiseResampled[20].x_mm > 99.9 && Math.abs(clockwiseResampled[20].y_mm) < 1e-8,
  '5 mm resampling must retain the transformed node coordinate frame');
const mirrored = external.route(document, 'nodes', { mirrorY: true });
assert(mirrored[1].y < -99.9);
const reversed = external.route(document, 'nodes', { reverse: true, originNodeId: 1 });
assert.strictEqual(reversed.length, 4);
const preserved = external.route(document, 'nodes', {
  originNodeId: 1,
  targetOriginX: 130,
  targetOriginY: 180
});
assert.strictEqual(preserved[0].x, 130);
assert.strictEqual(preserved[0].y, 180);
const restarted = external.route(document, 'nodes', {
  originNodeId: 2,
  targetOriginX: 100,
  targetOriginY: 300,
  startAtOrigin: true
});
assert.strictEqual(restarted[0].external_node_id, '2');
assert.strictEqual(restarted[0].x, 100);
assert.strictEqual(restarted[0].y, 300);
assert.strictEqual(restarted.at(-1).external_node_id, '2');
const bands = external.buildOffsetBands([
  { x: 0, y: 0 }, { x: 1000, y: 0 }
], 150, 40);
assert.strictEqual(bands.length, 2);
assert.strictEqual(bands[0].inner[0].y, 150);
assert.strictEqual(bands[0].outer[0].y, 190);
assert.strictEqual(bands[1].inner[0].y, -150);
assert.strictEqual(bands[1].outer[0].y, -190);

const networkDocument = external.parseDocument({
  ...fixture,
  nodes: [...fixture.nodes, { id: 4, name: 'D', x_mm: 500, y_mm: 200 }],
  links: [
    { from: 1, to: 2 },
    { from: 2, to: 3 },
    { from: 1, to: 4 }
  ]
});
const network = external.networkSegments(networkDocument, 'nodes', { rotationDeg: 0 });
assert.strictEqual(network.length, 3, 'shoulders must use every exported track link');
assert(network.some((segment) => segment.to.id === '4'), 'off-route track must remain in shoulder network');
const networkNodes = external.networkNodes(networkDocument, 'nodes', { rotationDeg: 0 });
assert.strictEqual(networkNodes.length, 4, 'the original graph layer must retain every exported node');
assert(networkNodes.some((point) => point.id === '4'), 'off-route nodes must remain visible');
const legacyNetworkDocument = external.parseDocument({
  ...fixture,
  original_links: [{ from: 1, to: 2 }, { from: 2, to: 3 }]
});
assert.strictEqual(
  external.networkSegments(legacyNetworkDocument, 'nodes', { rotationDeg: 0 }).length,
  2,
  'legacy original_links must remain supported'
);

const rectangleTrack = [
  { from: { x: 0, y: 0 }, to: { x: 1000, y: 0 } },
  { from: { x: 1000, y: 0 }, to: { x: 1000, y: 500 } },
  { from: { x: 1000, y: 500 }, to: { x: 0, y: 500 } },
  { from: { x: 0, y: 500 }, to: { x: 0, y: 0 } }
];
const rectangleShoulders = external.buildShoulderPaths(rectangleTrack, 150, 40);
assert.strictEqual(rectangleShoulders.length, 1, 'one enclosed region must produce one shoulder');
assert.strictEqual(rectangleShoulders[0].points.length, 2);
assert(Math.abs(rectangleShoulders[0].points[0].x - 150) <= 5);
assert(Math.abs(rectangleShoulders[0].points[1].x - 850) <= 5);
assert(rectangleShoulders[0].points.every((point) => Math.abs(point.y - 250) < 0.1));
const wideTrack = rectangleTrack.map((segment) => ({
  from: { ...segment.from, y: segment.from.y === 500 ? 800 : segment.from.y },
  to: { ...segment.to, y: segment.to.y === 500 ? 800 : segment.to.y }
}));
assert.strictEqual(
  external.buildShoulderPaths(wideTrack, 150, 40).length,
  1,
  'a large enclosed rectangle must still produce its internal centerline'
);
const narrowTrack = rectangleTrack.map((segment) => ({
  from: { ...segment.from, x: segment.from.x === 1000 ? 300 : segment.from.x },
  to: { ...segment.to, x: segment.to.x === 1000 ? 300 : segment.to.x }
}));
const narrowShoulder = external.buildShoulderPaths(narrowTrack, 150, 40);
assert.strictEqual(
  narrowShoulder.length,
  1,
  'a 300 mm region may place its centerline exactly 150 mm from both sides'
);
const adjacentRegions = [
  { from: { x: 0, y: 0 }, to: { x: 2000, y: 0 } },
  { from: { x: 2000, y: 0 }, to: { x: 2000, y: 500 } },
  { from: { x: 2000, y: 500 }, to: { x: 0, y: 500 } },
  { from: { x: 0, y: 500 }, to: { x: 0, y: 0 } },
  { from: { x: 1000, y: 0 }, to: { x: 1000, y: 500 } }
];
assert.strictEqual(
  external.buildShoulderPaths(adjacentRegions, 150, 40).length,
  2,
  'adjacent bounded regions must each receive exactly one shoulder line'
);

const concavePolygon = [
  { x: 0, y: 0 }, { x: 1200, y: 0 }, { x: 1200, y: 800 },
  { x: 700, y: 800 }, { x: 700, y: 500 }, { x: 500, y: 500 },
  { x: 500, y: 800 }, { x: 0, y: 800 }
];
const concaveTrack = concavePolygon.map((point, index) => ({
  from: point,
  to: concavePolygon[(index + 1) % concavePolygon.length]
}));
const concaveShoulders = external.buildShoulderPaths(concaveTrack, 150, 40);
assert(concaveShoulders.every((shoulder) => shoulder.points.length === 2),
  'shoulders must remain straight rather than bending around local road sections');
function pointSegmentDistance(point, segment) {
  const dx = segment.to.x - segment.from.x;
  const dy = segment.to.y - segment.from.y;
  const denominator = dx * dx + dy * dy;
  const ratio = denominator > 0 ? Math.max(0, Math.min(1,
    ((point.x - segment.from.x) * dx + (point.y - segment.from.y) * dy) / denominator
  )) : 0;
  return Math.hypot(point.x - segment.from.x - dx * ratio, point.y - segment.from.y - dy * ratio);
}
concaveShoulders.forEach((shoulder) => shoulder.points.slice(1).forEach((to, index) => {
  const from = shoulder.points[index];
  const length = Math.hypot(to.x - from.x, to.y - from.y);
  for (let sample = 0; sample <= Math.ceil(length / 2); sample += 1) {
    const ratio = sample / Math.max(1, Math.ceil(length / 2));
    const point = { x: from.x + (to.x - from.x) * ratio, y: from.y + (to.y - from.y) * ratio };
    const clearance = Math.min(...concaveTrack.map((segment) => pointSegmentDistance(point, segment)));
    assert(clearance >= 149.5, `concave shoulder clearance dropped to ${clearance.toFixed(2)} mm`);
  }
}));

const lPolygon = [
  { x: 0, y: 0 }, { x: 1000, y: 0 }, { x: 1000, y: 400 },
  { x: 400, y: 400 }, { x: 400, y: 1000 }, { x: 0, y: 1000 }
];
const lTrack = lPolygon.map((point, index) => ({
  from: point,
  to: lPolygon[(index + 1) % lPolygon.length]
}));
const lShoulders = external.buildShoulderPaths(lTrack, 150, 40);
assert.strictEqual(lShoulders.length, 2, 'an L-shaped enclosure must produce one centerline per long arm');
assert(lShoulders.some((shoulder) => shoulder.points.every((point) => Math.abs(point.y - 200) <= 5)),
  'the horizontal L arm must be centered');
assert(lShoulders.some((shoulder) => shoulder.points.every((point) => Math.abs(point.x - 200) <= 5)),
  'the vertical L arm must be centered');
lShoulders.forEach((shoulder) => shoulder.points.slice(1).forEach((to, index) => {
  const from = shoulder.points[index];
  const length = Math.hypot(to.x - from.x, to.y - from.y);
  for (let sample = 0; sample <= Math.ceil(length / 2); sample += 1) {
    const ratio = sample / Math.max(1, Math.ceil(length / 2));
    const point = { x: from.x + (to.x - from.x) * ratio, y: from.y + (to.y - from.y) * ratio };
    const clearance = Math.min(...lTrack.map((segment) => pointSegmentDistance(point, segment)));
    assert(clearance >= 149.5, `L shoulder clearance dropped to ${clearance.toFixed(2)} mm`);
  }
}));

assert.throws(() => external.parseDocument({}), /nodes/);
assert.throws(() => external.nodeRoute(external.parseDocument({ nodes: fixture.nodes })), /full_path/);

console.log(JSON.stringify({ aligned, segmentCount: external.segmentRoute(document).length }));
