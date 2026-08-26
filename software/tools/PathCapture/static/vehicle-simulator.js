(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  if (root) root.VehicleSimulator = api;
}(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  'use strict';

  function finite(value, fallback) {
    const number = Number(value);
    return Number.isFinite(number) ? number : fallback;
  }

  function plannedSpeed(point) {
    for (const value of [point && point.planned_speed_mm_s, point && point.speed_mm_s]) {
      const speed = Number(value);
      if (Number.isFinite(speed)) return speed;
    }
    const external = Number(point && point.external_speed_mps);
    return Number.isFinite(external) ? external * 1000 : null;
  }

  function preparePath(input, options) {
    const config = options || {};
    const fixedSpeedMmS = Math.max(1, finite(config.fixedSpeedMmS, 1200));
    const usePlannedSpeed = config.usePlannedSpeed !== false;
    const source = (Array.isArray(input) ? input : []).filter((point) =>
      Number.isFinite(Number(point && point.x)) && Number.isFinite(Number(point && point.y))
    );
    const points = [];
    let totalMm = 0;
    let totalSec = 0;
    let hasPlannedSpeed = false;
    source.forEach((point) => {
      const sourceSpeed = plannedSpeed(point);
      if (sourceSpeed !== null && point.has_planned_speed !== false) hasPlannedSpeed = true;
      const speedMmS = usePlannedSpeed && sourceSpeed !== null && point.has_planned_speed !== false
        ? Math.max(0, sourceSpeed) : fixedSpeedMmS;
      const next = { ...point, x: Number(point.x), y: Number(point.y), s_mm: totalMm, t_s: totalSec, speed_mm_s: speedMmS };
      const previous = points[points.length - 1];
      if (previous) {
        const distance = Math.hypot(next.x - previous.x, next.y - previous.y);
        if (distance < 1e-6) return;
        totalMm += distance;
        const averageSpeed = Math.max(1, (previous.speed_mm_s + next.speed_mm_s) * 0.5);
        totalSec += distance / averageSpeed;
        next.s_mm = totalMm;
        next.t_s = totalSec;
      }
      points.push(next);
    });
    let timedBySource = false;
    if (config.useElapsedTime && points.length > 1) {
      const sourceTimes = points.map((point) => finite(point.elapsed_s, finite(point.t, NaN)));
      const startTime = sourceTimes[0];
      const monotonic = Number.isFinite(startTime) && sourceTimes.every((time, index) =>
        Number.isFinite(time) && (index === 0 || time >= sourceTimes[index - 1])
      );
      if (monotonic && sourceTimes[sourceTimes.length - 1] > startTime) {
        points.forEach((point, index) => { point.t_s = sourceTimes[index] - startTime; });
        totalSec = points[points.length - 1].t_s;
        timedBySource = true;
      }
    }
    return {
      points,
      totalMm,
      totalSec,
      hasPlannedSpeed: usePlannedSpeed && hasPlannedSpeed,
      fixedSpeedMmS,
      timedBySource
    };
  }

  function pointAtDistance(path, distanceMm) {
    const points = path && Array.isArray(path.points) ? path.points : [];
    if (!points.length) return null;
    const target = Math.max(0, Math.min(path.totalMm, finite(distanceMm, 0)));
    let low = 1;
    let high = points.length - 1;
    while (low < high) {
      const middle = Math.floor((low + high) / 2);
      if (points[middle].s_mm < target) low = middle + 1;
      else high = middle;
    }
    const index = Math.min(points.length - 1, low);
    const before = points[Math.max(0, index - 1)];
    const after = points[index];
    const segment = after.s_mm - before.s_mm;
    const ratio = segment > 1e-6 ? (target - before.s_mm) / segment : 0;
    return {
      x: before.x + (after.x - before.x) * ratio,
      y: before.y + (after.y - before.y) * ratio,
      s_mm: target,
      t_s: before.t_s + (after.t_s - before.t_s) * ratio,
      speed_mm_s: before.speed_mm_s + (after.speed_mm_s - before.speed_mm_s) * ratio,
      index
    };
  }

  function pointAtTime(path, timeSec) {
    const points = path && Array.isArray(path.points) ? path.points : [];
    if (!points.length) return null;
    const target = Math.max(0, Math.min(path.totalSec, finite(timeSec, 0)));
    let low = 1;
    let high = points.length - 1;
    while (low < high) {
      const middle = Math.floor((low + high) / 2);
      if (points[middle].t_s < target) low = middle + 1;
      else high = middle;
    }
    const index = Math.min(points.length - 1, low);
    const before = points[Math.max(0, index - 1)];
    const after = points[index];
    const duration = after.t_s - before.t_s;
    const ratio = duration > 1e-9 ? (target - before.t_s) / duration : 0;
    return pointAtDistance(path, before.s_mm + (after.s_mm - before.s_mm) * ratio);
  }

  function poseAtDistance(path, distanceMm, tangentWindowMm) {
    const point = pointAtDistance(path, distanceMm);
    if (!point) return null;
    const windowMm = Math.max(1, finite(tangentWindowMm, 5));
    let before = pointAtDistance(path, point.s_mm - windowMm);
    let after = pointAtDistance(path, point.s_mm + windowMm);
    if (before && after && Math.hypot(after.x - before.x, after.y - before.y) < 1e-6) {
      before = path.points[Math.max(0, point.index - 1)];
      after = path.points[Math.min(path.points.length - 1, point.index)];
    }
    return {
      ...point,
      headingRad: before && after ? Math.atan2(after.y - before.y, after.x - before.x) : 0
    };
  }

  function poseAtTime(path, timeSec, tangentWindowMm) {
    const point = pointAtTime(path, timeSec);
    return point ? poseAtDistance(path, point.s_mm, tangentWindowMm) : null;
  }

  function straightSections(path, options) {
    const points = path && Array.isArray(path.points) ? path.points : [];
    if (points.length < 3) return [];
    const config = options || {};
    const windowMm = Math.max(5, finite(config.windowMm, 30));
    const curvatureLimit = Math.max(0, finite(config.curvatureLimitPerMm, 0.0008));
    const minimumLengthMm = Math.max(0, finite(config.minimumLengthMm, 140));
    const flags = points.map((point) => {
      const before = pointAtDistance(path, point.s_mm - windowMm);
      const after = pointAtDistance(path, point.s_mm + windowMm);
      if (!before || !after || after.s_mm - before.s_mm < 1e-6) return false;
      const incoming = Math.atan2(point.y - before.y, point.x - before.x);
      const outgoing = Math.atan2(after.y - point.y, after.x - point.x);
      let delta = outgoing - incoming;
      while (delta > Math.PI) delta -= Math.PI * 2;
      while (delta < -Math.PI) delta += Math.PI * 2;
      return Math.abs(delta) / (after.s_mm - before.s_mm) <= curvatureLimit;
    });
    const sections = [];
    let index = 0;
    while (index < flags.length) {
      while (index < flags.length && !flags[index]) index += 1;
      const startIndex = index;
      while (index < flags.length && flags[index]) index += 1;
      const endIndex = index - 1;
      if (startIndex < points.length && endIndex >= startIndex &&
          points[endIndex].s_mm - points[startIndex].s_mm >= minimumLengthMm) {
        const expandedStartIndex = sections.length === 0 &&
          points[startIndex].s_mm <= windowMm * 1.5 ? 0 : startIndex;
        sections.push({
          startIndex: expandedStartIndex,
          endIndex,
          startMm: points[expandedStartIndex].s_mm,
          endMm: points[endIndex].s_mm
        });
      }
    }
    return sections;
  }

  function footprint(pose, config) {
    if (!pose) return [];
    const lengthMm = Math.max(1, finite(config && config.lengthMm, 150));
    const widthMm = Math.max(1, finite(config && config.widthMm, 140));
    const pivotFromRearMm = Math.max(0, Math.min(lengthMm,
      finite(config && config.pivotFromRearMm, 50)));
    const cosine = Math.cos(pose.headingRad);
    const sine = Math.sin(pose.headingRad);
    const transform = (forward, left) => ({
      x: pose.x + forward * cosine - left * sine,
      y: pose.y + forward * sine + left * cosine
    });
    const rear = -pivotFromRearMm;
    const front = lengthMm - pivotFromRearMm;
    const halfWidth = widthMm * 0.5;
    return [
      transform(front, halfWidth),
      transform(front, -halfWidth),
      transform(rear, -halfWidth),
      transform(rear, halfWidth)
    ];
  }

  function pointSegmentDistance(point, segment) {
    const dx = segment.to.x - segment.from.x;
    const dy = segment.to.y - segment.from.y;
    const lengthSquared = dx * dx + dy * dy;
    if (lengthSquared < 1e-12) return Math.hypot(point.x - segment.from.x, point.y - segment.from.y);
    const ratio = Math.max(0, Math.min(1,
      ((point.x - segment.from.x) * dx + (point.y - segment.from.y) * dy) / lengthSquared
    ));
    return Math.hypot(point.x - (segment.from.x + dx * ratio), point.y - (segment.from.y + dy * ratio));
  }

  function samplePerimeter(corners, spacingMm) {
    const samples = [];
    const spacing = Math.max(1, finite(spacingMm, 5));
    corners.forEach((from, index) => {
      const to = corners[(index + 1) % corners.length];
      const length = Math.hypot(to.x - from.x, to.y - from.y);
      const steps = Math.max(1, Math.ceil(length / spacing));
      for (let step = 0; step < steps; step += 1) {
        const ratio = step / steps;
        samples.push({ x: from.x + (to.x - from.x) * ratio, y: from.y + (to.y - from.y) * ratio });
      }
    });
    return samples;
  }

  function collision(pose, config, trackSegments, clearanceMm) {
    const corners = footprint(pose, config);
    const segments = Array.isArray(trackSegments) ? trackSegments : [];
    if (corners.length !== 4 || !segments.length) {
      return { collided: false, maxDistanceMm: 0, intrusionMm: 0, contact: null, corners };
    }
    const clearance = Math.max(0, finite(clearanceMm, 150));
    let maxDistanceMm = -Infinity;
    let contact = null;
    samplePerimeter(corners, config && config.sampleSpacingMm).forEach((point) => {
      let nearest = Infinity;
      segments.forEach((segment) => { nearest = Math.min(nearest, pointSegmentDistance(point, segment)); });
      if (nearest > maxDistanceMm) {
        maxDistanceMm = nearest;
        contact = point;
      }
    });
    return {
      collided: maxDistanceMm >= clearance,
      maxDistanceMm,
      intrusionMm: Math.max(0, maxDistanceMm - clearance),
      contact,
      corners
    };
  }

  function orientation(a, b, c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  }

  function segmentsIntersect(first, second) {
    const a = orientation(first.from, first.to, second.from);
    const b = orientation(first.from, first.to, second.to);
    const c = orientation(second.from, second.to, first.from);
    const d = orientation(second.from, second.to, first.to);
    if ([a, b, c, d].every((value) => Math.abs(value) <= 1e-8)) {
      return Math.max(Math.min(first.from.x, first.to.x), Math.min(second.from.x, second.to.x)) <=
          Math.min(Math.max(first.from.x, first.to.x), Math.max(second.from.x, second.to.x)) + 1e-8 &&
        Math.max(Math.min(first.from.y, first.to.y), Math.min(second.from.y, second.to.y)) <=
          Math.min(Math.max(first.from.y, first.to.y), Math.max(second.from.y, second.to.y)) + 1e-8;
    }
    return ((a <= 1e-8 && b >= -1e-8) || (a >= -1e-8 && b <= 1e-8)) &&
      ((c <= 1e-8 && d >= -1e-8) || (c >= -1e-8 && d <= 1e-8));
  }

  function segmentDistance(first, second) {
    if (segmentsIntersect(first, second)) return 0;
    return Math.min(
      pointSegmentDistance(first.from, second),
      pointSegmentDistance(first.to, second),
      pointSegmentDistance(second.from, first),
      pointSegmentDistance(second.to, first)
    );
  }

  function pointInConvexPolygon(point, polygon) {
    let positive = false;
    let negative = false;
    polygon.forEach((from, index) => {
      const value = orientation(from, polygon[(index + 1) % polygon.length], point);
      if (value > 1e-8) positive = true;
      if (value < -1e-8) negative = true;
    });
    return !(positive && negative);
  }

  function collisionWithShoulders(pose, config, shoulderSegments, shoulderWidthMm) {
    const corners = footprint(pose, config);
    const segments = Array.isArray(shoulderSegments) ? shoulderSegments : [];
    const halfWidth = Math.max(0.5, finite(shoulderWidthMm, 40) * 0.5);
    if (corners.length !== 4 || !segments.length) {
      return { collided: false, overlapMm: 0, contact: null, corners };
    }
    const bodyEdges = corners.map((from, index) => ({
      from,
      to: corners[(index + 1) % corners.length]
    }));
    let minimumDistance = Infinity;
    let contact = null;
    segments.forEach((segment) => {
      if (pointInConvexPolygon(segment.from, corners) || pointInConvexPolygon(segment.to, corners)) {
        minimumDistance = 0;
        contact = pointInConvexPolygon(segment.from, corners) ? segment.from : segment.to;
        return;
      }
      bodyEdges.forEach((edge) => {
        const distance = segmentDistance(edge, segment);
        if (distance < minimumDistance) {
          minimumDistance = distance;
          contact = {
            x: (segment.from.x + segment.to.x) * 0.5,
            y: (segment.from.y + segment.to.y) * 0.5
          };
        }
      });
    });
    return {
      collided: minimumDistance <= halfWidth,
      overlapMm: Math.max(0, halfWidth - minimumDistance),
      contact,
      corners
    };
  }

  return {
    preparePath,
    pointAtDistance,
    pointAtTime,
    poseAtDistance,
    poseAtTime,
    straightSections,
    footprint,
    pointSegmentDistance,
    collision,
    collisionWithShoulders
  };
}));
