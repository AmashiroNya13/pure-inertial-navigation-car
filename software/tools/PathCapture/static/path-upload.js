(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  if (root) root.PathUpload = api;
}(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  'use strict';

  const DEFAULT_SPACING_MM = 5;
  const DEFAULT_SPEED_MM_S = 1200;
  const DEFAULT_CURVE_ENTER_PER_MM = 0.0015;
  const DEFAULT_CURVE_EXIT_PER_MM = 0.0008;
  const DEFAULT_CURVE_WINDOW_MM = 30;
  const DEFAULT_MIN_CURVE_MM = 30;
  const DEFAULT_MIN_STRAIGHT_MM = 180;
  const DEFAULT_MAX_MARKERS = 64;
  const DEFAULT_PHOTOTUBE_ENTER_HALF_MM = 90;
  const DEFAULT_PHOTOTUBE_EXIT_HALF_MM = 100;
  const DEFAULT_MAX_PHOTOTUBE_ZONES = 128;

  function finite(value) {
    const number = Number(value);
    return Number.isFinite(number) ? number : null;
  }

  function pointSpeed(point, fallback) {
    const direct = finite(point && point.planned_speed_mm_s);
    if (direct !== null) return direct;
    const millimeters = finite(point && point.speed_mm_s);
    if (millimeters !== null) return millimeters;
    const externalMeters = finite(point && point.external_speed_mps);
    if (externalMeters !== null) return externalMeters * 1000;
    const meters = finite(point && point.speed_mps);
    return meters !== null ? meters * 1000 : fallback;
  }

  function normalize(input, fallbackSpeedMmS) {
    const points = [];
    for (const raw of Array.isArray(input) ? input : []) {
      const x = finite(raw && (raw.x_mm ?? raw.x));
      const y = finite(raw && (raw.y_mm ?? raw.y));
      if (x === null || y === null) throw new Error('路径包含无效坐标');
      const previous = points[points.length - 1];
      if (previous && Math.hypot(x - previous.x, y - previous.y) < 0.001) continue;
      points.push({ x, y, speed_mm_s: pointSpeed(raw, fallbackSpeedMmS), s_mm: 0 });
    }
    if (points.length < 2) throw new Error('路径至少需要两个不同坐标点');

    let distance = 0;
    points.forEach((point, index) => {
      if (index) {
        const segment = Math.hypot(point.x - points[index - 1].x, point.y - points[index - 1].y);
        if (!Number.isFinite(segment) || segment > 100000) throw new Error('路径包含异常跳点');
        distance += segment;
      }
      point.s_mm = distance;
    });
    if (distance < 0.001) throw new Error('路径总长度为零');
    return points;
  }

  function interpolateAt(points, distance, cursorState) {
    let index = cursorState.index;
    while (index + 1 < points.length && points[index + 1].s_mm < distance - 1e-7) index += 1;
    cursorState.index = Math.min(index, points.length - 2);
    const a = points[cursorState.index];
    const b = points[cursorState.index + 1];
    const span = Math.max(1e-9, b.s_mm - a.s_mm);
    const ratio = Math.max(0, Math.min(1, (distance - a.s_mm) / span));
    return {
      x_mm: a.x + (b.x - a.x) * ratio,
      y_mm: a.y + (b.y - a.y) * ratio,
      speed_mm_s: a.speed_mm_s + (b.speed_mm_s - a.speed_mm_s) * ratio,
      s_mm: distance
    };
  }

  function applyHeadings(points) {
    let previous = null;
    points.forEach((point, index) => {
      const before = points[Math.max(0, index - 1)];
      const after = points[Math.min(points.length - 1, index + 1)];
      let heading = Math.atan2(after.y_mm - before.y_mm, after.x_mm - before.x_mm);
      if (previous !== null) {
        while (heading - previous > Math.PI) heading -= Math.PI * 2;
        while (heading - previous < -Math.PI) heading += Math.PI * 2;
      }
      point.theta_rad = heading;
      previous = heading;
    });
  }

  function wrapAngle(angle) {
    while (angle > Math.PI) angle -= Math.PI * 2;
    while (angle < -Math.PI) angle += Math.PI * 2;
    return angle;
  }

  function curveRuns(points, options) {
    const supplied = options || {};
    const enter = Math.max(0, finite(supplied.curveEnterPerMm) ?? DEFAULT_CURVE_ENTER_PER_MM);
    const exit = Math.min(enter, Math.max(0, finite(supplied.curveExitPerMm) ?? DEFAULT_CURVE_EXIT_PER_MM));
    const windowMm = Math.max(5, finite(supplied.curveWindowMm) ?? DEFAULT_CURVE_WINDOW_MM);
    const minimumCurveMm = Math.max(0, finite(supplied.minimumCurveMm) ?? DEFAULT_MIN_CURVE_MM);
    const minimumStraightMm = Math.max(0, finite(supplied.minimumStraightMm) ?? DEFAULT_MIN_STRAIGHT_MM);
    const halfWindow = Math.max(1, Math.round(windowMm / Math.max(0.5, supplied.spacingMm || DEFAULT_SPACING_MM)));
    const curvature = points.map((point, index) => {
      const before = points[Math.max(0, index - halfWindow)];
      const after = points[Math.min(points.length - 1, index + halfWindow)];
      const distance = Math.max(1e-6, after.s_mm - before.s_mm);
      return Math.abs(wrapAngle(after.theta_rad - before.theta_rad) / distance);
    });
    const runs = [];
    let index = 0;
    while (index < points.length) {
      if (curvature[index] < enter) {
        index += 1;
        continue;
      }
      let start = index;
      let end = index;
      while (start > 0 && curvature[start - 1] >= exit) start -= 1;
      while (end + 1 < points.length && curvature[end + 1] >= exit) end += 1;
      if (!runs.length || start > runs[runs.length - 1].end) runs.push({ start, end });
      else runs[runs.length - 1].end = Math.max(runs[runs.length - 1].end, end);
      index = end + 1;
    }

    const meaningful = runs.filter((run) => {
      const length = points[run.end].s_mm - points[run.start].s_mm;
      const headingChange = Math.abs(wrapAngle(points[run.end].theta_rad - points[run.start].theta_rad));
      return length >= minimumCurveMm || headingChange >= Math.PI / 36;
    });
    const merged = [];
    for (const run of meaningful) {
      const previous = merged[merged.length - 1];
      const gapMm = previous ? points[run.start].s_mm - points[previous.end].s_mm : Infinity;
      if (previous && gapMm < minimumStraightMm) previous.end = run.end;
      else merged.push({ start: run.start, end: run.end });
    }
    return merged;
  }

  function deriveCurveMarkers(points, options) {
    const supplied = options || {};
    const maxMarkers = Math.max(2, Math.floor(finite(supplied.maxMarkers) ?? DEFAULT_MAX_MARKERS));
    const runs = curveRuns(points, supplied);
    while (runs.length * 2 > maxMarkers && runs.length > 1) {
      let mergeIndex = 0;
      let shortestGap = Infinity;
      for (let index = 0; index + 1 < runs.length; index += 1) {
        const gap = points[runs[index + 1].start].s_mm - points[runs[index].end].s_mm;
        if (gap < shortestGap) {
          shortestGap = gap;
          mergeIndex = index;
        }
      }
      runs[mergeIndex].end = runs[mergeIndex + 1].end;
      runs.splice(mergeIndex + 1, 1);
    }
    const markers = [];
    runs.forEach((run) => {
      markers.push([run.start, 'in']);
      markers.push([run.end, 'out']);
    });
    return markers;
  }

  function derivePhototubeZones(points, nodes, options) {
    const supplied = options || {};
    const enterHalfMm = Math.max(0, finite(supplied.phototubeEnterHalfMm)
      ?? DEFAULT_PHOTOTUBE_ENTER_HALF_MM);
    const exitHalfMm = Math.max(enterHalfMm, finite(supplied.phototubeExitHalfMm)
      ?? DEFAULT_PHOTOTUBE_EXIT_HALF_MM);
    const maxZones = Math.max(0, Math.floor(finite(supplied.maxPhototubeZones)
      ?? DEFAULT_MAX_PHOTOTUBE_ZONES));
    const centers = [];
    for (const node of Array.isArray(nodes) ? nodes : []) {
      const xMm = finite(node && (node.x_mm ?? node.x));
      const yMm = finite(node && (node.y_mm ?? node.y));
      if (xMm !== null && yMm !== null) centers.push({ x_mm: xMm, y_mm: yMm });
    }
    if (!centers.length || !Array.isArray(points) || !points.length) return [];

    const active = new Array(centers.length).fill(false);
    const zones = [];
    let zoneStart = null;
    points.forEach((point, pointIndex) => {
      centers.forEach((center, centerIndex) => {
        const dx = Math.abs(point.x_mm - center.x_mm);
        const dy = Math.abs(point.y_mm - center.y_mm);
        if (active[centerIndex]) {
          if (dx > exitHalfMm || dy > exitHalfMm) active[centerIndex] = false;
        } else if (dx <= enterHalfMm && dy <= enterHalfMm) {
          active[centerIndex] = true;
        }
      });
      const forbidden = active.some(Boolean);
      if (forbidden && zoneStart === null) zoneStart = pointIndex;
      if (!forbidden && zoneStart !== null) {
        zones.push([zoneStart, pointIndex - 1]);
        zoneStart = null;
      }
    });
    if (zoneStart !== null) zones.push([zoneStart, points.length - 1]);
    if (zones.length > maxZones) {
      throw new Error(`Phototube exclusion zones exceed controller limit ${maxZones}`);
    }
    return zones;
  }

  function prepare(input, options) {
    const supplied = options || {};
    const spacingMm = Math.max(0.5, finite(supplied.spacingMm) ?? DEFAULT_SPACING_MM);
    const fallbackSpeedMmS = Math.max(0, finite(supplied.defaultSpeedMmS) ?? DEFAULT_SPEED_MM_S);
    const source = normalize(input, fallbackSpeedMmS);
    const totalDistanceMm = source[source.length - 1].s_mm;
    const distances = [];
    for (let distance = 0; distance < totalDistanceMm - 1e-7; distance += spacingMm) {
      distances.push(distance);
    }
    if (!distances.length || Math.abs(distances[distances.length - 1] - totalDistanceMm) > 1e-7) {
      distances.push(totalDistanceMm);
    }

    const cursor = { index: 0 };
    const points = distances.map((distance) => interpolateAt(source, distance, cursor));
    applyHeadings(points);
    if (supplied.stopAtEnd !== false) points[points.length - 1].speed_mm_s = 0;
    const markers = supplied.curveMarkers === false
      ? [] : deriveCurveMarkers(points, { ...supplied, spacingMm });
    const phototubeZones = derivePhototubeZones(points, supplied.phototubeNodes, supplied);
    return { points, markers, phototubeZones, spacingMm, totalDistanceMm };
  }

  return {
    DEFAULT_SPACING_MM,
    DEFAULT_SPEED_MM_S,
    DEFAULT_CURVE_ENTER_PER_MM,
    DEFAULT_CURVE_EXIT_PER_MM,
    DEFAULT_PHOTOTUBE_ENTER_HALF_MM,
    DEFAULT_PHOTOTUBE_EXIT_HALF_MM,
    deriveCurveMarkers,
    derivePhototubeZones,
    prepare
  };
}));
