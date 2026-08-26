(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  if (root) root.ExternalPath = api;
}(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  'use strict';

  const finite = (value) => Number.isFinite(Number(value)) ? Number(value) : null;

  function parseDocument(raw) {
    const data = typeof raw === 'string' ? JSON.parse(raw) : raw;
    if (!data || typeof data !== 'object' || !Array.isArray(data.nodes)) {
      throw new Error('外部文件缺少 nodes[]');
    }
    const nodes = data.nodes.map((node) => ({
      ...node,
      id: String(node.id),
      x: finite(node.x_mm),
      y: finite(node.y_mm)
    })).filter((node) => node.x !== null && node.y !== null);
    if (nodes.length < 2) throw new Error('外部文件至少需要两个有效节点');
    const routeIds = data.computed_shortest_route && data.computed_shortest_route.full_path_node_ids;
    const rawLinks = Array.isArray(data.links) && data.links.length
      ? data.links
      : (Array.isArray(data.original_links) ? data.original_links : []);
    const links = rawLinks.map((link) => ({
      from: String(link.from),
      to: String(link.to)
    })).filter((link) => link.from !== link.to);
    return {
      raw: data,
      nodes,
      nodeById: new Map(nodes.map((node) => [node.id, node])),
      startNodeId: data.start_node_id == null ? nodes[0].id : String(data.start_node_id),
      endNodeId: data.end_node_id == null ? null : String(data.end_node_id),
      travelDirection: data.travel_direction || null,
      startHeadingDeg: finite(data.start_heading_deg),
      plannedLapCount: Math.max(1, Math.round(finite(data.planned_lap_count) || 1)),
      stopLap: Math.max(1, Math.round(finite(data.stop_lap) || 1)),
      hasTraversalMetadata: data.end_node_id != null || data.travel_direction != null || data.planned_lap_count != null,
      links,
      routeIds: Array.isArray(routeIds) ? routeIds.map(String) : [],
      plannedPathPoints: Array.isArray(data.computed_shortest_route && data.computed_shortest_route.planned_path_points)
        ? data.computed_shortest_route.planned_path_points
        : [],
      speedSegments: Array.isArray(data.computed_shortest_route && data.computed_shortest_route.speed_segments)
        ? data.computed_shortest_route.speed_segments
        : []
    };
  }

  function nodeRoute(document) {
    if (!document.routeIds.length) throw new Error('外部文件没有 computed_shortest_route.full_path_node_ids');
    return document.routeIds.map((id, index) => {
      const node = document.nodeById.get(id);
      if (!node) throw new Error(`路线引用了不存在的节点 ${id}`);
      return { ...node, external_node_id: id, source_index: index };
    });
  }

  function segmentRoute(document) {
    if (document.plannedPathPoints.length) {
      const planned = document.plannedPathPoints.map((raw, pointIndex) => {
        const x = finite(raw && raw.x_mm);
        const y = finite(raw && raw.y_mm);
        if (x === null || y === null) return null;
        return {
          x,
          y,
          external_speed_mps: finite(raw.speed_mps),
          external_distance_mm: finite(raw.s_mm),
          source_index: pointIndex
        };
      }).filter(Boolean);
      if (planned.length >= 2) return planned;
    }
    const result = [];
    document.speedSegments.forEach((segment, segmentIndex) => {
      const points = Array.isArray(segment.points_mm) ? segment.points_mm : [];
      points.forEach((raw, pointIndex) => {
        const x = finite(raw && raw.x);
        const y = finite(raw && raw.y);
        if (x === null || y === null) return;
        const previous = result[result.length - 1];
        if (previous && Math.hypot(x - previous.x, y - previous.y) < 0.001) return;
        result.push({
          x, y,
          external_segment_id: segmentIndex,
          external_turn_node_id: segment.turn_node_id,
          external_speed_mps: finite(segment.speed_mps),
          source_index: pointIndex
        });
      });
    });
    if (result.length < 2) throw new Error('外部文件没有有效的 speed_segments[].points_mm');
    return result;
  }

  function transformPath(points, config) {
    const options = { ...(config || {}) };
    let source = points.slice();
    if (options.reverse) source.reverse();
    if (!source.length) return [];
    const origin = options.origin || source[0];
    let rotation = finite(options.rotationDeg) || 0;
    if (options.alignFirstSegment && source.length > 1) {
      const next = source.find((point) => Math.hypot(point.x - source[0].x, point.y - source[0].y) > 0.001);
      if (next) rotation -= Math.atan2(next.y - source[0].y, next.x - source[0].x) * 180 / Math.PI;
    }
    const radians = rotation * Math.PI / 180;
    const cosine = Math.cos(radians);
    const sine = Math.sin(radians);
    const scale = finite(options.scale) || 1;
    const targetOriginX = finite(options.targetOriginX) ?? 0;
    const targetOriginY = finite(options.targetOriginY) ?? 0;
    let distance = 0;
    let previous = null;
    return source.map((point) => {
      const localX = (point.x - origin.x) * scale;
      const localY = (point.y - origin.y) * scale * (options.mirrorY ? -1 : 1);
      const x = targetOriginX + localX * cosine - localY * sine;
      const y = targetOriginY + localX * sine + localY * cosine;
      const output = {
        ...point,
        x,
        y,
        // Keep both coordinate conventions canonical. The 5 mm resampler
        // accepts x_mm/y_mm first, so stale imported values must not survive.
        x_mm: x,
        y_mm: y
      };
      if (previous) distance += Math.hypot(output.x - previous.x, output.y - previous.y);
      output.s_mm = distance;
      previous = output;
      return output;
    });
  }

  function rotateClosedPath(points, startIndex) {
    if (startIndex <= 0 || startIndex >= points.length) return points;
    const first = points[0];
    const last = points[points.length - 1];
    const closed = Math.hypot(first.x - last.x, first.y - last.y) < 0.001;
    if (!closed) return points;
    const unique = points.slice(0, -1);
    if (startIndex >= unique.length) return points;
    const rotated = [...unique.slice(startIndex), ...unique.slice(0, startIndex)];
    return [...rotated, { ...rotated[0] }];
  }

  function sharedTransform(document, config, originId) {
    const referenceOrigin = document.nodeById.get(originId) || document.nodes[0];
    let reference = nodeRoute(document);
    if (config && config.startAtOrigin) {
      const startIndex = reference.findIndex((point) => String(point.external_node_id) === originId);
      reference = rotateClosedPath(reference, startIndex);
    }
    if (config && config.reverse) reference = reference.slice().reverse();
    let rotationDeg = finite(config && config.rotationDeg) || 0;
    if (config && config.alignFirstSegment && reference.length > 1) {
      const first = reference[0];
      const next = reference.find((point) => Math.hypot(point.x - first.x, point.y - first.y) > 0.001);
      if (next) rotationDeg -= Math.atan2(next.y - first.y, next.x - first.x) * 180 / Math.PI;
    }
    return { origin: referenceOrigin, rotationDeg };
  }

  function route(document, sourceName, config) {
    let points = sourceName === 'segments' ? segmentRoute(document) : nodeRoute(document);
    const originId = config && config.originNodeId ? String(config.originNodeId) : document.startNodeId;
    const origin = document.nodeById.get(originId) || points[0];
    if (config && config.startAtOrigin) {
      let startIndex = -1;
      if (sourceName === 'segments') {
        let bestDistance = Infinity;
        points.forEach((point, index) => {
          const distance = Math.hypot(point.x - origin.x, point.y - origin.y);
          if (distance < bestDistance) { bestDistance = distance; startIndex = index; }
        });
      } else {
        startIndex = points.findIndex((point) => String(point.external_node_id) === originId);
      }
      points = rotateClosedPath(points, startIndex);
    }
    const shared = sharedTransform(document, config, originId);
    return transformPath(points, {
      ...config,
      origin: shared.origin,
      rotationDeg: shared.rotationDeg,
      alignFirstSegment: false
    });
  }

  function networkNodes(document, sourceName, config) {
    const originId = config && config.originNodeId ? String(config.originNodeId) : document.startNodeId;
    const shared = sharedTransform(document, config, originId);
    return transformPath(document.nodes, {
      ...config,
      origin: shared.origin,
      rotationDeg: shared.rotationDeg,
      alignFirstSegment: false,
      reverse: false
    });
  }

  function networkSegments(document, sourceName, config) {
    if (!document.links.length) return [];
    const transformedNodes = networkNodes(document, sourceName, config);
    const transformedById = new Map(transformedNodes.map((node) => [node.id, node]));
    return document.links.map((link, index) => {
      const from = transformedById.get(link.from);
      const to = transformedById.get(link.to);
      if (!from || !to || Math.hypot(to.x - from.x, to.y - from.y) < 0.001) return null;
      return { from, to, source_index: index };
    }).filter(Boolean);
  }

  function offsetPoint(points, index, offset) {
    const point = points[index];
    const previous = points[Math.max(0, index - 1)];
    const next = points[Math.min(points.length - 1, index + 1)];
    let beforeX = point.x - previous.x;
    let beforeY = point.y - previous.y;
    let afterX = next.x - point.x;
    let afterY = next.y - point.y;
    let beforeLength = Math.hypot(beforeX, beforeY);
    let afterLength = Math.hypot(afterX, afterY);
    if (beforeLength < 1e-6) {
      beforeX = afterX; beforeY = afterY; beforeLength = afterLength;
    }
    if (afterLength < 1e-6) {
      afterX = beforeX; afterY = beforeY; afterLength = beforeLength;
    }
    if (beforeLength < 1e-6 || afterLength < 1e-6) return { x: point.x, y: point.y };
    const beforeNormal = { x: -beforeY / beforeLength, y: beforeX / beforeLength };
    const afterNormal = { x: -afterY / afterLength, y: afterX / afterLength };
    let miterX = beforeNormal.x + afterNormal.x;
    let miterY = beforeNormal.y + afterNormal.y;
    const miterLength = Math.hypot(miterX, miterY);
    if (miterLength < 1e-5) return { x: point.x + afterNormal.x * offset, y: point.y + afterNormal.y * offset };
    miterX /= miterLength;
    miterY /= miterLength;
    const projection = miterX * afterNormal.x + miterY * afterNormal.y;
    const scale = Math.abs(projection) > 0.05
      ? Math.max(-Math.abs(offset) * 3, Math.min(Math.abs(offset) * 3, offset / projection))
      : offset;
    return { x: point.x + miterX * scale, y: point.y + miterY * scale };
  }

  function buildOffsetBands(input, innerDistanceMm, widthMm) {
    const points = (Array.isArray(input) ? input : []).filter((point) =>
      Number.isFinite(Number(point && point.x)) && Number.isFinite(Number(point && point.y))
    );
    const inner = Math.max(0, finite(innerDistanceMm) ?? 0);
    const width = Math.max(0, finite(widthMm) ?? 0);
    if (points.length < 2 || width <= 0) return [];
    function side(sign) {
      const innerEdge = points.map((_, index) => offsetPoint(points, index, inner * sign));
      const outerEdge = points.map((_, index) => offsetPoint(points, index, (inner + width) * sign));
      return {
        side: sign > 0 ? 'left' : 'right',
        inner: innerEdge,
        outer: outerEdge,
        polygon: [...innerEdge, ...outerEdge.slice().reverse()]
      };
    }
    return [side(1), side(-1)];
  }

  function segmentIntersection(a, b) {
    const ax = a.to.x - a.from.x;
    const ay = a.to.y - a.from.y;
    const bx = b.to.x - b.from.x;
    const by = b.to.y - b.from.y;
    const denominator = ax * by - ay * bx;
    if (Math.abs(denominator) < 1e-8) return null;
    const dx = b.from.x - a.from.x;
    const dy = b.from.y - a.from.y;
    const ta = (dx * by - dy * bx) / denominator;
    const tb = (dx * ay - dy * ax) / denominator;
    if (ta < -1e-7 || ta > 1 + 1e-7 || tb < -1e-7 || tb > 1 + 1e-7) return null;
    return { x: a.from.x + ax * ta, y: a.from.y + ay * ta };
  }

  function coordinateKey(point) {
    return `${Math.round(point.x * 1000)},${Math.round(point.y * 1000)}`;
  }

  function splitNetworkSegments(input) {
    const segments = (Array.isArray(input) ? input : []).filter((segment) =>
      segment && segment.from && segment.to &&
      Number.isFinite(Number(segment.from.x)) && Number.isFinite(Number(segment.from.y)) &&
      Number.isFinite(Number(segment.to.x)) && Number.isFinite(Number(segment.to.y)) &&
      Math.hypot(segment.to.x - segment.from.x, segment.to.y - segment.from.y) > 1e-6
    );
    const splitPoints = segments.map((segment) => [segment.from, segment.to]);
    for (let first = 0; first < segments.length; first += 1) {
      for (let second = first + 1; second < segments.length; second += 1) {
        const intersection = segmentIntersection(segments[first], segments[second]);
        if (!intersection) continue;
        splitPoints[first].push(intersection);
        splitPoints[second].push(intersection);
      }
    }
    const vertices = new Map();
    const edges = new Map();
    segments.forEach((segment, segmentIndex) => {
      const dx = segment.to.x - segment.from.x;
      const dy = segment.to.y - segment.from.y;
      const lengthSquared = dx * dx + dy * dy;
      const points = splitPoints[segmentIndex].map((point) => ({
        x: Number(point.x),
        y: Number(point.y),
        t: ((point.x - segment.from.x) * dx + (point.y - segment.from.y) * dy) / lengthSquared
      })).sort((a, b) => a.t - b.t);
      const unique = points.filter((point, index) =>
        index === 0 || Math.hypot(point.x - points[index - 1].x, point.y - points[index - 1].y) > 1e-5
      );
      for (let index = 1; index < unique.length; index += 1) {
        const from = unique[index - 1];
        const to = unique[index];
        const fromKey = coordinateKey(from);
        const toKey = coordinateKey(to);
        if (fromKey === toKey) continue;
        vertices.set(fromKey, { x: from.x, y: from.y, key: fromKey });
        vertices.set(toKey, { x: to.x, y: to.y, key: toKey });
        const edgeKey = [fromKey, toKey].sort().join('|');
        edges.set(edgeKey, { from: fromKey, to: toKey });
      }
    });
    return { vertices, edges: [...edges.values()] };
  }

  function polygonArea(points) {
    let twiceArea = 0;
    points.forEach((point, index) => {
      const next = points[(index + 1) % points.length];
      twiceArea += point.x * next.y - next.x * point.y;
    });
    return twiceArea * 0.5;
  }

  function boundedNetworkFaces(input) {
    const graph = splitNetworkSegments(input);
    const neighbors = new Map([...graph.vertices.keys()].map((key) => [key, []]));
    graph.edges.forEach((edge) => {
      neighbors.get(edge.from).push(edge.to);
      neighbors.get(edge.to).push(edge.from);
    });
    neighbors.forEach((items, key) => {
      const origin = graph.vertices.get(key);
      items.sort((left, right) => {
        const a = graph.vertices.get(left);
        const b = graph.vertices.get(right);
        return Math.atan2(a.y - origin.y, a.x - origin.x) -
          Math.atan2(b.y - origin.y, b.x - origin.x);
      });
    });
    const visited = new Set();
    const faces = [];
    graph.edges.forEach((edge) => {
      [[edge.from, edge.to], [edge.to, edge.from]].forEach(([startFrom, startTo]) => {
        const startKey = `${startFrom}>${startTo}`;
        if (visited.has(startKey)) return;
        const keys = [];
        let from = startFrom;
        let to = startTo;
        for (let guard = 0; guard <= graph.edges.length * 2 + 2; guard += 1) {
          const directedKey = `${from}>${to}`;
          if (visited.has(directedKey)) break;
          visited.add(directedKey);
          keys.push(from);
          const options = neighbors.get(to) || [];
          const reverseIndex = options.indexOf(from);
          if (reverseIndex < 0 || !options.length) break;
          const next = options[(reverseIndex - 1 + options.length) % options.length];
          from = to;
          to = next;
          if (from === startFrom && to === startTo) {
            const points = keys.map((key) => graph.vertices.get(key));
            const area = polygonArea(points);
            if (points.length >= 3 && area > 1e-3) faces.push(points);
            break;
          }
        }
      });
    });
    return faces;
  }

  function pointSegmentDistance(point, from, to) {
    const dx = to.x - from.x;
    const dy = to.y - from.y;
    const lengthSquared = dx * dx + dy * dy;
    if (lengthSquared < 1e-12) return Math.hypot(point.x - from.x, point.y - from.y);
    const ratio = Math.max(0, Math.min(1,
      ((point.x - from.x) * dx + (point.y - from.y) * dy) / lengthSquared
    ));
    return Math.hypot(point.x - (from.x + dx * ratio), point.y - (from.y + dy * ratio));
  }

  function faceClearance(point, face) {
    let minimum = Infinity;
    face.forEach((from, index) => {
      minimum = Math.min(minimum, pointSegmentDistance(point, from, face[(index + 1) % face.length]));
    });
    return minimum;
  }

  function scanIntervals(face, station, horizontal) {
    const intersections = [];
    face.forEach((from, index) => {
      const to = face[(index + 1) % face.length];
      const first = horizontal ? from.x : from.y;
      const second = horizontal ? to.x : to.y;
      if (!((first <= station && second > station) || (second <= station && first > station))) return;
      const ratio = (station - first) / (second - first);
      intersections.push(horizontal
        ? from.y + (to.y - from.y) * ratio
        : from.x + (to.x - from.x) * ratio);
    });
    intersections.sort((left, right) => left - right);
    const intervals = [];
    for (let index = 1; index < intersections.length; index += 2) {
      if (intersections[index] - intersections[index - 1] > 1e-6) {
        intervals.push([intersections[index - 1], intersections[index]]);
      }
    }
    return intervals;
  }

  function segmentMaintainsClearance(from, to, face, requiredClearance) {
    const length = Math.hypot(to.x - from.x, to.y - from.y);
    const samples = Math.max(1, Math.ceil(length / 4));
    for (let index = 1; index < samples; index += 1) {
      const ratio = index / samples;
      const point = {
        x: from.x + (to.x - from.x) * ratio,
        y: from.y + (to.y - from.y) * ratio
      };
      if (faceClearance(point, face) + 0.25 < requiredClearance) return false;
    }
    return true;
  }

  function buildFaceShoulders(face, centerClearance) {
    const xs = face.map((point) => point.x);
    const ys = face.map((point) => point.y);
    const minX = Math.min(...xs);
    const maxX = Math.max(...xs);
    const minY = Math.min(...ys);
    const maxY = Math.max(...ys);
    const step = Math.max(5, Math.min(15, Math.max(maxX - minX, maxY - minY) / 300));

    const uniqueCenters = (horizontal) => {
      const minimum = horizontal ? minX : minY;
      const maximum = horizontal ? maxX : maxY;
      const centers = [];
      for (let station = minimum + step * 0.5; station < maximum; station += step) {
        scanIntervals(face, station, horizontal).forEach((interval) => {
          if (interval[1] - interval[0] + 0.25 < centerClearance * 2) return;
          const center = (interval[0] + interval[1]) * 0.5;
          if (!centers.some((value) => Math.abs(value - center) <= step * 0.5)) centers.push(center);
        });
      }
      return centers;
    };

    const linesAtCenters = (horizontal, centers) => {
      const minimum = horizontal ? minX : minY;
      const maximum = horizontal ? maxX : maxY;
      const lines = [];
      centers.forEach((cross) => {
        let run = [];
        let runMinimumGap = Infinity;
        const flush = () => {
          if (run.length >= 2) {
            const from = run[0];
            const to = run[run.length - 1];
            lines.push({
              points: [from, to],
              length: Math.hypot(to.x - from.x, to.y - from.y),
              crossGap: runMinimumGap,
              horizontal
            });
          }
          run = [];
          runMinimumGap = Infinity;
        };
        for (let station = minimum + step * 0.5; station < maximum; station += step) {
          const intervals = scanIntervals(face, station, horizontal);
          const containing = intervals.find((interval) =>
            cross >= interval[0] - 0.25 && cross <= interval[1] + 0.25
          );
          const point = horizontal ? { x: station, y: cross } : { x: cross, y: station };
          const valid = Boolean(containing) &&
            faceClearance(point, face) + 0.25 >= centerClearance &&
            (!run.length || segmentMaintainsClearance(run[run.length - 1], point, face, centerClearance));
          if (!valid) {
            flush();
            continue;
          }
          run.push(point);
          runMinimumGap = Math.min(runMinimumGap, containing[1] - containing[0]);
        }
        flush();
      });
      return lines;
    };

    const candidates = [
      ...linesAtCenters(true, uniqueCenters(true)),
      ...linesAtCenters(false, uniqueCenters(false))
    ].filter((line) => line.length > step);
    const longitudinal = candidates.filter((line) => line.length + step * 2 >= line.crossGap);
    const selected = longitudinal.length
      ? longitudinal
      : candidates.sort((left, right) => right.length - left.length).slice(0, 1);

    return selected
      .sort((left, right) => right.length - left.length)
      .filter((line, index, lines) => !lines.slice(0, index).some((other) =>
        line.horizontal === other.horizontal &&
        Math.abs((line.horizontal ? line.points[0].y : line.points[0].x) -
          (other.horizontal ? other.points[0].y : other.points[0].x)) <= step &&
        pointSegmentDistance(line.points[0], other.points[0], other.points[1]) <= step &&
        pointSegmentDistance(line.points[1], other.points[0], other.points[1]) <= step
      ))
      .map((line) => line.points);
  }

  function buildShoulderPaths(trackSegments, minimumClearanceMm, widthMm, maximumGapMm = 600) {
    const clearance = Math.max(0, finite(minimumClearanceMm) ?? 0);
    const width = Math.max(1, finite(widthMm) ?? 1);
    void width;
    void maximumGapMm;
    const centerOffset = clearance;
    return boundedNetworkFaces(trackSegments).flatMap((face, regionIndex) =>
      buildFaceShoulders(face, centerOffset)
        .map((points) => ({ regionIndex, points }))
    );
  }

  function shoulderPathSegments(paths) {
    return (Array.isArray(paths) ? paths : []).flatMap((path) => {
      const points = path && Array.isArray(path.points) ? path.points : [];
      if (points.length === 1) return [{ from: points[0], to: points[0], regionIndex: path.regionIndex }];
      return points.slice(1).map((point, index) => ({
        from: points[index],
        to: point,
        regionIndex: path.regionIndex
      }));
    });
  }

  return {
    parseDocument, nodeRoute, segmentRoute, transform: transformPath, route,
    rotateClosedPath, networkNodes, networkSegments, buildOffsetBands, boundedNetworkFaces,
    buildShoulderPaths, shoulderPathSegments
  };
}));
