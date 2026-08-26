(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  if (root) root.GeometryPlanner = api;
}(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  'use strict';

  const DEFAULTS = Object.freeze({
    sampleStepMm: 5,
    headingWindowMm: 60,
    straightAngleDeg: 3,
    straightResidualMm: 4,
    minStraightLengthMm: 140,
    bridgeGapMm: 35,
    mergeGapMm: 80,
    mergeAngleDeg: 10,
    minCornerAngleDeg: 15,
    maxCornerAngleDeg: 150,
    transitionLengthMm: 80,
    minRadiusMm: 60,
    maxRadiusMm: 1200,
    maxGeometryOffsetMm: 120,
    integrationStepMm: 1,
    globalControlSpacingMm: 35,
    globalLineWeight: 160,
    globalEndpointWeight: 1200,
    globalSecondDifferenceWeight: 12,
    globalThirdDifferenceWeight: 55,
    globalMaxIterations: 260,
    minimumRadiusMm: 0,
    corridorControlSpacingMm: 100,
    corridorOffsetPenalty: 0.04,
    corridorSmoothnessPenalty: 0.8,
    corridorSweepsPerStep: 2
  });

  const clamp = (value, min, max) => Math.max(min, Math.min(max, value));
  const hypot = (x, y) => Math.sqrt(x * x + y * y);
  const cross = (a, b) => a.x * b.y - a.y * b.x;
  const dot = (a, b) => a.x * b.x + a.y * b.y;

  function wrapPi(angle) {
    while (angle > Math.PI) angle -= Math.PI * 2;
    while (angle < -Math.PI) angle += Math.PI * 2;
    return angle;
  }

  function cleanPoints(input) {
    const result = [];
    for (const raw of Array.isArray(input) ? input : []) {
      const x = Number(raw && raw.x);
      const y = Number(raw && raw.y);
      if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
      const previous = result[result.length - 1];
      if (previous && hypot(x - previous.x, y - previous.y) < 0.05) continue;
      result.push({ ...raw, x, y });
    }
    return result;
  }

  function resample(input, stepMm) {
    const source = cleanPoints(input);
    if (source.length < 2) return source;
    const cumulative = [0];
    for (let index = 1; index < source.length; index += 1) {
      cumulative.push(cumulative[index - 1] + hypot(
        source[index].x - source[index - 1].x,
        source[index].y - source[index - 1].y
      ));
    }
    const total = cumulative[cumulative.length - 1];
    if (total < stepMm) return [source[0], source[source.length - 1]];
    const output = [];
    let sourceIndex = 1;
    for (let distance = 0; distance < total; distance += stepMm) {
      while (sourceIndex < cumulative.length - 1 && cumulative[sourceIndex] < distance) sourceIndex += 1;
      const startDistance = cumulative[sourceIndex - 1];
      const endDistance = cumulative[sourceIndex];
      const ratio = endDistance > startDistance ? (distance - startDistance) / (endDistance - startDistance) : 0;
      const a = source[sourceIndex - 1];
      const b = source[sourceIndex];
      output.push({
        x: a.x + (b.x - a.x) * ratio,
        y: a.y + (b.y - a.y) * ratio,
        sourceDistanceMm: distance,
        sourceIndex: sourceIndex - 1 + ratio
      });
    }
    output.push({
      ...source[source.length - 1],
      sourceDistanceMm: total,
      sourceIndex: source.length - 1
    });
    return output;
  }

  function pointLineDistance(point, a, b) {
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const length = hypot(dx, dy);
    if (length < 1e-6) return hypot(point.x - a.x, point.y - a.y);
    return Math.abs(dx * (a.y - point.y) - (a.x - point.x) * dy) / length;
  }

  function classifyStraight(points, options) {
    const span = Math.max(2, Math.round(options.headingWindowMm / options.sampleStepMm));
    const threshold = options.straightAngleDeg * Math.PI / 180;
    const flags = new Array(points.length).fill(false);
    for (let index = span; index + span < points.length; index += 1) {
      const before = points[index - span];
      const center = points[index];
      const after = points[index + span];
      const incoming = Math.atan2(center.y - before.y, center.x - before.x);
      const outgoing = Math.atan2(after.y - center.y, after.x - center.x);
      const angle = Math.abs(wrapPi(outgoing - incoming));
      const residual = pointLineDistance(center, before, after);
      const quarterA = points[index - Math.floor(span / 2)];
      const quarterB = points[index + Math.floor(span / 2)];
      const maxResidual = Math.max(
        residual,
        pointLineDistance(quarterA, before, after),
        pointLineDistance(quarterB, before, after)
      );
      flags[index] = angle <= threshold && maxResidual <= options.straightResidualMm;
    }

    const bridge = Math.max(1, Math.round(options.bridgeGapMm / options.sampleStepMm));
    let index = 0;
    while (index < flags.length) {
      if (flags[index]) { index += 1; continue; }
      const start = index;
      while (index < flags.length && !flags[index]) index += 1;
      if (start > 0 && index < flags.length && index - start <= bridge) {
        for (let fill = start; fill < index; fill += 1) flags[fill] = true;
      }
    }
    return flags;
  }

  function fitLine(points, start, end) {
    let meanX = 0;
    let meanY = 0;
    const count = end - start + 1;
    for (let index = start; index <= end; index += 1) {
      meanX += points[index].x;
      meanY += points[index].y;
    }
    meanX /= count;
    meanY /= count;
    let xx = 0;
    let xy = 0;
    let yy = 0;
    let maxResidual = 0;
    for (let index = start; index <= end; index += 1) {
      const x = points[index].x - meanX;
      const y = points[index].y - meanY;
      xx += x * x;
      xy += x * y;
      yy += y * y;
    }
    const angle = 0.5 * Math.atan2(2 * xy, xx - yy);
    let direction = { x: Math.cos(angle), y: Math.sin(angle) };
    const pathDirection = {
      x: points[end].x - points[start].x,
      y: points[end].y - points[start].y
    };
    if (dot(direction, pathDirection) < 0) direction = { x: -direction.x, y: -direction.y };
    const origin = { x: meanX, y: meanY };
    for (let index = start; index <= end; index += 1) {
      maxResidual = Math.max(maxResidual, pointLineDistance(
        points[index], origin, { x: origin.x + direction.x, y: origin.y + direction.y }
      ));
    }
    return { start, end, origin, direction, maxResidual };
  }

  function detectLines(points, flags, options) {
    const minimum = Math.max(3, Math.round(options.minStraightLengthMm / options.sampleStepMm));
    const lines = [];
    let index = 0;
    while (index < flags.length) {
      while (index < flags.length && !flags[index]) index += 1;
      const start = index;
      while (index < flags.length && flags[index]) index += 1;
      const end = index - 1;
      if (end - start + 1 >= minimum) lines.push(fitLine(points, start, end));
    }

    const mergeGap = Math.round(options.mergeGapMm / options.sampleStepMm);
    const mergeAngle = options.mergeAngleDeg * Math.PI / 180;
    const merged = [];
    for (const line of lines) {
      const previous = merged[merged.length - 1];
      if (previous && line.start - previous.end <= mergeGap &&
          Math.abs(wrapPi(Math.atan2(line.direction.y, line.direction.x) -
            Math.atan2(previous.direction.y, previous.direction.x))) <= mergeAngle) {
        merged[merged.length - 1] = fitLine(points, previous.start, line.end);
      } else {
        merged.push(line);
      }
    }
    return merged;
  }

  function lineIntersection(a, b) {
    const denominator = cross(a.direction, b.direction);
    if (Math.abs(denominator) < 1e-5) return null;
    const delta = { x: b.origin.x - a.origin.x, y: b.origin.y - a.origin.y };
    const alongA = cross(delta, b.direction) / denominator;
    return {
      x: a.origin.x + a.direction.x * alongA,
      y: a.origin.y + a.direction.y * alongA
    };
  }

  function integrateProfile(turnAngle, radius, transitionLength, integrationStep, collect) {
    const arcAngle = Math.max(0, turnAngle - transitionLength / radius);
    const arcLength = radius * arcAngle;
    const totalLength = transitionLength * 2 + arcLength;
    const count = Math.max(1, Math.ceil(totalLength / integrationStep));
    const ds = totalLength / count;
    let x = 0;
    let y = 0;
    let heading = 0;
    const samples = collect ? [{ x, y, heading, curvature: 0, distance: 0, phase: 'clothoid_in' }] : null;
    for (let index = 0; index < count; index += 1) {
      const midpoint = (index + 0.5) * ds;
      let curvature;
      let phase;
      if (midpoint < transitionLength) {
        curvature = midpoint / (radius * transitionLength);
        phase = 'clothoid_in';
      } else if (midpoint < transitionLength + arcLength) {
        curvature = 1 / radius;
        phase = 'arc';
      } else {
        curvature = Math.max(0, (totalLength - midpoint) / (radius * transitionLength));
        phase = 'clothoid_out';
      }
      const middleHeading = heading + curvature * ds * 0.5;
      x += Math.cos(middleHeading) * ds;
      y += Math.sin(middleHeading) * ds;
      heading += curvature * ds;
      if (collect) samples.push({ x, y, heading, curvature, distance: (index + 1) * ds, phase });
    }
    const cosine = Math.cos(turnAngle);
    const sine = Math.sin(turnAngle);
    const trimX = x / Math.max(1e-6, 1 + cosine);
    const trimY = Math.abs(sine) > 1e-5 ? y / sine : trimX;
    return { x, y, heading, trim: (trimX + trimY) * 0.5, totalLength, arcLength, samples };
  }

  function solveCornerProfile(turnAngle, targetTrim, options) {
    let transitionLength = Math.min(options.transitionLengthMm, targetTrim * 0.62);
    transitionLength = Math.max(options.sampleStepMm * 2, transitionLength);
    let minimumRadius = Math.max(options.minRadiusMm, transitionLength / turnAngle * 1.001);
    let maximumRadius = options.maxRadiusMm;
    let minimum = integrateProfile(turnAngle, minimumRadius, transitionLength, options.integrationStepMm, false);
    if (minimum.trim > targetTrim) {
      transitionLength *= clamp(targetTrim / minimum.trim * 0.9, 0.25, 1);
      minimumRadius = Math.max(options.minRadiusMm, transitionLength / turnAngle * 1.001);
      minimum = integrateProfile(turnAngle, minimumRadius, transitionLength, options.integrationStepMm, false);
    }
    const maximum = integrateProfile(turnAngle, maximumRadius, transitionLength, options.integrationStepMm, false);
    if (targetTrim < minimum.trim || targetTrim > maximum.trim) return null;
    for (let iteration = 0; iteration < 36; iteration += 1) {
      const middleRadius = (minimumRadius + maximumRadius) * 0.5;
      const profile = integrateProfile(turnAngle, middleRadius, transitionLength, options.integrationStepMm, false);
      if (profile.trim < targetTrim) minimumRadius = middleRadius;
      else maximumRadius = middleRadius;
    }
    const radius = (minimumRadius + maximumRadius) * 0.5;
    return {
      radius,
      transitionLength,
      profile: integrateProfile(turnAngle, radius, transitionLength, options.integrationStepMm, true)
    };
  }

  function project(point, line) {
    const along = (point.x - line.origin.x) * line.direction.x +
      (point.y - line.origin.y) * line.direction.y;
    return {
      x: line.origin.x + line.direction.x * along,
      y: line.origin.y + line.direction.y * along,
      along
    };
  }

  function buildCorner(points, incoming, outgoing, options) {
    const intersection = lineIntersection(incoming, outgoing);
    if (!intersection) return { valid: false, reason: 'parallel_lines' };
    const signedAngle = wrapPi(
      Math.atan2(outgoing.direction.y, outgoing.direction.x) -
      Math.atan2(incoming.direction.y, incoming.direction.x)
    );
    const turnAngle = Math.abs(signedAngle);
    const minAngle = options.minCornerAngleDeg * Math.PI / 180;
    const maxAngle = options.maxCornerAngleDeg * Math.PI / 180;
    if (turnAngle < minAngle || turnAngle > maxAngle) return { valid: false, reason: 'corner_angle' };

    const incomingBoundary = project(points[incoming.end], incoming);
    const outgoingBoundary = project(points[outgoing.start], outgoing);
    const toIntersection = {
      x: intersection.x - incomingBoundary.x,
      y: intersection.y - incomingBoundary.y
    };
    const fromIntersection = {
      x: outgoingBoundary.x - intersection.x,
      y: outgoingBoundary.y - intersection.y
    };
    const availableIn = dot(toIntersection, incoming.direction);
    const availableOut = dot(fromIntersection, outgoing.direction);
    if (availableIn <= options.sampleStepMm * 2 || availableOut <= options.sampleStepMm * 2) {
      return { valid: false, reason: 'negative_trim' };
    }
    const targetTrim = Math.min(
      (availableIn + availableOut) * 0.5,
      availableIn * 0.96,
      availableOut * 0.96
    );
    const solved = solveCornerProfile(turnAngle, targetTrim, options);
    if (!solved) return { valid: false, reason: 'profile_unsolved' };

    const trim = solved.profile.trim;
    const pin = {
      x: intersection.x - incoming.direction.x * trim,
      y: intersection.y - incoming.direction.y * trim
    };
    const pout = {
      x: intersection.x + outgoing.direction.x * trim,
      y: intersection.y + outgoing.direction.y * trim
    };
    const sign = signedAngle >= 0 ? 1 : -1;
    const heading = Math.atan2(incoming.direction.y, incoming.direction.x);
    const cosine = Math.cos(heading);
    const sine = Math.sin(heading);
    const samples = solved.profile.samples.map((sample) => ({
      x: pin.x + cosine * sample.x - sine * sample.y * sign,
      y: pin.y + sine * sample.x + cosine * sample.y * sign,
      theta_rad: wrapPi(heading + sample.heading * sign),
      theta_deg: wrapPi(heading + sample.heading * sign) * 180 / Math.PI,
      curvature_per_mm: sample.curvature * sign,
      radius_mm: sample.curvature > 1e-8 ? 1 / sample.curvature : null,
      geometryType: sample.phase,
      geometryDistanceMm: sample.distance
    }));
    const endpointError = hypot(
      samples[samples.length - 1].x - pout.x,
      samples[samples.length - 1].y - pout.y
    );
    if (endpointError > Math.max(2, options.sampleStepMm * 0.6)) {
      return { valid: false, reason: 'integration_error' };
    }
    let maxSourceOffset = 0;
    const sourceStart = Math.max(0, incoming.end - 4);
    const sourceEnd = Math.min(points.length - 1, outgoing.start + 4);
    for (const sample of samples) {
      let nearest = Infinity;
      for (let index = sourceStart; index <= sourceEnd; index += 1) {
        nearest = Math.min(nearest, hypot(sample.x - points[index].x, sample.y - points[index].y));
      }
      maxSourceOffset = Math.max(maxSourceOffset, nearest);
    }
    if (maxSourceOffset > options.maxGeometryOffsetMm) {
      return { valid: false, reason: 'geometry_offset', maxSourceOffsetMm: maxSourceOffset };
    }
    return {
      valid: true,
      intersection,
      pin,
      pout,
      samples,
      radiusMm: solved.radius,
      transitionLengthMm: solved.transitionLength,
      turnAngleDeg: signedAngle * 180 / Math.PI,
      endpointErrorMm: endpointError,
      maxSourceOffsetMm: maxSourceOffset,
      sourceStart: incoming.end,
      sourceEnd: outgoing.start
    };
  }

  function appendLine(output, start, end, stepMm, segmentIndex) {
    const dx = end.x - start.x;
    const dy = end.y - start.y;
    const length = hypot(dx, dy);
    if (length < 0.1) return;
    const count = Math.max(1, Math.ceil(length / stepMm));
    const heading = Math.atan2(dy, dx);
    for (let index = output.length ? 1 : 0; index <= count; index += 1) {
      const ratio = index / count;
      output.push({
        x: start.x + dx * ratio,
        y: start.y + dy * ratio,
        theta_rad: heading,
        theta_deg: heading * 180 / Math.PI,
        curvature_per_mm: 0,
        radius_mm: null,
        geometryType: 'line',
        geometrySegment: segmentIndex
      });
    }
  }

  function appendFallback(output, points, start, end, segmentIndex) {
    for (let index = start; index <= end && index < points.length; index += 1) {
      const previous = points[Math.max(0, index - 1)];
      const next = points[Math.min(points.length - 1, index + 1)];
      const heading = Math.atan2(next.y - previous.y, next.x - previous.x);
      output.push({
        ...points[index], theta_rad: heading, theta_deg: heading * 180 / Math.PI,
        curvature_per_mm: null, radius_mm: null,
        geometryType: 'fallback', geometrySegment: segmentIndex
      });
    }
  }

  function summarizeOffsets(planned, source) {
    const cellSize = 50;
    const grid = new Map();
    source.forEach((point) => {
      const key = `${Math.floor(point.x / cellSize)},${Math.floor(point.y / cellSize)}`;
      if (!grid.has(key)) grid.set(key, []);
      grid.get(key).push(point);
    });
    let max = 0;
    let sum = 0;
    let count = 0;
    for (const point of planned) {
      const cellX = Math.floor(point.x / cellSize);
      const cellY = Math.floor(point.y / cellSize);
      let nearest = Infinity;
      for (let dx = -2; dx <= 2; dx += 1) {
        for (let dy = -2; dy <= 2; dy += 1) {
          const candidates = grid.get(`${cellX + dx},${cellY + dy}`) || [];
          for (const candidate of candidates) {
            nearest = Math.min(nearest, hypot(point.x - candidate.x, point.y - candidate.y));
          }
        }
      }
      if (!Number.isFinite(nearest)) {
        for (const candidate of source) {
          nearest = Math.min(nearest, hypot(point.x - candidate.x, point.y - candidate.y));
        }
      }
      max = Math.max(max, nearest);
      sum += nearest;
      count += 1;
    }
    return { maxOffsetMm: max, meanOffsetMm: count ? sum / count : 0 };
  }

  function plan(input, overrides) {
    const options = { ...DEFAULTS, ...(overrides || {}) };
    const source = resample(input, options.sampleStepMm);
    if (source.length < 8) {
      return { points: [], source, lines: [], corners: [], warnings: ['路径点不足'], options };
    }
    const flags = classifyStraight(source, options);
    const lines = detectLines(source, flags, options);
    const corners = [];
    for (let index = 0; index + 1 < lines.length; index += 1) {
      corners.push(buildCorner(source, lines[index], lines[index + 1], options));
    }

    const output = [];
    const warnings = [];
    if (!lines.length) {
      appendFallback(output, source, 0, source.length - 1, 0);
      warnings.push('没有检测到足够长的直线段');
    } else {
      if (lines[0].start > 0) appendFallback(output, source, 0, lines[0].start, 0);
      for (let index = 0; index < lines.length; index += 1) {
        const line = lines[index];
        const previousCorner = index > 0 ? corners[index - 1] : null;
        const nextCorner = index < corners.length ? corners[index] : null;
        const start = previousCorner && previousCorner.valid
          ? previousCorner.pout : project(source[line.start], line);
        const end = nextCorner && nextCorner.valid
          ? nextCorner.pin : project(source[line.end], line);
        appendLine(output, start, end, options.sampleStepMm, index);
        if (nextCorner) {
          if (nextCorner.valid) {
            let lastOutputDistance = 0;
            for (let pointIndex = 1; pointIndex < nextCorner.samples.length; pointIndex += 1) {
              const sample = nextCorner.samples[pointIndex];
              const isLast = pointIndex === nextCorner.samples.length - 1;
              if (!isLast && sample.geometryDistanceMm - lastOutputDistance < options.sampleStepMm) continue;
              output.push({ ...sample, geometrySegment: index });
              lastOutputDistance = sample.geometryDistanceMm;
            }
          } else {
            appendFallback(output, source, line.end + 1, lines[index + 1].start - 1, index);
            warnings.push(`弯道 ${index + 1} 未重建：${nextCorner.reason}`);
          }
        }
      }
      const lastLine = lines[lines.length - 1];
      if (lastLine.end < source.length - 1) {
        appendFallback(output, source, lastLine.end + 1, source.length - 1, lines.length);
      }
    }
    const offsets = summarizeOffsets(output, source);
    if (offsets.maxOffsetMm > options.maxGeometryOffsetMm) {
      warnings.push(`最大偏移 ${offsets.maxOffsetMm.toFixed(1)}mm 超过限制 ${options.maxGeometryOffsetMm}mm`);
    }
    return {
      points: output,
      source,
      lines,
      corners,
      warnings,
      options,
      stats: {
        sourcePointCount: source.length,
        plannedPointCount: output.length,
        lineCount: lines.length,
        cornerCount: corners.filter((corner) => corner.valid).length,
        failedCornerCount: corners.filter((corner) => !corner.valid).length,
        ...offsets
      }
    };
  }

  function splineBasisAt(sampleIndex, stride, controlCount) {
    const parameter = sampleIndex / stride;
    const base = Math.floor(parameter);
    const u = parameter - base;
    const u2 = u * u;
    const u3 = u2 * u;
    const weights = [
      ((1 - u) * (1 - u) * (1 - u)) / 6,
      (3 * u3 - 6 * u2 + 4) / 6,
      (-3 * u3 + 3 * u2 + 3 * u + 1) / 6,
      u3 / 6
    ];
    const indices = [base, base + 1, base + 2, base + 3]
      .map((index) => clamp(index, 0, controlCount - 1));
    return { indices, weights };
  }

  function dotArray(a, b) {
    let result = 0;
    for (let index = 0; index < a.length; index += 1) result += a[index] * b[index];
    return result;
  }

  function conjugateGradient(apply, rhs, initial, maxIterations) {
    const x = initial.slice();
    const ax = apply(x);
    const residual = rhs.map((value, index) => value - ax[index]);
    const direction = residual.slice();
    let residualNorm = dotArray(residual, residual);
    const initialNorm = Math.max(residualNorm, 1e-20);
    let iterations = 0;
    for (; iterations < maxIterations && residualNorm > initialNorm * 1e-12; iterations += 1) {
      const appliedDirection = apply(direction);
      const denominator = dotArray(direction, appliedDirection);
      if (Math.abs(denominator) < 1e-20) break;
      const alpha = residualNorm / denominator;
      for (let index = 0; index < x.length; index += 1) {
        x[index] += alpha * direction[index];
        residual[index] -= alpha * appliedDirection[index];
      }
      const nextNorm = dotArray(residual, residual);
      const beta = nextNorm / Math.max(residualNorm, 1e-20);
      for (let index = 0; index < direction.length; index += 1) {
        direction[index] = residual[index] + beta * direction[index];
      }
      residualNorm = nextNorm;
    }
    return { values: x, iterations, residual: Math.sqrt(residualNorm / initialNorm) };
  }

  function globalSplineSolve(source, lines, options, secondWeight, thirdWeight) {
    const stride = Math.max(2, Math.round(options.globalControlSpacingMm / options.sampleStepMm));
    const controlCount = Math.ceil((source.length - 1) / stride) + 4;
    const basis = source.map((_, index) => splineBasisAt(index, stride, controlCount));
    const lineTargets = new Array(source.length).fill(null);
    lines.forEach((line) => {
      for (let index = line.start; index <= line.end; index += 1) {
        lineTargets[index] = project(source[index], line);
      }
    });

    function solveCoordinate(key) {
      const rhs = new Array(controlCount).fill(0);
      function appendRow(sampleIndex, weight, target) {
        const row = basis[sampleIndex];
        for (let local = 0; local < 4; local += 1) {
          rhs[row.indices[local]] += weight * row.weights[local] * target;
        }
      }
      source.forEach((point, index) => {
        appendRow(index, 1, point[key]);
        if (lineTargets[index]) appendRow(index, options.globalLineWeight, lineTargets[index][key]);
      });
      appendRow(0, options.globalEndpointWeight, source[0][key]);
      appendRow(source.length - 1, options.globalEndpointWeight, source[source.length - 1][key]);

      function apply(values) {
        const output = new Array(controlCount).fill(0);
        function applyRow(sampleIndex, weight) {
          const row = basis[sampleIndex];
          let prediction = 0;
          for (let local = 0; local < 4; local += 1) {
            prediction += row.weights[local] * values[row.indices[local]];
          }
          for (let local = 0; local < 4; local += 1) {
            output[row.indices[local]] += weight * row.weights[local] * prediction;
          }
        }
        source.forEach((_, index) => {
          applyRow(index, 1);
          if (lineTargets[index]) applyRow(index, options.globalLineWeight);
        });
        applyRow(0, options.globalEndpointWeight);
        applyRow(source.length - 1, options.globalEndpointWeight);
        for (let index = 0; index + 2 < controlCount; index += 1) {
          const delta = values[index] - 2 * values[index + 1] + values[index + 2];
          output[index] += secondWeight * delta;
          output[index + 1] -= 2 * secondWeight * delta;
          output[index + 2] += secondWeight * delta;
        }
        for (let index = 0; index + 3 < controlCount; index += 1) {
          const delta = -values[index] + 3 * values[index + 1] -
            3 * values[index + 2] + values[index + 3];
          output[index] -= thirdWeight * delta;
          output[index + 1] += 3 * thirdWeight * delta;
          output[index + 2] -= 3 * thirdWeight * delta;
          output[index + 3] += thirdWeight * delta;
        }
        for (let index = 0; index < controlCount; index += 1) output[index] += values[index] * 1e-8;
        return output;
      }

      const initial = new Array(controlCount).fill(0).map((_, index) => {
        const sourceIndex = clamp(Math.round((index - 1.5) * stride), 0, source.length - 1);
        return source[sourceIndex][key];
      });
      return conjugateGradient(apply, rhs, initial, options.globalMaxIterations);
    }

    const solvedX = solveCoordinate('x');
    const solvedY = solveCoordinate('y');
    const points = basis.map((row) => {
      let x = 0;
      let y = 0;
      for (let local = 0; local < 4; local += 1) {
        x += row.weights[local] * solvedX.values[row.indices[local]];
        y += row.weights[local] * solvedY.values[row.indices[local]];
      }
      return { x, y };
    });
    return {
      points,
      lineTargets,
      controlCount,
      iterations: Math.max(solvedX.iterations, solvedY.iterations),
      residual: Math.max(solvedX.residual, solvedY.residual),
      secondWeight,
      thirdWeight
    };
  }

  function annotateGlobalGeometry(points, lineTargets, stepMm) {
    const output = points.map((point) => ({ ...point }));
    for (let index = 0; index < output.length; index += 1) {
      const before = output[Math.max(0, index - 1)];
      const after = output[Math.min(output.length - 1, index + 1)];
      const heading = Math.atan2(after.y - before.y, after.x - before.x);
      output[index].theta_rad = heading;
      output[index].theta_deg = heading * 180 / Math.PI;
      output[index].geometryType = lineTargets[index] ? 'line' : 'curvature_spline';
      output[index].geometrySegment = index;
      if (index === 0 || index === output.length - 1) {
        output[index].curvature_per_mm = 0;
        output[index].radius_mm = null;
        continue;
      }
      const a = output[index - 1];
      const b = output[index];
      const c = output[index + 1];
      const ab = hypot(b.x - a.x, b.y - a.y);
      const bc = hypot(c.x - b.x, c.y - b.y);
      const ac = hypot(c.x - a.x, c.y - a.y);
      const denominator = ab * bc * ac;
      const curvature = denominator > 1e-8
        ? (2 * ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x))) / denominator
        : 0;
      output[index].curvature_per_mm = curvature;
      output[index].radius_mm = Math.abs(curvature) > 1e-8 ? 1 / Math.abs(curvature) : null;
    }
    return output;
  }

  function planGlobal(input, overrides) {
    const options = { ...DEFAULTS, ...(overrides || {}) };
    const source = resample(input, options.sampleStepMm);
    if (source.length < 8) {
      return { points: [], source, lines: [], warnings: ['路径点不足'], options, algorithm: 'global_spline' };
    }
    const flags = classifyStraight(source, options);
    const lines = detectLines(source, flags, options);
    let secondWeight = options.globalSecondDifferenceWeight;
    let thirdWeight = options.globalThirdDifferenceWeight;
    let solved = null;
    let annotated = null;
    let offsets = null;
    let rejectedOffsetMm = null;
    const requestedMinimumRadiusMm = Number(options.minimumRadiusMm) > 0
      ? Number(options.minimumRadiusMm)
      : 0;
    let achievedMinimumRadiusMm = null;
    let curvatureLimitMet = requestedMinimumRadiusMm <= 0;
    let smoothingScale = 1;

    function evaluateCandidate(scale) {
      const candidateSolved = globalSplineSolve(
        source,
        lines,
        options,
        options.globalSecondDifferenceWeight * scale,
        options.globalThirdDifferenceWeight * scale
      );
      const candidatePoints = annotateGlobalGeometry(
        candidateSolved.points,
        candidateSolved.lineTargets,
        options.sampleStepMm
      );
      const candidateOffsets = summarizeOffsets(candidatePoints, source);
      let maximumCurvature = 0;
      for (const point of candidatePoints) {
        maximumCurvature = Math.max(maximumCurvature, Math.abs(point.curvature_per_mm || 0));
      }
      return {
        solved: candidateSolved,
        annotated: candidatePoints,
        offsets: candidateOffsets,
        maximumCurvature,
        minimumRadiusMm: maximumCurvature > 1e-10 ? 1 / maximumCurvature : Infinity,
        scale
      };
    }

    if (requestedMinimumRadiusMm > 0) {
      let bestCandidate = null;
      let previousScale = 1;
      for (const scale of [
        1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192,
        16384, 32768, 65536, 131072, 262144
      ]) {
        const candidate = evaluateCandidate(scale);
        if (candidate.offsets.maxOffsetMm > options.maxGeometryOffsetMm) continue;
        if (!bestCandidate || candidate.minimumRadiusMm > bestCandidate.minimumRadiusMm) {
          bestCandidate = candidate;
        }
        if (candidate.minimumRadiusMm >= requestedMinimumRadiusMm) {
          let lowerScale = previousScale;
          let upperScale = scale;
          let closestCandidate = candidate;
          for (let iteration = 0; iteration < 12; iteration += 1) {
            const middleScale = (lowerScale + upperScale) * 0.5;
            const middleCandidate = evaluateCandidate(middleScale);
            const middleValid = middleCandidate.offsets.maxOffsetMm <= options.maxGeometryOffsetMm &&
              middleCandidate.minimumRadiusMm >= requestedMinimumRadiusMm;
            if (middleValid) {
              closestCandidate = middleCandidate;
              upperScale = middleScale;
            } else {
              lowerScale = middleScale;
            }
          }
          bestCandidate = closestCandidate;
          curvatureLimitMet = true;
          break;
        }
        previousScale = scale;
      }
      if (bestCandidate) {
        ({ solved, annotated, offsets } = bestCandidate);
        achievedMinimumRadiusMm = bestCandidate.minimumRadiusMm;
        smoothingScale = bestCandidate.scale;
      } else {
        solved = globalSplineSolve(source, lines, options, secondWeight, thirdWeight);
        annotated = annotateGlobalGeometry(solved.points, solved.lineTargets, options.sampleStepMm);
        offsets = summarizeOffsets(annotated, source);
      }
    } else {
      for (let attempt = 0; attempt < 5; attempt += 1) {
        solved = globalSplineSolve(source, lines, options, secondWeight, thirdWeight);
        annotated = annotateGlobalGeometry(solved.points, solved.lineTargets, options.sampleStepMm);
        offsets = summarizeOffsets(annotated, source);
        if (offsets.maxOffsetMm <= options.maxGeometryOffsetMm) break;
        secondWeight *= 0.35;
        thirdWeight *= 0.35;
      }
      let maximumCurvature = 0;
      for (const point of annotated) {
        maximumCurvature = Math.max(maximumCurvature, Math.abs(point.curvature_per_mm || 0));
      }
      achievedMinimumRadiusMm = maximumCurvature > 1e-10 ? 1 / maximumCurvature : Infinity;
    }
    const warnings = [];
    if (requestedMinimumRadiusMm > 0 && !curvatureLimitMet && achievedMinimumRadiusMm !== null) {
      warnings.push(
        `目标最小半径 ${requestedMinimumRadiusMm.toFixed(1)}mm 在 ` +
        `${options.maxGeometryOffsetMm}mm 偏移限制内不可满足；最佳为 ` +
        `${achievedMinimumRadiusMm.toFixed(1)}mm`
      );
    }
    if (offsets.maxOffsetMm > options.maxGeometryOffsetMm) {
      const rejectedOffset = offsets.maxOffsetMm;
      rejectedOffsetMm = rejectedOffset;
      annotated = annotateGlobalGeometry(
        source.map((point) => ({ x: point.x, y: point.y })),
        new Array(source.length).fill(null),
        options.sampleStepMm
      ).map((point) => ({ ...point, geometryType: 'fallback' }));
      offsets = summarizeOffsets(annotated, source);
      warnings.push(
        `全局样条最大偏移 ${rejectedOffset.toFixed(1)}mm 超过限制 ` +
        `${options.maxGeometryOffsetMm}mm，已回退到原路径`
      );
    }
    return {
      algorithm: 'global_spline',
      points: annotated,
      source,
      lines,
      corners: [],
      warnings,
      options,
      stats: {
        sourcePointCount: source.length,
        plannedPointCount: annotated.length,
        lineCount: lines.length,
        cornerCount: 0,
        failedCornerCount: 0,
        controlCount: solved.controlCount,
        iterations: solved.iterations,
        solverResidual: solved.residual,
        secondWeight: solved.secondWeight,
        thirdWeight: solved.thirdWeight,
        smoothingScale,
        requestedMinimumRadiusMm,
        achievedMinimumRadiusMm,
        curvatureLimitMet,
        rejected: rejectedOffsetMm !== null,
        rejectedOffsetMm,
        ...offsets
      }
    };
  }

  function planCorridor(input, overrides) {
    const options = { ...DEFAULTS, ...(overrides || {}) };
    const requestedMinimumRadiusMm = Number(options.minimumRadiusMm) > 0
      ? Number(options.minimumRadiusMm)
      : 0;
    const baseline = planGlobal(input, { ...options, minimumRadiusMm: 0 });
    if (baseline.points.length < 8 || requestedMinimumRadiusMm <= 0) {
      return { ...baseline, algorithm: 'corridor_optimizer' };
    }

    const source = baseline.source;
    const base = baseline.points;
    const count = base.length;
    const stride = Math.max(3, Math.round(options.corridorControlSpacingMm / options.sampleStepMm));
    const controlCount = Math.ceil((count - 1) / stride) + 4;
    const basis = base.map((_, index) => splineBasisAt(index, stride, controlCount));
    const controls = new Float64Array(controlCount);
    const normalX = new Float64Array(count);
    const normalY = new Float64Array(count);
    const candidateX = new Float64Array(count);
    const candidateY = new Float64Array(count);
    const targetCurvature = 1 / requestedMinimumRadiusMm;
    const maxOffset = options.maxGeometryOffsetMm;

    for (let index = 0; index < count; index += 1) {
      const before = base[Math.max(0, index - 1)];
      const after = base[Math.min(count - 1, index + 1)];
      const length = hypot(after.x - before.x, after.y - before.y) || 1;
      normalX[index] = -(after.y - before.y) / length;
      normalY[index] = (after.x - before.x) / length;
    }

    function evaluate(collectPoints) {
      let offsetCost = 0;
      let corridorViolation = 0;
      for (let index = 0; index < count; index += 1) {
        const row = basis[index];
        let lateral = 0;
        for (let local = 0; local < 4; local += 1) {
          lateral += row.weights[local] * controls[row.indices[local]];
        }
        candidateX[index] = base[index].x + normalX[index] * lateral;
        candidateY[index] = base[index].y + normalY[index] * lateral;
        const sourceOffset = hypot(
          candidateX[index] - source[index].x,
          candidateY[index] - source[index].y
        );
        const normalizedOffset = sourceOffset / maxOffset;
        offsetCost += normalizedOffset * normalizedOffset;
        corridorViolation = Math.max(corridorViolation, normalizedOffset - 1);
      }

      let maximumCurvature = 0;
      let violationSum = 0;
      let curvatureRateCost = 0;
      let previousCurvature = 0;
      for (let index = 1; index + 1 < count; index += 1) {
        const ab = hypot(
          candidateX[index] - candidateX[index - 1],
          candidateY[index] - candidateY[index - 1]
        );
        const bc = hypot(
          candidateX[index + 1] - candidateX[index],
          candidateY[index + 1] - candidateY[index]
        );
        const ac = hypot(
          candidateX[index + 1] - candidateX[index - 1],
          candidateY[index + 1] - candidateY[index - 1]
        );
        const denominator = ab * bc * ac;
        const signedAreaTwice =
          (candidateX[index] - candidateX[index - 1]) *
            (candidateY[index + 1] - candidateY[index - 1]) -
          (candidateY[index] - candidateY[index - 1]) *
            (candidateX[index + 1] - candidateX[index - 1]);
        const curvature = denominator > 1e-8 ? 2 * signedAreaTwice / denominator : 0;
        const normalizedViolation = Math.max(0, Math.abs(curvature) / targetCurvature - 1);
        maximumCurvature = Math.max(maximumCurvature, Math.abs(curvature));
        violationSum += normalizedViolation * normalizedViolation;
        const rate = (curvature - previousCurvature) / targetCurvature;
        curvatureRateCost += rate * rate;
        previousCurvature = curvature;
      }

      let controlSmoothness = 0;
      for (let index = 0; index + 2 < controlCount; index += 1) {
        const secondDifference = controls[index] - 2 * controls[index + 1] + controls[index + 2];
        controlSmoothness += (secondDifference / maxOffset) ** 2;
      }
      const maximumViolation = Math.max(0, maximumCurvature / targetCurvature - 1);
      const score =
        8000 * maximumViolation * maximumViolation +
        240 * violationSum / count +
        12000 * Math.max(0, corridorViolation) ** 2 +
        options.corridorOffsetPenalty * offsetCost / count +
        options.corridorSmoothnessPenalty * controlSmoothness / controlCount +
        0.002 * curvatureRateCost / count;
      return {
        score,
        maximumCurvature,
        minimumRadiusMm: maximumCurvature > 1e-10 ? 1 / maximumCurvature : Infinity,
        corridorViolation,
        points: collectPoints
          ? Array.from({ length: count }, (_, index) => ({ x: candidateX[index], y: candidateY[index] }))
          : null
      };
    }

    let current = evaluate(false);
    let evaluations = 1;
    const movableStart = 2;
    const movableEnd = controlCount - 3;
    for (const step of [14, 8, 4, 2, 1]) {
      for (let sweep = 0; sweep < options.corridorSweepsPerStep; sweep += 1) {
        let changed = false;
        for (let control = movableStart; control <= movableEnd; control += 1) {
          const original = controls[control];
          let bestValue = original;
          let bestResult = current;
          for (const direction of [-1, 1]) {
            controls[control] = clamp(original + direction * step, -maxOffset, maxOffset);
            const trial = evaluate(false);
            evaluations += 1;
            if (trial.score < bestResult.score) {
              bestResult = trial;
              bestValue = controls[control];
            }
          }
          controls[control] = bestValue;
          changed = changed || bestValue !== original;
          current = bestResult;
        }
        if (!changed) break;
      }
    }

    const finalResult = evaluate(true);
    evaluations += 1;
    let points = annotateGlobalGeometry(
      finalResult.points,
      new Array(count).fill(null),
      options.sampleStepMm
    ).map((point) => ({ ...point, geometryType: 'corridor_optimized' }));
    let offsets = summarizeOffsets(points, source);
    const corridorValid = offsets.maxOffsetMm <= maxOffset + 1e-6 && finalResult.corridorViolation <= 1e-6;
    const curvatureLimitMet = finalResult.minimumRadiusMm >= requestedMinimumRadiusMm && corridorValid;
    const warnings = [];
    if (!curvatureLimitMet) {
      warnings.push(
        `横向偏移优化未满足 ${requestedMinimumRadiusMm.toFixed(1)}mm；` +
        `当前最佳 ${finalResult.minimumRadiusMm.toFixed(1)}mm`
      );
    }
    if (!corridorValid) {
      points = baseline.points.map((point) => ({ ...point, geometryType: 'fallback' }));
      offsets = summarizeOffsets(points, source);
      warnings.push(`横向偏移结果超过 ${maxOffset}mm 走廊，已回退到全局样条`);
    }

    return {
      algorithm: 'corridor_optimizer',
      points,
      source,
      lines: baseline.lines,
      corners: [],
      warnings,
      options,
      stats: {
        ...baseline.stats,
        plannedPointCount: points.length,
        controlCount,
        requestedMinimumRadiusMm,
        achievedMinimumRadiusMm: finalResult.minimumRadiusMm,
        curvatureLimitMet,
        corridorValid,
        corridorEvaluations: evaluations,
        corridorControlCount: controlCount,
        ...offsets
      }
    };
  }

  return { plan, planGlobal, planCorridor, resample, wrapPi, DEFAULTS };
}));
