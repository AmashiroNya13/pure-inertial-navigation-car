(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  if (root) root.FirmwareSpeedPlanner = api;
}(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  'use strict';

  const DEFAULTS = Object.freeze({
    minSpeedMmS: 400,
    maxSpeedMmS: 3200,
    lateralAccelMmS2: 25000,
    curveLateralAccelMmS2: 25000,
    longitudinalAccelMmS2: 25000,
    longitudinalDecelMmS2: 25000,
    curvatureGapPoints: 20,
    curvatureEpsilonPerMm: 0.00005,
    wheelBaseMm: 120,
    wheelSpeedExtraMmS: 700,
    wheelCorrectionReserveMmS: 400,
    wheelAccelReserveRatio: 0.85,
    wheelCoefficientMinimum: 0.05,
    startHoldDistanceMm: 200,
    startHoldSpeedMmS: 500,
    endDecelDistanceMm: 300,
    endBrakeDistanceMm: 10,
    frictionCircleEnabled: true
  });

  const clamp = (value, min, max) => Math.max(min, Math.min(max, value));
  function normalizeOptions(options) {
    return { ...DEFAULTS, ...(options || {}) };
  }

  function curvatureAt(points, index, gapPoints = DEFAULTS.curvatureGapPoints) {
    if (points.length < 3 || index < 0 || index >= points.length) return 0;
    const gap = Math.max(1, Math.floor(Number(gapPoints) || 1));
    const previousIndex = Math.max(0, index - gap);
    const nextIndex = Math.min(points.length - 1, index + gap);
    if (previousIndex === index || nextIndex === index || previousIndex === nextIndex) return 0;
    const a = points[previousIndex];
    const b = points[index];
    const c = points[nextIndex];
    const ab = Math.hypot(b.x - a.x, b.y - a.y);
    const bc = Math.hypot(c.x - b.x, c.y - b.y);
    const ac = Math.hypot(c.x - a.x, c.y - a.y);
    if (ab < 1e-6 || bc < 1e-6 || ac < 1e-6) return 0;
    const cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    return 2 * cross / (ab * bc * ac);
  }

  function normalizePath(input, options = DEFAULTS) {
    const points = [];
    for (const raw of Array.isArray(input) ? input : []) {
      const x = Number(raw && (raw.x_mm ?? raw.x));
      const y = Number(raw && (raw.y_mm ?? raw.y));
      if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
      const previous = points[points.length - 1];
      if (previous && Math.hypot(x - previous.x, y - previous.y) < 0.001) continue;
      points.push({ ...raw, x, y });
    }
    let distance = 0;
    points.forEach((point, index) => {
      if (index) distance += Math.hypot(point.x - points[index - 1].x, point.y - points[index - 1].y);
      point.s_mm = distance;
    });
    points.forEach((point, index) => {
      point.curvature_per_mm = curvatureAt(points, index, options.curvatureGapPoints);
    });
    return points;
  }

  function speedClamp(speed, options) {
    return clamp(speed, options.minSpeedMmS, options.maxSpeedMmS);
  }

  function curvatureCap(curvature, options) {
    const absolute = Math.abs(curvature);
    if (absolute <= options.curvatureEpsilonPerMm || options.curveLateralAccelMmS2 <= 0) {
      return options.maxSpeedMmS;
    }
    const lateral = Math.min(options.curveLateralAccelMmS2, options.lateralAccelMmS2);
    return speedClamp(Math.sqrt(lateral / absolute), options);
  }

  function wheelStaticCap(curvature, options) {
    const halfBase = options.wheelBaseMm * 0.5;
    const outerCoefficient = 1 + halfBase * Math.abs(curvature);
    const innerCoefficient = 1 - halfBase * Math.abs(curvature);
    if (innerCoefficient <= options.wheelCoefficientMinimum) return options.minSpeedMmS;
    const wheelLimit = options.maxSpeedMmS + options.wheelSpeedExtraMmS - options.wheelCorrectionReserveMmS;
    return speedClamp(Math.max(options.minSpeedMmS, wheelLimit) / outerCoefficient, options);
  }

  function availableLongitudinalAccel(speed, curvature, braking, options) {
    const base = braking ? options.longitudinalDecelMmS2 : options.longitudinalAccelMmS2;
    if (!options.frictionCircleEnabled || options.lateralAccelMmS2 <= 0) return base;
    const lateral = speed * speed * Math.abs(curvature);
    if (lateral <= 0) return base;
    const ratio = lateral / options.lateralAccelMmS2;
    if (ratio >= 1) return 0;
    return base * Math.sqrt(Math.max(0, 1 - ratio * ratio));
  }

  function wheelTransitionCap(cap, sourceSpeed, sourceCurvature, targetCurvature, distance, accel, options) {
    if (distance <= 0.001 || accel <= 0) return cap;
    const halfBase = options.wheelBaseMm * 0.5;
    const sourceLeft = Math.max(0, 1 - halfBase * sourceCurvature);
    const sourceRight = Math.max(0, 1 + halfBase * sourceCurvature);
    const targetLeft = 1 - halfBase * targetCurvature;
    const targetRight = 1 + halfBase * targetCurvature;
    if (targetLeft <= options.wheelCoefficientMinimum || targetRight <= options.wheelCoefficientMinimum) {
      return options.minSpeedMmS;
    }
    const reservedAccel = accel * options.wheelAccelReserveRatio;
    const leftAllowed = Math.sqrt((sourceSpeed * sourceLeft) ** 2 + 2 * reservedAccel * distance) / targetLeft;
    const rightAllowed = Math.sqrt((sourceSpeed * sourceRight) ** 2 + 2 * reservedAccel * distance) / targetRight;
    return speedClamp(Math.min(cap, leftAllowed, rightAllowed), options);
  }

  function endScale(remaining, options) {
    const span = options.endDecelDistanceMm - options.endBrakeDistanceMm;
    if (remaining <= options.endBrakeDistanceMm) return 0;
    if (span <= 0 || remaining >= options.endDecelDistanceMm) return 1;
    return Math.sqrt((remaining - options.endBrakeDistanceMm) / span);
  }

  function plan(input, suppliedOptions) {
    const options = normalizeOptions(suppliedOptions);
    const points = normalizePath(input, options);
    if (points.length < 2) return { points, series: {}, options, totalDistanceMm: 0 };
    const count = points.length;
    const total = points[count - 1].s_mm;
    const curvature = points.map((point) => point.curvature_per_mm);
    const curvatureLimit = curvature.map((value) => curvatureCap(value, options));
    const wheelLimit = curvature.map((value) => wheelStaticCap(value, options));
    const staticLimit = curvatureLimit.map((value, index) => Math.min(value, wheelLimit[index]));

    for (let index = 0; index < count; index += 1) {
      if (points[index].s_mm <= options.startHoldDistanceMm) {
        staticLimit[index] = Math.min(staticLimit[index], speedClamp(options.startHoldSpeedMmS, options));
      }
      const remaining = total - points[index].s_mm;
      if (remaining < options.endDecelDistanceMm) staticLimit[index] *= endScale(remaining, options);
    }
    staticLimit[count - 1] = 0;

    const forwardLimit = staticLimit.slice();
    for (let index = 1; index < count; index += 1) {
      const distance = points[index].s_mm - points[index - 1].s_mm;
      const sourceSpeed = forwardLimit[index - 1];
      const accel = availableLongitudinalAccel(sourceSpeed, curvature[index - 1], false, options);
      let allowed = Math.sqrt(sourceSpeed * sourceSpeed + 2 * accel * distance);
      allowed = wheelTransitionCap(
        allowed, sourceSpeed, curvature[index - 1], curvature[index], distance, accel, options
      );
      forwardLimit[index] = Math.min(forwardLimit[index], allowed);
    }

    const backwardLimit = forwardLimit.slice();
    for (let index = count - 2; index >= 0; index -= 1) {
      const distance = points[index + 1].s_mm - points[index].s_mm;
      const sourceSpeed = backwardLimit[index + 1];
      const decel = availableLongitudinalAccel(sourceSpeed, curvature[index + 1], true, options);
      let allowed = Math.sqrt(sourceSpeed * sourceSpeed + 2 * decel * distance);
      allowed = wheelTransitionCap(
        allowed, sourceSpeed, curvature[index + 1], curvature[index], distance, decel, options
      );
      backwardLimit[index] = Math.min(backwardLimit[index], allowed);
    }

    const output = points.map((point, index) => ({
      ...point,
      planned_speed_mm_s: backwardLimit[index],
      curvature_speed_cap_mm_s: curvatureLimit[index],
      wheel_speed_cap_mm_s: wheelLimit[index],
      forward_speed_cap_mm_s: forwardLimit[index],
      backward_speed_cap_mm_s: backwardLimit[index]
    }));
    return {
      points: output,
      totalDistanceMm: total,
      options,
      series: {
        distanceMm: points.map((point) => point.s_mm),
        curvatureCapMmS: curvatureLimit,
        wheelCapMmS: wheelLimit,
        forwardCapMmS: forwardLimit,
        backwardCapMmS: backwardLimit,
        finalSpeedMmS: backwardLimit.slice()
      }
    };
  }

  return {
    DEFAULTS, plan, normalizePath, curvatureAt, curvatureCap, wheelStaticCap,
    availableLongitudinalAccel, endScale
  };
}));
