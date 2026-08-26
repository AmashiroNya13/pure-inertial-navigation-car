(function () {
  'use strict';

  const $ = (selector) => document.querySelector(selector);
  const canvas = $('#canvas');
  const ctx = canvas.getContext('2d');
  const container = $('#canvas-container');
  const curvatureCanvas = $('#curvature-canvas');
  const curvatureCtx = curvatureCanvas.getContext('2d');
  const speedCanvas = $('#speed-canvas');
  const speedCtx = speedCanvas.getContext('2d');
  const socket = io({ transports: ['websocket', 'polling'] });
  const MAX_POINTS = 100000;
  const MAX_CONSOLE_LINES = 500;
  const MAX_SERIAL_EVENTS_PER_FRAME = 400;
  const DPR = Math.min(window.devicePixelRatio || 1, 2);

  const dom = {
    port: $('#port-select'), baudrate: $('#baudrate-select'), connect: $('#connect-btn'),
    refreshPorts: $('#refresh-ports-btn'), connDot: $('#conn-indicator'), connText: $('#conn-text'),
    commandTemplate: $('#command-template'), command: $('#command-input'), sendCommand: $('#send-command-btn'),
    recordStart: $('#record-start-btn'), recordStop: $('#record-stop-btn'), replayStart: $('#replay-start-btn'),
    phototubeCal: $('#phototube-cal-btn'),
    emergencyStop: $('#emergency-stop-btn'), modeBadge: $('#mode-badge'), modeReason: $('#mode-reason'),
    showPredefined: $('#show-predefined'), showRealtime: $('#show-realtime'),
    showSlipCorrection: $('#show-slip-correction'), slipState: $('#slip-state'),
    showPhototubeObservation: $('#show-phototube-observation'), phototubeState: $('#phototube-state'),
    showLookahead: $('#show-lookahead'), showLookaheadHistory: $('#show-lookahead-history'),
    showSimulationLookahead: $('#show-simulation-lookahead'),
    simulationLookaheadConfig: $('#simulation-lookahead-config'),
    simulationLookaheadSource: $('#simulation-lookahead-source'),
    simulationSteeringLookahead: $('#simulation-steering-lookahead-mm'),
    simulationTangentDistance: $('#simulation-tangent-distance-mm'),
    simulationSpeedPreview: $('#simulation-speed-preview-mm'),
    showDiagnostics: $('#show-diagnostics'), lookaheadState: $('#lookahead-state'),
    geometryValidation: $('#geometry-validation'),
    showGeometryPlan: $('#show-geometry-plan'),
    geometrySummary: $('#geometry-summary'),
    geometryMethodWrap: $('#geometry-method-wrap'), geometryAlgorithm: $('#geometry-algorithm'),
    curvatureLimitWrap: $('#curvature-limit-wrap'), curvatureLimit: $('#curvature-limit'),
    minimumRadius: $('#minimum-radius-mm'), corridorOptimizer: $('#corridor-optimizer'),
    curvatureBoundLegend: $('#curvature-bound-legend'),
    curvaturePane: $('#curvature-pane'), speedPane: $('#speed-pane'),
    speedCanvas, showFirmwarePlan: $('#show-firmware-plan'),
    showExternalRoute: $('#show-external-route'), showExternalNetwork: $('#show-external-network'),
    showExternalSegments: $('#show-external-segments'),
    showExternalShoulders: $('#show-external-shoulders'),
    workspaceTabs: [...document.querySelectorAll('.workspace-tab')],
    planningTools: $('#planning-tools'), externalTools: $('#external-tools'),
    planningAlgorithm: $('#planning-algorithm'), planningRadiusLimit: $('#planning-radius-limit'),
    planningRadius: $('#planning-radius-mm'), speedPlanEnabled: $('#speed-plan-enabled'),
    speedMax: $('#speed-max'), speedLateralAccel: $('#speed-lateral-accel'),
    speedLongAccel: $('#speed-long-accel'), lateralOffset: $('#planning-lateral-offset-mm'),
    steeringLookahead: $('#steering-lookahead-mm'),
    tangentDistance: $('#tangent-distance-mm'), speedPreview: $('#speed-preview-mm'),
    planningProgress: $('#planning-progress'), planningProgressLabel: $('#planning-progress-label'),
    planningMetrics: $('#planning-metrics'),
    planningUpload: $('#planning-upload-btn'), planningUploadCancel: $('#planning-upload-cancel-btn'),
    planningUploadStatus: $('#planning-upload-status'),
    externalLoad: $('#external-load-btn'), externalFile: $('#external-file-input'),
    externalPaste: $('#external-paste-btn'), externalPasteDialog: $('#external-paste-dialog'),
    externalPasteText: $('#external-paste-text'), externalPasteImport: $('#external-paste-import-btn'),
    externalSource: $('#external-source'), externalOrigin: $('#external-origin'),
    externalOriginPosition: $('#external-origin-position'),
    externalAlign: $('#external-align'), externalRotation: $('#external-rotation'),
    externalMirrorY: $('#external-mirror-y'), externalReverse: $('#external-reverse'),
    externalOffsetX: $('#external-offset-x'), externalOffsetY: $('#external-offset-y'),
    externalShoulderDistance: $('#external-shoulder-distance'),
    externalShoulderWidth: $('#external-shoulder-width'),
    externalUseRaw: $('#external-use-raw-btn'), externalSummary: $('#external-summary'),
    externalUpload: $('#external-upload-btn'), externalUploadCancel: $('#external-upload-cancel-btn'),
    externalUploadStatus: $('#external-upload-status'),
    vehicleLength: $('#vehicle-length-mm'), vehicleWidth: $('#vehicle-width-mm'),
    vehiclePivot: $('#vehicle-pivot-mm'), vehicleSimEnabled: $('#vehicle-sim-enabled'),
    vehicleSimSource: $('#vehicle-sim-source'), vehicleSimUseSpeed: $('#vehicle-sim-use-speed'),
    vehicleSimFixedSpeed: $('#vehicle-sim-fixed-speed'), vehicleSimPlay: $('#vehicle-sim-play'),
    vehicleSimProgress: $('#vehicle-sim-progress'), vehicleSimProgressLabel: $('#vehicle-sim-progress-label'),
    vehicleSimClear: $('#vehicle-sim-clear'),
    vehicleSimPanel: $('#vehicle-sim-panel'), vehicleSimState: $('#vehicle-sim-state'),
    vehicleSimDetail: $('#vehicle-sim-detail'), vehicleCollisionEvents: $('#vehicle-collision-events'),
    layerbar: $('#layerbar'), autoFit: $('#auto-fit-btn'), originView: $('#origin-view-btn'), zoomIn: $('#zoom-in-btn'),
    zoomOut: $('#zoom-out-btn'), save: $('#save-btn'), exportRaw: $('#export-raw-btn'),
    load: $('#load-btn'), clear: $('#clear-btn'),
    theme: $('#theme-toggle-btn'), file: $('#file-input'), console: $('#serial-console'),
    clearConsole: $('#clear-console-btn'), runSelect: $('#run-select'), liveView: $('#live-view-btn'),
    playbackReset: $('#playback-reset-btn'), playbackToggle: $('#playback-toggle-btn'),
    playbackTime: $('#playback-time'), playbackSlider: $('#playback-slider'),
    slipTimelineMarkers: $('#slip-timeline-markers'),
    playbackSpeed: $('#playback-speed'), tooltip: $('#point-tooltip'), zoomHint: $('#zoom-hint'),
    sbCoords: $('#sb-coords'), sbScale: $('#sb-scale'), sbPoints: $('#sb-points'), sbFps: $('#sb-fps'),
    toasts: $('#toast-container')
  };

  let predefinedPath = [];
  let realtimePath = [];
  let lookaheadPoints = [];
  let replayRuns = [];
  let geometryPlan = null;
  let firmwareSpeedPlan = null;
  let importedFirmwarePath = [];
  let externalDocument = null;
  let externalRoute = [];
  let externalSegmentRoute = [];
  let externalNetworkPoints = [];
  let externalTrackSegments = [];
  let externalShoulderPaths = [];
  let externalShoulderSegments = [];
  let externalShoulderBounds = [];
  const vehicleSimulation = {
    pose: null,
    result: null,
    distanceMm: 0,
    elapsedSec: 0,
    colliding: false,
    processedCount: 0,
    sourceKey: '',
    sourceLabel: '',
    path: null,
    playing: false,
    lastFrame: 0,
    events: [],
    straightSections: []
  };
  let workspaceMode = 'live';
  let geometryTimer = null;
  let baselineSource = 'session';
  let runtimeMode = { mode: 'idle', reason: 'startup', updated_at: 0 };
  let serialConnected = false;
  let selectedRunId = null;
  let viewingHistory = false;
  let isDark = false;
  let dirty = true;
  let hovered = null;
  let fpsFrames = 0;
  let fpsTimestamp = performance.now();
  let currentFps = 0;
  let pathUploadActive = false;
  const pendingSerialEvents = [];

  const playback = {
    playing: false,
    time: 0,
    speed: 1,
    duration: 0,
    lastFrame: 0
  };
  let slipTimelineKey = '';

  const view = { ox: 0, oy: 0, scale: 1, minScale: 0.005, maxScale: 50 };
  const drag = { active: false, x: 0, y: 0, ox: 0, oy: 0 };
  let mouseWorld = { x: 0, y: 0 };

  function pushLimited(array, item, limit = MAX_POINTS) {
    array.push(item);
    if (array.length > limit) array.splice(0, array.length - limit);
  }

  function geometryValidationActive() {
    return Boolean(dom.geometryValidation && dom.geometryValidation.checked);
  }

  function curvatureLimitActive() {
    return geometryValidationActive() && dom.geometryAlgorithm.value === 'global' &&
      dom.curvatureLimit.checked;
  }

  function syncGeometryConstraintControls() {
    const globalActive = geometryValidationActive() && dom.geometryAlgorithm.value === 'global';
    dom.curvatureLimitWrap.hidden = !globalActive;
    dom.minimumRadius.disabled = !dom.curvatureLimit.checked;
    dom.corridorOptimizer.disabled = !dom.curvatureLimit.checked;
    dom.curvatureBoundLegend.hidden = !globalActive || !dom.curvatureLimit.checked;
  }

  function setWorkspace(mode) {
    workspaceMode = ['live', 'planning', 'external'].includes(mode) ? mode : 'live';
    dom.workspaceTabs.forEach((button) => button.classList.toggle(
      'active', button.dataset.workspace === workspaceMode
    ));
    dom.planningTools.hidden = workspaceMode !== 'planning';
    dom.externalTools.hidden = workspaceMode !== 'external';
    dom.vehicleSimPanel.hidden = !dom.vehicleSimEnabled.checked;
    dom.layerbar.classList.toggle('planning-workspace', workspaceMode === 'planning');
    if (workspaceMode === 'planning') {
      dom.geometryValidation.checked = true;
      setGeometryValidation(true);
    } else if (workspaceMode === 'external') {
      dom.geometryValidation.checked = false;
      setGeometryValidation(false);
    } else {
      dom.geometryValidation.checked = false;
      setGeometryValidation(false);
    }
    hideTooltip();
    setTimeout(() => { resizeCanvas(); resizePlanningCharts(); autoFit(false); }, 0);
    markDirty();
  }

  function speedPlannerOptions() {
    return {
      maxSpeedMmS: Math.max(400, finiteNumber(dom.speedMax.value, 3200)),
      lateralAccelMmS2: Math.max(1000, finiteNumber(dom.speedLateralAccel.value, 25000)),
      curveLateralAccelMmS2: Math.max(1000, finiteNumber(dom.speedLateralAccel.value, 25000)),
      longitudinalAccelMmS2: Math.max(1000, finiteNumber(dom.speedLongAccel.value, 25000)),
      longitudinalDecelMmS2: Math.max(1000, finiteNumber(dom.speedLongAccel.value, 25000))
    };
  }

  function computeFirmwareSpeedPlan() {
    const source = geometryPlan && geometryPlan.points.length ? geometryPlan.points : predefinedPath;
    if (!dom.speedPlanEnabled.checked || !window.FirmwareSpeedPlanner || source.length < 2) {
      firmwareSpeedPlan = null;
    } else {
      firmwareSpeedPlan = window.FirmwareSpeedPlanner.plan(source, speedPlannerOptions());
    }
    dom.speedPane.hidden = !geometryValidationActive() || !firmwareSpeedPlan;
    requestAnimationFrame(drawSpeedChart);
    markDirty();
  }

  function geometryFailureName(reason) {
    return ({
      parallel_lines: '平行直线', corner_angle: '转角不适用', negative_trim: '切点方向异常',
      profile_unsolved: '空间不足', integration_error: '积分误差', geometry_offset: '偏移超限'
    })[reason] || reason || '未知原因';
  }

  function geometryTypeName(type) {
    return ({
      line: '严格直线', clothoid_in: '进入回旋线', arc: '等半径圆弧',
      clothoid_out: '退出回旋线', curvature_spline: '全局曲率样条',
      corridor_optimized: '横向偏移优化', fallback: '原路径回退'
    })[type] || type || '-';
  }

  function computeGeometryPlan() {
    if (!geometryValidationActive()) return;
    if (!window.GeometryPlanner || predefinedPath.length < 8) {
      geometryPlan = null;
      dom.geometrySummary.textContent = predefinedPath.length
        ? `路径点不足（当前 ${predefinedPath.length} 点，至少需要 8 点）`
        : '请先录制或导入路径';
      computeFirmwareSpeedPlan();
      syncPlanningProgress();
      requestAnimationFrame(drawCurvatureChart);
      markDirty();
      return;
    }
    const started = performance.now();
    const useGlobal = dom.geometryAlgorithm.value === 'global';
    const requestedRadiusMm = curvatureLimitActive()
      ? Math.max(60, Math.min(1000, finiteNumber(dom.minimumRadius.value, 150)))
      : 0;
    const useCorridorOptimizer = useGlobal && requestedRadiusMm > 0 && dom.corridorOptimizer.checked;
    if (useCorridorOptimizer && window.GeometryPlanner.planCorridor) {
      geometryPlan = window.GeometryPlanner.planCorridor(
        predefinedPath,
        { minimumRadiusMm: requestedRadiusMm }
      );
    } else {
      geometryPlan = useGlobal
        ? window.GeometryPlanner.planGlobal(predefinedPath, { minimumRadiusMm: requestedRadiusMm })
        : window.GeometryPlanner.plan(predefinedPath);
    }
    const stats = geometryPlan.stats;
    if (useGlobal) {
      const rejectionText = stats.rejected
        ? ` · 偏移 ${stats.rejectedOffsetMm.toFixed(1)}mm 超限，已回退原路径`
        : '';
      const achievedRadiusText = Number.isFinite(stats.achievedMinimumRadiusMm)
        ? `${stats.achievedMinimumRadiusMm.toFixed(1)}mm`
        : stats.achievedMinimumRadiusMm === Infinity ? '∞' : '不可用';
      const radiusText = stats.requestedMinimumRadiusMm > 0
        ? ` · 半径限制 ≥${stats.requestedMinimumRadiusMm.toFixed(0)}mm：` +
          `${stats.curvatureLimitMet ? '已满足' : '未满足'}（实际 ${achievedRadiusText}）`
        : ` · 实际最小半径 ${achievedRadiusText}`;
      const algorithmText = useCorridorOptimizer
        ? `横向偏移优化 · 偏移控制点 ${stats.corridorControlCount}` +
          ` · 评估 ${stats.corridorEvaluations} 次`
        : `全局曲率样条 · 直线约束 ${stats.lineCount} 段 · 控制点 ${stats.controlCount}` +
          ` · 求解 ${stats.iterations} 次`;
      dom.geometrySummary.textContent =
        `${algorithmText} · 平均偏移 ${stats.meanOffsetMm.toFixed(1)}mm` +
        ` · 最大 ${stats.maxOffsetMm.toFixed(1)}mm${radiusText}${rejectionText}` +
        ` · ${Math.round(performance.now() - started)}ms`;
    } else {
      const failed = geometryPlan.corners.filter((corner) => !corner.valid);
      const failureText = failed.length
        ? ` · 回退 ${failed.length} 段（${[...new Set(failed.map((item) => geometryFailureName(item.reason)))].join('、')}）`
        : '';
      dom.geometrySummary.textContent =
        `分段回旋圆弧 · 直线 ${stats.lineCount} 段 · 几何弯道 ${stats.cornerCount} 段${failureText}` +
        ` · 平均偏移 ${stats.meanOffsetMm.toFixed(1)}mm · 最大 ${stats.maxOffsetMm.toFixed(1)}mm` +
        ` · ${Math.round(performance.now() - started)}ms`;
    }
    dom.geometrySummary.title = geometryPlan.warnings.join('\n');
    computeFirmwareSpeedPlan();
    syncPlanningProgress();
    requestAnimationFrame(drawCurvatureChart);
    markDirty();
    syncPathUploadUI();
  }

  function scheduleGeometryPlan() {
    if (!geometryValidationActive()) return;
    clearTimeout(geometryTimer);
    geometryTimer = setTimeout(computeGeometryPlan, 180);
  }

  function setGeometryValidation(enabled) {
    dom.layerbar.classList.toggle('geometry-active', enabled);
    dom.geometryMethodWrap.hidden = !enabled;
    dom.geometrySummary.hidden = !enabled;
    dom.curvaturePane.hidden = !enabled;
    dom.speedPane.hidden = !enabled || !firmwareSpeedPlan;
    syncGeometryConstraintControls();
    playback.playing = false;
    hideTooltip();
    if (enabled) computeGeometryPlan();
    markDirty();
    setTimeout(() => {
      resizeCurvatureCanvas();
      resizeSpeedCanvas();
      autoFit(true);
    }, 0);
  }

  function finiteNumber(value, fallback = null) {
    const result = Number(value);
    return Number.isFinite(result) ? result : fallback;
  }

  function toPoint(value, fallbackT = 0) {
    if (!value || typeof value !== 'object') return null;
    const x = finiteNumber(value.x);
    const y = finiteNumber(value.y);
    if (x === null || y === null) return null;
    return { ...value, x, y, t: finiteNumber(value.t, fallbackT) };
  }

  function toPointArray(values) {
    if (!Array.isArray(values)) return [];
    const result = [];
    for (let index = 0; index < values.length; index += 1) {
      const point = toPoint(values[index], index * 0.05);
      if (point) result.push(point);
    }
    return result;
  }

  function addElapsedTimes(points) {
    if (!points.length) return points;
    const firstT = finiteNumber(points[0].t, 0);
    let previous = 0;
    for (let index = 0; index < points.length; index += 1) {
      const explicit = finiteNumber(points[index].elapsed_s);
      const timestamp = finiteNumber(points[index].t);
      let elapsed = explicit;
      if (elapsed === null && timestamp !== null) elapsed = Math.max(0, timestamp - firstT);
      if (elapsed === null || elapsed < previous) elapsed = previous + (index ? 0.05 : 0);
      points[index].elapsed_s = elapsed;
      previous = elapsed;
    }
    return points;
  }

  function rebuildElapsedFromSampleIntervals(points) {
    if (points.length < 2) return false;
    const validIntervals = points.slice(1).filter((point) => {
      const intervalMs = finiteNumber(point.sample_dt_ms);
      return intervalMs !== null && intervalMs >= 1 && intervalMs <= 1000;
    }).length;
    if (validIntervals < Math.ceil((points.length - 1) * 0.5)) return false;

    const baseT = finiteNumber(points[0].t, 0);
    let elapsed = 0;
    points.forEach((point, index) => {
      if (index > 0) {
        const intervalMs = finiteNumber(point.sample_dt_ms);
        elapsed += intervalMs !== null && intervalMs >= 1 && intervalMs <= 1000
          ? intervalMs / 1000
          : 0.05;
      }
      point.elapsed_s = elapsed;
      point.t = baseT + elapsed;
    });
    return true;
  }

  function normalizeRun(raw, fallbackId) {
    const points = toPointArray(raw && raw.points);
    const rebuiltTiming = rebuildElapsedFromSampleIntervals(points);
    if (!rebuiltTiming) addElapsedTimes(points);
    const targets = toPointArray(raw && raw.lookahead);
    if (targets.length) {
      const elapsedById = new Map(points
        .filter((point) => point.id !== undefined)
        .map((point) => [String(point.id), point.elapsed_s]));
      const firstT = finiteNumber(targets[0].t, 0);
      targets.forEach((point, index) => {
        const matchedElapsed = point.id === undefined ? null : elapsedById.get(String(point.id));
        if (matchedElapsed !== null && matchedElapsed !== undefined) {
          point.elapsed_s = matchedElapsed;
          point.t = finiteNumber(points[0] && points[0].t, firstT) + matchedElapsed;
        } else if (rebuiltTiming && points[index]) {
          point.elapsed_s = points[index].elapsed_s;
          point.t = points[index].t;
        } else if (finiteNumber(point.elapsed_s) === null) {
          const t = finiteNumber(point.t);
          point.elapsed_s = t === null ? index * 0.05 : Math.max(0, t - firstT);
        }
      });
    }
    const startT = points.length ? finiteNumber(points[0].t, 0) : finiteNumber(raw && raw.start_t, 0);
    const endT = points.length ? finiteNumber(points[points.length - 1].t, startT) : startT;
    return {
      id: raw && raw.id !== undefined ? raw.id : fallbackId,
      status: raw && raw.status ? raw.status : 'imported',
      reason: raw && raw.reason ? raw.reason : 'imported',
      start_t: rebuiltTiming ? startT : finiteNumber(raw && raw.start_t, startT),
      end_t: rebuiltTiming ? endT : finiteNumber(raw && raw.end_t, points.length ? endT : null),
      points,
      lookahead: targets
    };
  }

  function normalizeRuns(rawRuns, flatPoints, flatTargets) {
    if (Array.isArray(rawRuns) && rawRuns.length) {
      return rawRuns.map((run, index) => normalizeRun(run, index + 1));
    }
    if (!flatPoints.length) return [];
    return [normalizeRun({
      id: 1, status: 'imported', reason: 'legacy_import',
      points: flatPoints, lookahead: flatTargets
    }, 1)];
  }

  function runById(id = selectedRunId) {
    return replayRuns.find((run) => String(run.id) === String(id)) || null;
  }

  function durationOf(run) {
    if (!run || !run.points.length) return 0;
    return finiteNumber(run.points[run.points.length - 1].elapsed_s, 0);
  }

  function upperBoundByTime(points, time) {
    let low = 0;
    let high = points.length;
    while (low < high) {
      const mid = (low + high) >>> 1;
      if (finiteNumber(points[mid].elapsed_s, 0) <= time) low = mid + 1;
      else high = mid;
    }
    return low;
  }

  function renderSeries() {
    if (!viewingHistory) return { poses: realtimePath, targets: lookaheadPoints };
    const run = runById();
    if (!run) return { poses: [], targets: [] };
    const poseEnd = upperBoundByTime(run.points, playback.time);
    const targetEnd = upperBoundByTime(run.lookahead, playback.time);
    return {
      poses: run.points.slice(0, poseEnd),
      targets: run.lookahead.slice(0, targetEnd)
    };
  }

  function markDirty() { dirty = true; }

  function resizeCanvas() {
    const width = Math.max(1, container.clientWidth);
    const height = Math.max(1, container.clientHeight);
    canvas.width = Math.round(width * DPR);
    canvas.height = Math.round(height * DPR);
    canvas.style.width = `${width}px`;
    canvas.style.height = `${height}px`;
    markDirty();
  }

  function resizeCurvatureCanvas() {
    if (!curvatureCanvas || dom.curvaturePane.hidden) return;
    const width = Math.max(1, curvatureCanvas.clientWidth);
    const height = Math.max(1, curvatureCanvas.clientHeight);
    curvatureCanvas.width = Math.round(width * DPR);
    curvatureCanvas.height = Math.round(height * DPR);
    drawCurvatureChart();
  }

  function resizeSpeedCanvas() {
    if (!speedCanvas || dom.speedPane.hidden) return;
    const width = Math.max(1, speedCanvas.clientWidth);
    const height = Math.max(1, speedCanvas.clientHeight);
    speedCanvas.width = Math.round(width * DPR);
    speedCanvas.height = Math.round(height * DPR);
    drawSpeedChart();
  }

  function resizePlanningCharts() {
    resizeCurvatureCanvas();
    resizeSpeedCanvas();
  }

  function buildCurvatureSeries(points, useAnnotatedCurvature) {
    const distances = [0];
    const curvatures = [0];
    for (let index = 1; index < points.length; index += 1) {
      distances.push(distances[index - 1] + Math.hypot(
        points[index].x - points[index - 1].x,
        points[index].y - points[index - 1].y
      ) / 1000);
      if (index === points.length - 1) {
        curvatures.push(0);
        continue;
      }
      if (useAnnotatedCurvature && Number.isFinite(points[index].curvature_per_mm)) {
        curvatures.push(points[index].curvature_per_mm * 1000);
        continue;
      }
      const a = points[index - 1];
      const b = points[index];
      const c = points[index + 1];
      const ab = Math.hypot(b.x - a.x, b.y - a.y);
      const bc = Math.hypot(c.x - b.x, c.y - b.y);
      const ac = Math.hypot(c.x - a.x, c.y - a.y);
      const denominator = ab * bc * ac;
      const signedAreaTwice = (b.x - a.x) * (c.y - a.y) -
        (b.y - a.y) * (c.x - a.x);
      curvatures.push(denominator > 1e-8 ? 2000 * signedAreaTwice / denominator : 0);
    }
    return { distances, curvatures, total: distances[distances.length - 1] || 0 };
  }

  function drawCurvatureChart() {
    if (!curvatureCanvas || dom.curvaturePane.hidden) return;
    const width = curvatureCanvas.clientWidth;
    const height = curvatureCanvas.clientHeight;
    if (width <= 1 || height <= 1) return;
    if (curvatureCanvas.width !== Math.round(width * DPR) ||
        curvatureCanvas.height !== Math.round(height * DPR)) {
      curvatureCanvas.width = Math.round(width * DPR);
      curvatureCanvas.height = Math.round(height * DPR);
    }
    const colors = themeColors();
    curvatureCtx.setTransform(DPR, 0, 0, DPR, 0, 0);
    curvatureCtx.clearRect(0, 0, width, height);
    curvatureCtx.fillStyle = colors.canvas;
    curvatureCtx.fillRect(0, 0, width, height);
    const plannedPoints = geometryPlan ? geometryPlan.points : [];
    const sourcePoints = geometryPlan ? geometryPlan.source : [];
    if (plannedPoints.length < 2 || sourcePoints.length < 2) return;

    const sourceSeries = buildCurvatureSeries(sourcePoints, false);
    const plannedSeries = buildCurvatureSeries(plannedPoints, true);
    const total = Math.max(sourceSeries.total, plannedSeries.total, 0.001);
    const absoluteValues = [...sourceSeries.curvatures, ...plannedSeries.curvatures]
      .map((value) => Math.abs(value))
      .filter(Number.isFinite)
      .sort((a, b) => a - b);
    const robustIndex = Math.min(
      absoluteValues.length - 1,
      Math.floor(absoluteValues.length * 0.995)
    );
    const plannedMaximum = Math.max(...plannedSeries.curvatures.map((value) => Math.abs(value)));
    const curvatureBound = curvatureLimitActive()
      ? 1000 / Math.max(1, finiteNumber(dom.minimumRadius.value, 150))
      : 0;
    const maxAbs = Math.max(
      0.5,
      absoluteValues[robustIndex] || 0,
      plannedMaximum,
      curvatureBound
    ) * 1.08;
    const margin = { left: 46, right: 12, top: 8, bottom: 20 };
    const plotWidth = Math.max(1, width - margin.left - margin.right);
    const plotHeight = Math.max(1, height - margin.top - margin.bottom);
    const xAt = (distance) => margin.left + distance / total * plotWidth;
    const yAt = (curvature) => margin.top + (maxAbs - curvature) / (maxAbs * 2) * plotHeight;

    curvatureCtx.font = '10px Consolas, monospace';
    curvatureCtx.fillStyle = colors.text;
    curvatureCtx.strokeStyle = colors.grid;
    curvatureCtx.lineWidth = 1;
    curvatureCtx.textAlign = 'right';
    curvatureCtx.textBaseline = 'middle';
    for (const ratio of [-1, -0.5, 0, 0.5, 1]) {
      const value = maxAbs * ratio;
      const y = yAt(value);
      curvatureCtx.beginPath();
      curvatureCtx.moveTo(margin.left, y);
      curvatureCtx.lineTo(width - margin.right, y);
      curvatureCtx.stroke();
      curvatureCtx.fillText(value.toFixed(1), margin.left - 6, y);
    }
    curvatureCtx.textAlign = 'center';
    curvatureCtx.textBaseline = 'top';
    for (let tick = 0; tick <= 4; tick += 1) {
      const distance = total * tick / 4;
      const x = xAt(distance);
      curvatureCtx.fillText(distance.toFixed(1), x, height - margin.bottom + 5);
    }
    curvatureCtx.strokeStyle = colors.axis;
    curvatureCtx.lineWidth = 1.2;
    curvatureCtx.beginPath();
    curvatureCtx.moveTo(margin.left, yAt(0));
    curvatureCtx.lineTo(width - margin.right, yAt(0));
    curvatureCtx.stroke();
    if (curvatureBound > 0) {
      curvatureCtx.save();
      curvatureCtx.strokeStyle = '#c62828';
      curvatureCtx.lineWidth = 1;
      curvatureCtx.setLineDash([5, 4]);
      for (const value of [-curvatureBound, curvatureBound]) {
        curvatureCtx.beginPath();
        curvatureCtx.moveTo(margin.left, yAt(value));
        curvatureCtx.lineTo(width - margin.right, yAt(value));
        curvatureCtx.stroke();
      }
      curvatureCtx.restore();
    }
    function drawSeries(series, color, lineWidth) {
      curvatureCtx.strokeStyle = color;
      curvatureCtx.lineWidth = lineWidth;
      curvatureCtx.beginPath();
      for (let index = 0; index < series.curvatures.length; index += 1) {
        const x = xAt(series.distances[index]);
        const value = Math.max(-maxAbs, Math.min(maxAbs, series.curvatures[index]));
        const y = yAt(value);
        if (index === 0) curvatureCtx.moveTo(x, y);
        else curvatureCtx.lineTo(x, y);
      }
      curvatureCtx.stroke();
    }
    drawSeries(sourceSeries, '#1976d2', 1);
    drawSeries(plannedSeries, '#f57c00', 1.5);
  }

  function drawSpeedChart() {
    if (!speedCanvas || dom.speedPane.hidden || !firmwareSpeedPlan) return;
    const width = speedCanvas.clientWidth;
    const height = speedCanvas.clientHeight;
    if (width <= 1 || height <= 1) return;
    if (speedCanvas.width !== Math.round(width * DPR) || speedCanvas.height !== Math.round(height * DPR)) {
      speedCanvas.width = Math.round(width * DPR);
      speedCanvas.height = Math.round(height * DPR);
    }
    const colors = themeColors();
    const series = firmwareSpeedPlan.series;
    const total = Math.max(1, firmwareSpeedPlan.totalDistanceMm);
    const maximum = Math.max(500, ...series.curvatureCapMmS, ...series.wheelCapMmS) / 1000 * 1.08;
    const margin = { left: 46, right: 12, top: 8, bottom: 20 };
    const plotWidth = Math.max(1, width - margin.left - margin.right);
    const plotHeight = Math.max(1, height - margin.top - margin.bottom);
    const xAt = (distance) => margin.left + distance / total * plotWidth;
    const yAt = (speed) => margin.top + (maximum - speed / 1000) / maximum * plotHeight;
    speedCtx.setTransform(DPR, 0, 0, DPR, 0, 0);
    speedCtx.clearRect(0, 0, width, height);
    speedCtx.fillStyle = colors.canvas;
    speedCtx.fillRect(0, 0, width, height);
    speedCtx.font = '10px Consolas, monospace';
    speedCtx.fillStyle = colors.text;
    speedCtx.strokeStyle = colors.grid;
    speedCtx.lineWidth = 1;
    speedCtx.textAlign = 'right';
    speedCtx.textBaseline = 'middle';
    for (let tick = 0; tick <= 4; tick += 1) {
      const value = maximum * tick / 4;
      const y = yAt(value * 1000);
      speedCtx.beginPath(); speedCtx.moveTo(margin.left, y); speedCtx.lineTo(width - margin.right, y); speedCtx.stroke();
      speedCtx.fillText(value.toFixed(1), margin.left - 6, y);
    }
    speedCtx.textAlign = 'center';
    speedCtx.textBaseline = 'top';
    for (let tick = 0; tick <= 4; tick += 1) {
      const distance = total * tick / 4;
      speedCtx.fillText((distance / 1000).toFixed(1), xAt(distance), height - margin.bottom + 5);
    }
    function line(values, color, widthPx, dashed = false) {
      speedCtx.save();
      speedCtx.strokeStyle = color;
      speedCtx.lineWidth = widthPx;
      if (dashed) speedCtx.setLineDash([5, 3]);
      speedCtx.beginPath();
      values.forEach((value, index) => {
        const x = xAt(series.distanceMm[index]);
        const y = yAt(value);
        if (!index) speedCtx.moveTo(x, y); else speedCtx.lineTo(x, y);
      });
      speedCtx.stroke();
      speedCtx.restore();
    }
    line(series.curvatureCapMmS, '#00838f', 1, true);
    line(series.wheelCapMmS, '#c2185b', 1, true);
    line(series.forwardCapMmS, '#7e57c2', 1.1);
    line(series.backwardCapMmS, '#ef6c00', 1.1);
    line(series.finalSpeedMmS, '#2e7d32', 2);
  }

  function worldToScreen(x, y) {
    return { x: view.ox + x * view.scale, y: view.oy - y * view.scale };
  }

  function screenToWorld(x, y) {
    return { x: (x - view.ox) / view.scale, y: (view.oy - y) / view.scale };
  }

  function gridSpacing() {
    const worldTarget = 64 / view.scale;
    const magnitude = 10 ** Math.floor(Math.log10(Math.max(worldTarget, 0.0001)));
    const ratio = worldTarget / magnitude;
    const step = ratio < 2 ? 1 : ratio < 5 ? 2 : 5;
    return step * magnitude;
  }

  function themeColors() {
    const css = getComputedStyle(document.documentElement);
    return {
      canvas: css.getPropertyValue('--canvas').trim(), grid: css.getPropertyValue('--grid').trim(),
      major: css.getPropertyValue('--grid-major').trim(), axis: css.getPropertyValue('--axis').trim(),
      text: css.getPropertyValue('--muted').trim()
    };
  }

  function drawGrid(width, height) {
    const colors = themeColors();
    const spacing = gridSpacing();
    const major = spacing * 5;
    const left = (0 - view.ox) / view.scale;
    const right = (width / DPR - view.ox) / view.scale;
    const bottom = (view.oy - height / DPR) / view.scale;
    const top = view.oy / view.scale;

    ctx.save();
    ctx.setTransform(DPR, 0, 0, DPR, 0, 0);
    ctx.fillStyle = colors.canvas;
    ctx.fillRect(0, 0, width / DPR, height / DPR);

    function lines(step, color, lineWidth) {
      ctx.beginPath();
      ctx.strokeStyle = color;
      ctx.lineWidth = lineWidth;
      for (let x = Math.floor(left / step) * step; x <= right; x += step) {
        const screen = worldToScreen(x, 0).x;
        ctx.moveTo(screen, 0);
        ctx.lineTo(screen, height / DPR);
      }
      for (let y = Math.floor(bottom / step) * step; y <= top; y += step) {
        const screen = worldToScreen(0, y).y;
        ctx.moveTo(0, screen);
        ctx.lineTo(width / DPR, screen);
      }
      ctx.stroke();
    }

    lines(spacing, colors.grid, 0.6);
    lines(major, colors.major, 1);

    const axisX = Math.max(24, Math.min(width / DPR - 8, view.ox));
    const axisY = Math.max(8, Math.min(height / DPR - 22, view.oy));
    ctx.strokeStyle = colors.axis;
    ctx.lineWidth = 1.4;
    ctx.beginPath();
    ctx.moveTo(0, axisY); ctx.lineTo(width / DPR, axisY);
    ctx.moveTo(axisX, 0); ctx.lineTo(axisX, height / DPR);
    ctx.stroke();

    ctx.fillStyle = colors.text;
    ctx.font = '10px Consolas, monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    for (let x = Math.floor(left / major) * major; x <= right; x += major) {
      const screenX = worldToScreen(x, 0).x;
      if (screenX < 22 || screenX > width / DPR - 18) continue;
      ctx.fillText(`${(x / 10).toFixed(0)}`, screenX, axisY + 4);
    }
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (let y = Math.floor(bottom / major) * major; y <= top; y += major) {
      const screenY = worldToScreen(0, y).y;
      if (screenY < 10 || screenY > height / DPR - 10) continue;
      ctx.fillText(`${(y / 10).toFixed(0)}`, axisX - 5, screenY);
    }
    ctx.textAlign = 'right';
    ctx.textBaseline = 'bottom';
    ctx.fillText('X / cm', width / DPR - 7, axisY - 5);
    ctx.save();
    ctx.translate(axisX + 10, 8);
    ctx.rotate(-Math.PI / 2);
    ctx.textAlign = 'right';
    ctx.fillText('Y / cm', 0, 0);
    ctx.restore();
    ctx.restore();
  }

  function beginWorld() {
    ctx.setTransform(view.scale * DPR, 0, 0, -view.scale * DPR, view.ox * DPR, view.oy * DPR);
  }

  function drawPoints(points, color, radiusPx, stride = 1) {
    if (!points.length) return;
    ctx.fillStyle = color;
    ctx.beginPath();
    const radius = radiusPx / view.scale;
    for (let index = 0; index < points.length; index += stride) {
      const point = points[index];
      ctx.moveTo(point.x + radius, point.y);
      ctx.arc(point.x, point.y, radius, 0, Math.PI * 2);
    }
    ctx.fill();
  }

  function drawPolyline(points, color, widthPx) {
    if (points.length < 2) return;
    ctx.strokeStyle = color;
    ctx.lineWidth = widthPx / view.scale;
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.moveTo(points[0].x, points[0].y);
    for (let index = 1; index < points.length; index += 1) ctx.lineTo(points[index].x, points[index].y);
    ctx.stroke();
  }

  function drawSlipCorrectionOverlay(points) {
    if (!points.length) return;
    ctx.save();
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    for (let index = 1; index < points.length; index += 1) {
      const point = points[index];
      if (finiteNumber(point.slip_correction_delta_mm, 0) <= 0.001) continue;
      ctx.strokeStyle = 'rgba(0, 166, 166, 0.92)';
      ctx.lineWidth = 5 / view.scale;
      ctx.beginPath();
      ctx.moveTo(points[index - 1].x, points[index - 1].y);
      ctx.lineTo(point.x, point.y);
      ctx.stroke();
    }
    ctx.strokeStyle = '#d58a17';
    ctx.lineWidth = 2 / view.scale;
    let previousConfirmed = false;
    points.forEach((point) => {
      const confirmed = finiteNumber(point.slip_confirmed, 0) > 0;
      if (!confirmed || previousConfirmed) {
        previousConfirmed = confirmed;
        return;
      }
      ctx.beginPath();
      ctx.arc(point.x, point.y, 5 / view.scale, 0, Math.PI * 2);
      ctx.stroke();
      previousConfirmed = true;
    });
    points.forEach((point) => {
      const turnEvent = finiteNumber(point.turn_slip_event, 0) > 0;
      const accelEvent = finiteNumber(point.accel_slip_event_state, 0) === 1;
      if (!turnEvent && !accelEvent) return;
      const eventX = finiteNumber(turnEvent ? point.turn_slip_x_mm : point.accel_slip_x_mm, point.x);
      const eventY = finiteNumber(turnEvent ? point.turn_slip_y_mm : point.accel_slip_y_mm, point.y);
      ctx.fillStyle = accelEvent ? '#d32f2f' : '#ef8b16';
      ctx.beginPath();
      ctx.arc(eventX, eventY, 5 / view.scale, 0, Math.PI * 2);
      ctx.fill();
    });
    ctx.restore();
  }

  function drawPhototubeObservationOverlay(points) {
    if (!points.length) return;
    ctx.save();
    ctx.lineCap = 'round';
    for (const point of points) {
      if (finiteNumber(point.phototube_correction_event, 0) > 0) {
        const eventX = finiteNumber(point.phototube_event_x_mm, point.x);
        const eventY = finiteNumber(point.phototube_event_y_mm, point.y);
        ctx.fillStyle = '#1b9e4b';
        ctx.beginPath();
        ctx.arc(eventX, eventY, 5 / view.scale, 0, Math.PI * 2);
        ctx.fill();
      }
      if (finiteNumber(point.phototube_line_valid, 0) <= 0 ||
          finiteNumber(point.phototube_projection_valid, 0) <= 0) continue;
      const lineX = finiteNumber(point.phototube_line_world_x_mm);
      const lineY = finiteNumber(point.phototube_line_world_y_mm);
      const projectionX = finiteNumber(point.phototube_projection_x_mm);
      const projectionY = finiteNumber(point.phototube_projection_y_mm);
      if ([lineX, lineY, projectionX, projectionY].some((value) => value === null)) continue;

      const applied = finiteNumber(point.phototube_correction_applied, 0) > 0;
      ctx.strokeStyle = applied ? 'rgba(198, 40, 120, 0.92)' : 'rgba(0, 131, 143, 0.58)';
      ctx.lineWidth = (applied ? 2.4 : 1.1) / view.scale;
      ctx.setLineDash([4 / view.scale, 3 / view.scale]);
      ctx.beginPath();
      ctx.moveTo(lineX, lineY);
      ctx.lineTo(projectionX, projectionY);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = '#00acc1';
      ctx.beginPath();
      ctx.arc(lineX, lineY, 3 / view.scale, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = '#ff8f00';
      ctx.beginPath();
      ctx.arc(projectionX, projectionY, 3 / view.scale, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.restore();
  }

  function drawExternalNetwork() {
    if (!externalTrackSegments.length) return;
    ctx.save();
    ctx.strokeStyle = 'rgba(69, 90, 100, 0.82)';
    ctx.lineWidth = 1.4 / view.scale;
    ctx.lineCap = 'round';
    ctx.beginPath();
    externalTrackSegments.forEach((segment) => {
      ctx.moveTo(segment.from.x, segment.from.y);
      ctx.lineTo(segment.to.x, segment.to.y);
    });
    ctx.stroke();
    ctx.restore();
  }

  function drawExternalShoulders() {
    if (!externalShoulderPaths.length) return;
    const width = Math.max(1, finiteNumber(dom.externalShoulderWidth.value, 40));
    ctx.save();
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    externalShoulderPaths.forEach((path) => {
      if (!path.points.length) return;
      if (path.points.length === 1) {
        ctx.fillStyle = 'rgba(198, 40, 40, 0.72)';
        ctx.beginPath();
        ctx.arc(path.points[0].x, path.points[0].y, width * 0.5, 0, Math.PI * 2);
        ctx.fill();
        return;
      }
      ctx.strokeStyle = 'rgba(198, 40, 40, 0.72)';
      ctx.lineWidth = width;
      ctx.beginPath();
      ctx.moveTo(path.points[0].x, path.points[0].y);
      for (let index = 1; index < path.points.length; index += 1) {
        ctx.lineTo(path.points[index].x, path.points[index].y);
      }
      ctx.stroke();
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.94)';
      ctx.lineWidth = 1.2 / view.scale;
      ctx.setLineDash([18 / view.scale, 12 / view.scale]);
      ctx.stroke();
      ctx.setLineDash([]);
    });
    ctx.restore();
  }

  function selectedExternalPath() {
    return dom.externalSource.value === 'segments' ? externalSegmentRoute : externalRoute;
  }

  function resampleExternalPath(points) {
    if (!window.PathUpload || !Array.isArray(points) || points.length < 2) return points || [];
    const hasPlannedSpeed = points.some((point) =>
      point && point.has_planned_speed !== false && [point.planned_speed_mm_s, point.speed_mm_s, point.external_speed_mps]
        .some((value) => Number.isFinite(Number(value)))
    );
    const prepared = window.PathUpload.prepare(points, {
      spacingMm: 5,
      defaultSpeedMmS: 1200,
      stopAtEnd: false
    });
    return prepared.points.map((point, index) => ({
      x: point.x_mm,
      y: point.y_mm,
      s_mm: point.s_mm,
      speed_mm_s: point.speed_mm_s,
      planned_speed_mm_s: hasPlannedSpeed ? point.speed_mm_s : null,
      external_speed_mps: hasPlannedSpeed ? point.speed_mm_s / 1000 : null,
      has_planned_speed: hasPlannedSpeed,
      theta_rad: point.theta_rad,
      source_index: index,
      resampled_spacing_mm: 5
    }));
  }

  function vehicleSimulationConfig() {
    const lengthMm = Math.max(20, finiteNumber(dom.vehicleLength.value, 150));
    return {
      lengthMm,
      widthMm: Math.max(20, finiteNumber(dom.vehicleWidth.value, 140)),
      pivotFromRearMm: Math.max(0, Math.min(lengthMm, finiteNumber(dom.vehiclePivot.value, 50))),
      sampleSpacingMm: 5
    };
  }

  function updateVehicleSimulationPanel() {
    const simulation = vehicleSimulation;
    const intrusion = simulation.result ? simulation.result.overlapMm : 0;
    const totalSec = simulation.path ? simulation.path.totalSec : 0;
    const totalMm = simulation.path ? simulation.path.totalMm : 0;
    let stateText = dom.vehicleSimEnabled.checked ? '等待路径' : '未启用';
    if (simulation.result && simulation.result.collided) stateText = '碰撞路肩';
    else if (simulation.playing) stateText = '仿真运行中';
    else if (simulation.pose) stateText = '仿真暂停';
    dom.vehicleSimState.textContent = stateText;
    dom.vehicleSimPanel.classList.toggle('collision', Boolean(simulation.result && simulation.result.collided));
    dom.vehicleSimProgress.max = String(Math.max(0.001, totalSec));
    dom.vehicleSimProgress.value = String(Math.min(totalSec, simulation.elapsedSec));
    dom.vehicleSimProgress.disabled = !simulation.path || simulation.path.points.length < 2;
    dom.vehicleSimProgressLabel.textContent = `${formatTime(simulation.elapsedSec)} / ${formatTime(totalSec)}`;
    if (!simulation.pose) {
      dom.vehicleSimDetail.textContent = externalShoulderSegments.length
        ? '选择路径并从起点播放后显示车体'
        : '请先导入带封闭区域的外部节点图';
    } else {
      const targets = simulationTargets();
      const lookaheadText = targets && targets.trackMode === 1 ? '直道终点锁定' : '普通前瞻';
      dom.vehicleSimDetail.textContent =
        `${simulation.sourceLabel || '路径'} · ${formatTime(simulation.elapsedSec)} / ${formatTime(totalSec)}` +
        ` · ${(simulation.distanceMm / 1000).toFixed(2)} / ${(totalMm / 1000).toFixed(2)} m` +
        ` · ${lookaheadText}` +
        (simulation.pose.speed_mm_s != null ? ` · ${simulation.pose.speed_mm_s.toFixed(0)} mm/s` : '') +
        (intrusion > 0 ? ` · 路肩重叠 ${intrusion.toFixed(1)} mm` : ' · 车体范围安全');
    }
    dom.vehicleCollisionEvents.innerHTML = simulation.events.map((event) =>
      `<li>${formatTime(event.timeSec)} · ${(event.distanceMm / 1000).toFixed(2)} m` +
      ` · (${(event.x / 10).toFixed(1)}, ${(event.y / 10).toFixed(1)}) cm</li>`
    ).join('');
  }

  function replayPoseAt(points, index) {
    const point = points[index];
    if (!point) return null;
    const theta = finiteNumber(point.theta_deg);
    let headingRad = theta === null ? null : theta * Math.PI / 180;
    if (headingRad === null) {
      const before = points[Math.max(0, index - 1)];
      const after = points[Math.min(points.length - 1, index + 1)];
      headingRad = before && after && Math.hypot(after.x - before.x, after.y - before.y) > 1e-6
        ? Math.atan2(after.y - before.y, after.x - before.x)
        : 0;
    }
    return { x: point.x, y: point.y, headingRad };
  }

  function evaluateReplayVehiclePose(pose, distanceMm, elapsedSec, recordTransition) {
    const simulation = vehicleSimulation;
    if (!pose || !window.VehicleSimulator) return;
    simulation.distanceMm = Math.max(0, distanceMm);
    simulation.elapsedSec = Math.max(0, elapsedSec);
    simulation.pose = pose;
    simulation.result = window.VehicleSimulator.collisionWithShoulders(
      simulation.pose,
      vehicleSimulationConfig(),
      externalShoulderSegments,
      Math.max(1, finiteNumber(dom.externalShoulderWidth.value, 40))
    );
    if (recordTransition && simulation.result.collided && !simulation.colliding) {
      simulation.events.push({
        timeSec: simulation.elapsedSec,
        distanceMm: simulation.distanceMm,
        x: simulation.pose.x,
        y: simulation.pose.y
      });
      if (simulation.events.length > 100) simulation.events.shift();
    }
    simulation.colliding = simulation.result.collided;
  }

  function resetVehicleSimulation() {
    vehicleSimulation.distanceMm = 0;
    vehicleSimulation.elapsedSec = 0;
    vehicleSimulation.colliding = false;
    vehicleSimulation.processedCount = 0;
    vehicleSimulation.sourceKey = '';
    vehicleSimulation.sourceLabel = '';
    vehicleSimulation.path = null;
    vehicleSimulation.playing = false;
    vehicleSimulation.lastFrame = 0;
    vehicleSimulation.events = [];
    vehicleSimulation.pose = null;
    vehicleSimulation.result = null;
    vehicleSimulation.straightSections = [];
    updateVehicleSimulationPanel();
    markDirty();
  }

  function simulationPathSource(series) {
    const firmwarePoints = firmwareSpeedPlan ? firmwareSpeedPlan.points : importedFirmwarePath;
    let source = dom.vehicleSimSource.value;
    if (source === 'auto') {
      if (workspaceMode === 'external') source = dom.externalSource.value === 'segments' ? 'external-segments' : 'external-nodes';
      else if (workspaceMode === 'planning') source = firmwarePoints.length ? 'firmware' : 'geometry';
      else source = 'replay';
    }
    const identity = (points) => {
      if (!points.length) return '0';
      const samples = [points[0], points[Math.floor(points.length / 2)], points[points.length - 1]];
      return `${points.length}:` + samples.map((point) => `${Number(point.x).toFixed(2)},${Number(point.y).toFixed(2)}`).join(':');
    };
    const choices = {
      recorded: { label: '录制路径', points: predefinedPath },
      geometry: { label: '几何规划路径', points: geometryPlan ? geometryPlan.points : [] },
      firmware: { label: '主控速度规划路径', points: firmwarePoints },
      'external-nodes': { label: '外部节点路径', points: externalRoute },
      'external-segments': { label: '外部切弯路径', points: externalSegmentRoute }
    };
    if (source === 'replay') {
      const run = viewingHistory ? runById() : null;
      return {
        key: viewingHistory ? `replay:history:${selectedRunId}` : 'replay:live',
        label: '复现轨迹',
        points: run ? run.points : (series?.poses || []),
        replay: true,
        timedReplay: Boolean(run)
      };
    }
    const selected = choices[source] || choices.recorded;
    return { ...selected, key: `${source}:${identity(selected.points)}` };
  }

  function syncVehicleSimulation(series) {
    const simulation = vehicleSimulation;
    if (!dom.vehicleSimEnabled.checked) return;
    const selectedSource = simulationPathSource(series);
    if (!selectedSource.replay || selectedSource.timedReplay) {
      const fixedSpeed = Math.max(10, finiteNumber(dom.vehicleSimFixedSpeed.value, 1200));
        const sourceKey = `${selectedSource.key}:${dom.vehicleSimUseSpeed.checked}:${fixedSpeed}`;
      if (simulation.sourceKey !== sourceKey) {
        const events = simulation.events;
        resetVehicleSimulation();
        simulation.events = events;
        simulation.sourceKey = sourceKey;
        simulation.sourceLabel = selectedSource.label;
        simulation.path = window.VehicleSimulator.preparePath(selectedSource.points, {
          usePlannedSpeed: dom.vehicleSimUseSpeed.checked,
          fixedSpeedMmS: fixedSpeed,
          useElapsedTime: selectedSource.timedReplay
        });
        simulation.straightSections = window.VehicleSimulator.straightSections(simulation.path);
        const pose = window.VehicleSimulator.poseAtTime(simulation.path, 0);
        if (pose) evaluateReplayVehiclePose(pose, 0, 0, false);
      }
      if (selectedSource.timedReplay && !simulation.playing) {
        const elapsedSec = Math.max(0, Math.min(simulation.path.totalSec, playback.time));
        const pose = window.VehicleSimulator.poseAtTime(simulation.path, elapsedSec);
        if (pose) evaluateReplayVehiclePose(pose, pose.s_mm, elapsedSec, playback.playing);
      }
      updateVehicleSimulationPanel();
      return;
    }
    const points = selectedSource.points;
    const sourceKey = viewingHistory ? `replay:history:${selectedRunId}` : 'replay:live';
    if (simulation.sourceKey !== sourceKey || points.length < simulation.processedCount) {
      resetVehicleSimulation();
      simulation.sourceKey = sourceKey;
      simulation.sourceLabel = selectedSource.label;
    }
    for (let index = simulation.processedCount; index < points.length; index += 1) {
      if (index > 0) {
        simulation.distanceMm += Math.hypot(
          points[index].x - points[index - 1].x,
          points[index].y - points[index - 1].y
        );
      }
      evaluateReplayVehiclePose(
        replayPoseAt(points, index),
        simulation.distanceMm,
        finiteNumber(points[index].elapsed_s, viewingHistory ? playback.time : 0),
        true
      );
    }
    simulation.processedCount = points.length;
    updateVehicleSimulationPanel();
  }

  function drawVehicleSimulation() {
    const simulation = vehicleSimulation;
    if (!dom.vehicleSimEnabled.checked || !simulation.pose || !simulation.result) return;
    const corners = simulation.result.corners;
    if (corners.length !== 4) return;
    const collided = simulation.result.collided;
    ctx.save();
    ctx.fillStyle = collided ? 'rgba(198, 40, 40, 0.38)' : 'rgba(24, 134, 75, 0.32)';
    ctx.strokeStyle = collided ? '#c62828' : '#18864b';
    ctx.lineWidth = 2 / view.scale;
    ctx.beginPath();
    ctx.moveTo(corners[0].x, corners[0].y);
    for (let index = 1; index < corners.length; index += 1) ctx.lineTo(corners[index].x, corners[index].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.strokeStyle = collided ? '#7f1010' : '#0d5f35';
    ctx.beginPath();
    ctx.moveTo(corners[0].x, corners[0].y);
    ctx.lineTo(corners[1].x, corners[1].y);
    ctx.stroke();
    ctx.fillStyle = '#111';
    ctx.beginPath();
    ctx.arc(simulation.pose.x, simulation.pose.y, 3.5 / view.scale, 0, Math.PI * 2);
    ctx.fill();
    if (collided && simulation.result.contact) {
      ctx.fillStyle = '#ff1744';
      ctx.beginPath();
      ctx.arc(simulation.result.contact.x, simulation.result.contact.y, 5 / view.scale, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.restore();
  }

  function headingRadians(points) {
    if (!points.length) return 0;
    const last = points[points.length - 1];
    const theta = finiteNumber(last.theta_deg);
    if (theta !== null) return theta * Math.PI / 180;
    if (points.length < 2) return 0;
    const previous = points[points.length - 2];
    return Math.atan2(last.y - previous.y, last.x - previous.x);
  }

  function drawCar(points) {
    if (!points.length) return;
    const point = points[points.length - 1];
    const angle = headingRadians(points);
    const length = 18 / view.scale;
    const width = 9 / view.scale;
    ctx.save();
    ctx.translate(point.x, point.y);
    ctx.rotate(angle);
    ctx.fillStyle = '#b71c1c';
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.3 / view.scale;
    ctx.beginPath();
    ctx.moveTo(length, 0);
    ctx.lineTo(-length * 0.65, width);
    ctx.lineTo(-length * 0.35, 0);
    ctx.lineTo(-length * 0.65, -width);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  }

  function drawTargetLink(poses, targets) {
    if (!poses.length || !targets.length) return;
    const pose = poses[poses.length - 1];
    const target = targets[targets.length - 1];
    const endpointLocked = finiteNumber(pose.track_mode) === 1;
    ctx.strokeStyle = endpointLocked ? 'rgba(25, 118, 210, 0.92)' : 'rgba(46, 155, 81, 0.75)';
    ctx.lineWidth = (endpointLocked ? 2 : 1) / view.scale;
    ctx.setLineDash([5 / view.scale, 4 / view.scale]);
    ctx.beginPath();
    ctx.moveTo(pose.x, pose.y);
    ctx.lineTo(target.x, target.y);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  function pathPointAtDistance(points, distanceMm) {
    if (!points.length) return null;
    const target = Math.max(0, distanceMm);
    let distance = 0;
    for (let index = 1; index < points.length; index += 1) {
      const a = points[index - 1];
      const b = points[index];
      const segment = Math.hypot(b.x - a.x, b.y - a.y);
      if (distance + segment >= target) {
        const ratio = segment > 1e-6 ? (target - distance) / segment : 0;
        return { x: a.x + (b.x - a.x) * ratio, y: a.y + (b.y - a.y) * ratio, index, s_mm: target };
      }
      distance += segment;
    }
    return { ...points[points.length - 1], index: points.length - 1, s_mm: distance };
  }

  function planningPath() {
    return geometryPlan && geometryPlan.points.length ? geometryPlan.points : predefinedPath;
  }

  function pathLength(points) {
    let total = 0;
    for (let index = 1; index < points.length; index += 1) {
      total += Math.hypot(points[index].x - points[index - 1].x, points[index].y - points[index - 1].y);
    }
    return total;
  }

  function nearestPathProgress(points, query) {
    if (!Array.isArray(points) || !points.length || !query) return null;
    if (points.length === 1) return 0;
    let accumulated = 0;
    let bestDistanceSquared = Infinity;
    let bestProgress = 0;
    for (let index = 1; index < points.length; index += 1) {
      const from = points[index - 1];
      const to = points[index];
      const dx = to.x - from.x;
      const dy = to.y - from.y;
      const lengthSquared = dx * dx + dy * dy;
      const length = Math.sqrt(lengthSquared);
      const ratio = lengthSquared > 1e-9
        ? Math.max(0, Math.min(1, ((query.x - from.x) * dx + (query.y - from.y) * dy) / lengthSquared))
        : 0;
      const projectedX = from.x + dx * ratio;
      const projectedY = from.y + dy * ratio;
      const distanceSquared = (query.x - projectedX) ** 2 + (query.y - projectedY) ** 2;
      if (distanceSquared < bestDistanceSquared) {
        bestDistanceSquared = distanceSquared;
        bestProgress = accumulated + length * ratio;
      }
      accumulated += length;
    }
    return bestProgress;
  }

  function syncPlanningProgressToReplay(series) {
    if (workspaceMode !== 'planning' || !series || !series.poses.length) return;
    const pose = series.poses[series.poses.length - 1];
    const query = {
      x: finiteNumber(pose.base_x_mm, pose.x),
      y: finiteNumber(pose.base_y_mm, pose.y)
    };
    const progress = nearestPathProgress(planningPath(), query);
    if (progress === null) return;
    dom.planningProgress.value = String(Math.round(progress));
    syncPlanningProgress();
  }

  function syncPlanningProgress() {
    const total = pathLength(planningPath());
    dom.planningProgress.max = String(Math.max(1, Math.round(total)));
    if (finiteNumber(dom.planningProgress.value, 0) > total) dom.planningProgress.value = String(Math.round(total));
    dom.planningProgressLabel.textContent = `${(finiteNumber(dom.planningProgress.value, 0) / 1000).toFixed(2)} m / ${(total / 1000).toFixed(2)} m`;
    const targets = planningTargets();
    if (targets.current && targets.steering) {
      const steeringPath = Math.max(0, targets.steering.s_mm - targets.base.s_mm);
      const steeringStraight = Math.hypot(
        targets.steering.x - targets.current.x,
        targets.steering.y - targets.current.y
      );
      const speedPath = Math.max(0, targets.speed.s_mm - targets.base.s_mm);
      const speedStraight = Math.hypot(targets.speed.x - targets.current.x, targets.speed.y - targets.current.y);
      dom.planningMetrics.textContent = `转向 路径/直线 ${steeringPath.toFixed(0)}/${steeringStraight.toFixed(0)} mm · ` +
        `速度 ${speedPath.toFixed(0)}/${speedStraight.toFixed(0)} mm`;
    } else {
      dom.planningMetrics.textContent = '';
    }
  }

  function targetsForPath(points, progress, options = {}) {
    const base = pathPointAtDistance(points, progress);
    const tangentBefore = pathPointAtDistance(points, Math.max(0, progress - 5));
    const tangentAfter = pathPointAtDistance(points, progress + 5);
    let current = base;
    if (base && tangentBefore && tangentAfter) {
      const heading = Math.atan2(tangentAfter.y - tangentBefore.y, tangentAfter.x - tangentBefore.x);
      const offset = finiteNumber(options.lateralOffsetMm, 0);
      current = { ...base, x: base.x - Math.sin(heading) * offset, y: base.y + Math.cos(heading) * offset };
    }
    const steeringLookahead = finiteNumber(options.steeringLookaheadMm,
      finiteNumber(dom.steeringLookahead.value, 250));
    const tangentDistance = finiteNumber(options.tangentDistanceMm,
      finiteNumber(dom.tangentDistance.value, 30));
    const speedPreview = finiteNumber(options.speedPreviewMm,
      finiteNumber(dom.speedPreview.value, 300));
    const straight = (options.straightSections || []).find((section) =>
      progress >= section.startMm && progress <= section.endMm
    );
    const steeringDistance = straight ? straight.endMm : progress + steeringLookahead;
    return {
      current,
      base,
      steering: pathPointAtDistance(points, steeringDistance),
      tangent: pathPointAtDistance(points, steeringDistance + tangentDistance),
      speed: pathPointAtDistance(points, progress + speedPreview),
      trackMode: straight ? 1 : 0
    };
  }

  function planningTargets() {
    return targetsForPath(planningPath(), finiteNumber(dom.planningProgress.value, 0), {
      lateralOffsetMm: finiteNumber(dom.lateralOffset.value, 0)
    });
  }

  function simulationTargets() {
    if (!vehicleSimulation.path || !vehicleSimulation.pose) return null;
    return targetsForPath(vehicleSimulation.path.points, vehicleSimulation.pose.s_mm, {
      straightSections: vehicleSimulation.straightSections,
      steeringLookaheadMm: finiteNumber(dom.simulationSteeringLookahead.value, 400),
      tangentDistanceMm: finiteNumber(dom.simulationTangentDistance.value, 50),
      speedPreviewMm: finiteNumber(dom.simulationSpeedPreview.value, 100)
    });
  }

  function realLookaheadAvailable(series) {
    return workspaceMode === 'live' && Boolean(series && series.poses.length && series.targets.length);
  }

  function updateSimulationLookaheadStatus(series) {
    if (realLookaheadAvailable(series)) {
      dom.simulationLookaheadSource.textContent = '真实回包优先';
      dom.simulationLookaheadSource.title = '当前存在主控目标点回包，不使用模拟参数覆盖';
      return;
    }
    const steering = finiteNumber(dom.simulationSteeringLookahead.value, 400);
    const tangent = finiteNumber(dom.simulationTangentDistance.value, 50);
    const speed = finiteNumber(dom.simulationSpeedPreview.value, 100);
    dom.simulationLookaheadSource.textContent = `模拟 ${steering}/${tangent}/${speed}`;
    dom.simulationLookaheadSource.title = '前馈 / 切线 / 速度预瞄距离，单位 mm';
  }

  function drawLookaheadTargets(targets) {
    if (!targets || !targets.current || !targets.steering) return;
    ctx.save();
    ctx.lineWidth = 1.2 / view.scale;
    ctx.strokeStyle = '#2e7d32';
    ctx.setLineDash([5 / view.scale, 4 / view.scale]);
    ctx.beginPath(); ctx.moveTo(targets.current.x, targets.current.y); ctx.lineTo(targets.steering.x, targets.steering.y); ctx.stroke();
    ctx.setLineDash([]);
    ctx.strokeStyle = '#1976d2';
    ctx.beginPath(); ctx.moveTo(targets.current.x, targets.current.y); ctx.lineTo(targets.base.x, targets.base.y); ctx.stroke();
    const markers = [
      [targets.current, '#212121', '当前位置'],
      [targets.base, '#1976d2', '最近基点'],
      [targets.steering, '#2e7d32', targets.trackMode === 1 ? '前馈点（直道终点）' : '前馈点'],
      [targets.tangent, '#f57c00', '前馈切线点'],
      [targets.speed, '#7e57c2', '速度预瞄']
    ];
    markers.forEach(([point, color]) => {
      ctx.fillStyle = color;
      ctx.beginPath(); ctx.arc(point.x, point.y, 4 / view.scale, 0, Math.PI * 2); ctx.fill();
    });
    ctx.restore();
    ctx.setTransform(DPR, 0, 0, DPR, 0, 0);
    ctx.font = '10px sans-serif';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'bottom';
    markers.forEach(([point, color, label], index) => {
      const screen = worldToScreen(point.x, point.y);
      ctx.fillStyle = color;
      ctx.fillText(label, screen.x + 6, screen.y - 5 + index * 11);
    });
    beginWorld();
  }

  function drawScale(width, height) {
    const desired = 90 / view.scale;
    const magnitude = 10 ** Math.floor(Math.log10(Math.max(desired, 0.001)));
    const value = desired / magnitude < 2 ? magnitude : desired / magnitude < 5 ? 2 * magnitude : 5 * magnitude;
    const pixels = value * view.scale;
    ctx.setTransform(DPR, 0, 0, DPR, 0, 0);
    const x = 15;
    const y = height / DPR - 17;
    ctx.strokeStyle = themeColors().axis;
    ctx.fillStyle = themeColors().text;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(x, y); ctx.lineTo(x + pixels, y);
    ctx.moveTo(x, y - 4); ctx.lineTo(x, y + 4);
    ctx.moveTo(x + pixels, y - 4); ctx.lineTo(x + pixels, y + 4);
    ctx.stroke();
    ctx.font = '10px Consolas, monospace';
    ctx.textAlign = 'center';
    ctx.fillText(value >= 1000 ? `${(value / 1000).toFixed(1)} m` : `${Math.round(value)} mm`, x + pixels / 2, y - 6);
  }

  function render() {
    const width = canvas.width;
    const height = canvas.height;
    drawGrid(width, height);
    beginWorld();

    const series = renderSeries();
    updateSimulationLookaheadStatus(series);
    updateLookaheadState(series);
    updateSlipState(series);
    updatePhototubeState(series);
    syncVehicleSimulation(series);
    syncPlanningProgressToReplay(series);
    const firmwarePoints = firmwareSpeedPlan ? firmwareSpeedPlan.points : importedFirmwarePath;
    if (dom.showExternalShoulders.checked) drawExternalShoulders();
    if (dom.showExternalNetwork.checked) drawExternalNetwork();
    if (dom.showPredefined.checked) {
      drawPolyline(predefinedPath, 'rgba(25, 118, 210, 0.72)', 1.15);
      drawPoints(predefinedPath, '#1976d2', 2.2);
    }
    if (dom.showExternalRoute.checked) {
      drawPolyline(externalRoute, '#00838f', 1.5);
      drawPoints(externalRoute, '#00838f', 2.5);
    }
    if (dom.showExternalSegments.checked) {
      drawPolyline(externalSegmentRoute, '#c2185b', 1.2);
      drawPoints(externalSegmentRoute, '#c2185b', 2);
    }
    if (dom.showExternalNetwork.checked) drawPoints(externalNetworkPoints, '#455a64', 3.8);
    if (dom.showGeometryPlan.checked && geometryPlan && geometryPlan.points.length) {
      drawPolyline(geometryPlan.points, 'rgba(245, 124, 0, 0.72)', 1.1);
      drawPoints(geometryPlan.points, '#f57c00', 1.65);
    }
    if (dom.showFirmwarePlan.checked && firmwarePoints.length) {
      drawPolyline(firmwarePoints, '#7e57c2', 1.1);
    }
    if (dom.showRealtime.checked) {
      drawPolyline(series.poses, 'rgba(229, 57, 53, 0.58)', 1.25);
      drawPoints(series.poses, '#e53935', 1.8);
    }
    if (dom.showSlipCorrection.checked) drawSlipCorrectionOverlay(series.poses);
    if (dom.showPhototubeObservation.checked) drawPhototubeObservationOverlay(series.poses);
    if (dom.showLookahead.checked && series.targets.length) {
      const targets = dom.showLookaheadHistory.checked ? series.targets : series.targets.slice(-1);
      drawPoints(targets, '#2e9b51', 3);
      drawTargetLink(series.poses, series.targets);
    }
    if (workspaceMode === 'planning') drawLookaheadTargets(planningTargets());
    else if (dom.vehicleSimEnabled.checked && dom.showSimulationLookahead.checked &&
        !realLookaheadAvailable(series)) {
      drawLookaheadTargets(simulationTargets());
    }
    drawVehicleSimulation();
    if (dom.showRealtime.checked) drawCar(series.poses);
    if (hovered) {
      ctx.strokeStyle = '#ff9800';
      ctx.lineWidth = 2 / view.scale;
      ctx.beginPath();
      ctx.arc(hovered.point.x, hovered.point.y, 7 / view.scale, 0, Math.PI * 2);
      ctx.stroke();
    }
    drawScale(width, height);
    updateStatus(series);
    dirty = false;
  }

  function renderLoop(now) {
    drainSerialEvents(MAX_SERIAL_EVENTS_PER_FRAME);
    if (vehicleSimulation.playing && vehicleSimulation.path) {
      if (!vehicleSimulation.lastFrame) vehicleSimulation.lastFrame = now;
      vehicleSimulation.elapsedSec += Math.max(0, (now - vehicleSimulation.lastFrame) / 1000);
      vehicleSimulation.lastFrame = now;
      if (vehicleSimulation.elapsedSec >= vehicleSimulation.path.totalSec) {
        vehicleSimulation.elapsedSec = vehicleSimulation.path.totalSec;
        vehicleSimulation.playing = false;
      }
      const pose = window.VehicleSimulator.poseAtTime(vehicleSimulation.path, vehicleSimulation.elapsedSec);
      if (pose) evaluateReplayVehiclePose(pose, pose.s_mm, vehicleSimulation.elapsedSec, true);
      updateVehicleSimulationPanel();
      dom.vehicleSimPlay.textContent = vehicleSimulation.playing ? '暂停' : '从起点播放';
      markDirty();
    } else {
      vehicleSimulation.lastFrame = now;
    }
    if (playback.playing && viewingHistory) {
      if (!playback.lastFrame) playback.lastFrame = now;
      playback.time += ((now - playback.lastFrame) / 1000) * playback.speed;
      playback.lastFrame = now;
      if (playback.time >= playback.duration) {
        playback.time = playback.duration;
        playback.playing = false;
      }
      syncPlaybackUI();
      markDirty();
    } else {
      playback.lastFrame = now;
    }
    if (dirty) render();
    fpsFrames += 1;
    if (now - fpsTimestamp >= 1000) {
      currentFps = Math.round(fpsFrames * 1000 / (now - fpsTimestamp));
      fpsFrames = 0;
      fpsTimestamp = now;
      dom.sbFps.textContent = `${currentFps} FPS`;
    }
    requestAnimationFrame(renderLoop);
  }

  function updateStatus(series) {
    dom.sbCoords.textContent = `X: ${(mouseWorld.x / 10).toFixed(1)} cm  Y: ${(mouseWorld.y / 10).toFixed(1)} cm`;
    dom.sbScale.textContent = `缩放 ${view.scale.toFixed(2)}x`;
    dom.sbPoints.textContent = `录制 ${predefinedPath.length} / 几何 ${geometryPlan ? geometryPlan.points.length : 0}` +
      ` / 外部 ${externalRoute.length + externalSegmentRoute.length} / 复现 ${series.poses.length} 点`;
  }

  function trackModeInfo(value) {
    const mode = finiteNumber(value);
    return ({
      0: { label: '普通前瞻', style: 'dense' },
      1: { label: '直线终点锁定', style: 'marker' },
      2: { label: '入弯过渡', style: 'blend' },
      3: { label: '出弯过渡', style: 'blend' }
    })[mode] || { label: '状态未知', style: 'waiting' };
  }

  function updateLookaheadState(series) {
    const pose = series.poses.length ? series.poses[series.poses.length - 1] : null;
    const target = series.targets.length ? series.targets[series.targets.length - 1] : null;
    if (!pose) {
      dom.lookaheadState.dataset.mode = 'waiting';
      dom.lookaheadState.textContent = '前瞻：等待数据';
      dom.lookaheadState.title = '';
      return;
    }
    const info = trackModeInfo(pose.track_mode);
    const distance = finiteNumber(pose.actual_target_distance_mm,
      target ? Math.hypot(target.x - pose.x, target.y - pose.y) : null);
    dom.lookaheadState.dataset.mode = info.style;
    dom.lookaheadState.textContent = distance === null
      ? `前瞻：${info.label}`
      : `前瞻：${info.label} · ${distance.toFixed(0)} mm`;
    dom.lookaheadState.title = '显示当前帧车体到实际选中目标点的直线距离';
  }

  function updateSlipState(series) {
    const point = series.poses.length ? series.poses[series.poses.length - 1] : null;
    if (point && finiteNumber(point.accel_slip_event_state, 0) === 1) {
      dom.slipState.dataset.mode = 'detected';
      dom.slipState.textContent = '滑移：加速疑似滑移';
      dom.slipState.title = `编码器 / IMU 加速度 ${formatValue(point.accel_slip_encoder_accel_mm_s2, '', 0)} / ${formatValue(point.accel_slip_imu_accel_mm_s2, ' mm/s²', 0)}`;
      return;
    }
    if (point && finiteNumber(point.turn_slip_event, 0) > 0) {
      dom.slipState.dataset.mode = 'detected';
      dom.slipState.textContent = '滑移：转弯滑移已确认';
      dom.slipState.title = `转角兑现率 ${formatValue(point.turn_slip_ratio, '', 3)}`;
      return;
    }
    if (!point || finiteNumber(point.slip_active) === null) {
      dom.slipState.dataset.mode = 'waiting';
      dom.slipState.textContent = '滑移：等待数据';
      dom.slipState.title = '';
      return;
    }
    const pending = finiteNumber(point.slip_pending_correction_mm, 0);
    const total = finiteNumber(point.slip_total_correction_mm, 0);
    const delta = finiteNumber(point.slip_correction_delta_mm, 0);
    const bucketCount = finiteNumber(point.slip_window_bucket_count, 0);
    const validCount = finiteNumber(point.slip_window_valid_count, 0);
    const rejectLabels = ['已触发', '窗口未满', '有效样本不足', '累计转角不足', '兑现率正常', '残差方向无效', '修正额度已满', '已检测但修正置信不足'];
    const rejectReason = finiteNumber(point.slip_reject_reason, 0);
    const rejectLabel = rejectLabels[rejectReason] || `未知原因 ${rejectReason}`;
    if (delta > 0.001) {
      dom.slipState.dataset.mode = 'correcting';
      dom.slipState.textContent = `滑移：修正 +${delta.toFixed(1)} mm`;
    } else if (pending > 0.01) {
      dom.slipState.dataset.mode = 'detected';
      dom.slipState.textContent = finiteNumber(point.slip_straight_ready, 0) > 0
        ? `滑移：释放中 ${pending.toFixed(1)} mm`
        : `滑移：等待直道 ${pending.toFixed(1)} mm`;
    } else if (total > 0.01) {
      dom.slipState.dataset.mode = 'correcting';
      dom.slipState.textContent = `滑移：已修正 ${total.toFixed(1)} mm`;
    } else {
      dom.slipState.dataset.mode = 'monitoring';
      dom.slipState.textContent = '滑移：监测中';
    }
    dom.slipState.title = `转角兑现率 ${formatValue(point.slip_yaw_realization_ratio, '', 3)}，窗口 ${validCount}/${bucketCount}，${rejectLabel}，累计修正 ${total.toFixed(1)} mm`;
  }

  function updatePhototubeState(series) {
    const point = series.poses.length ? series.poses[series.poses.length - 1] : null;
    if (point && finiteNumber(point.phototube_correction_event, 0) > 0) {
      dom.phototubeState.dataset.mode = 'correcting';
      dom.phototubeState.textContent = '光电：已执行修正';
      dom.phototubeState.title = `修正 X / Y ${formatValue(point.phototube_correction_x_mm, '', 2)} / ${formatValue(point.phototube_correction_y_mm, ' mm', 2)}`;
      return;
    }
    if (!point || finiteNumber(point.phototube_correction_enabled) === null) {
      dom.phototubeState.dataset.mode = 'waiting';
      dom.phototubeState.textContent = '光电：未监视';
      dom.phototubeState.title = '没有收到 {ptcorr} 诊断包';
      return;
    }
    const enabled = finiteNumber(point.phototube_correction_enabled, 0) > 0;
    const monitor = finiteNumber(point.phototube_monitor_enabled, enabled ? 0 : 1) > 0;
    const powered = finiteNumber(point.phototube_sensor_powered, 1) > 0;
    const lineValid = finiteNumber(point.phototube_line_valid, 0) > 0;
    const projectionValid = finiteNumber(point.phototube_projection_valid, 0) > 0;
    const shapePass = finiteNumber(point.phototube_shape_pass, 0) > 0;
    const runtimeAllowed = finiteNumber(point.phototube_runtime_allowed, enabled ? 1 : 0) > 0;
    const straight = finiteNumber(point.phototube_path_straight, 1) > 0;
    const gatePass = finiteNumber(point.phototube_gate_pass, 0) > 0;
    const applied = finiteNumber(point.phototube_correction_applied, 0) > 0;

    let mode = 'waiting';
    let label = '未监视';
    if (!powered) label = '传感器未上电';
    else if (monitor && !enabled) {
      mode = lineValid && projectionValid ? 'monitoring' : 'invalid';
      label = lineValid && projectionValid ? '仅观察' : '仅观察，数据无效';
    }
    else if (!lineValid || !projectionValid) { mode = 'invalid'; label = '数据无效'; }
    else if (enabled && (!runtimeAllowed || !straight)) { mode = 'blocked'; label = '弯道禁止'; }
    else if (!shapePass) { mode = 'invalid'; label = '线形态拒绝'; }
    else if (enabled && !gatePass) { mode = 'gated'; label = '等待稳定'; }
    else if (applied) { mode = 'correcting'; label = '修正中'; }
    else if (enabled) { mode = 'allowed'; label = '允许但未修正'; }
    dom.phototubeState.dataset.mode = mode;
    dom.phototubeState.textContent = `光电：${label}`;
    dom.phototubeState.title = `上电 ${powered ? '是' : '否'}，观测 ${lineValid ? '有效' : '无效'}，投影 ${projectionValid ? '有效' : '无效'}，许可 ${runtimeAllowed ? '是' : '否'}，实际修正 ${applied ? '是' : '否'}`;
  }

  function formatTime(seconds) {
    const safe = Math.max(0, finiteNumber(seconds, 0));
    const minutes = Math.floor(safe / 60);
    const remainder = safe - minutes * 60;
    return `${String(minutes).padStart(2, '0')}:${remainder.toFixed(3).padStart(6, '0')}`;
  }

  function syncSlipTimelineMarkers() {
    const run = runById();
    const points = run ? run.points : [];
    const last = points.length ? points[points.length - 1] : null;
    const key = `${run ? run.id : ''}:${points.length}:${playback.duration}:` +
      `${last ? finiteNumber(last.slip_total_correction_mm, 0) : 0}:` +
      `${last ? finiteNumber(last.slip_confirmed, 0) : 0}:` +
      `${last ? finiteNumber(last.slip_pending_correction_mm, 0) : 0}`;
    if (key === slipTimelineKey) return;
    slipTimelineKey = key;
    dom.slipTimelineMarkers.innerHTML = '';
    if (!points.length || playback.duration <= 0) return;

    let previousConfirmed = false;
    points.forEach((point) => {
      const confirmed = finiteNumber(point.slip_confirmed, 0) > 0;
      const corrected = finiteNumber(point.slip_correction_delta_mm, 0) > 0.001;
      if ((!confirmed || previousConfirmed) && !corrected) {
        previousConfirmed = confirmed;
        return;
      }
      const marker = document.createElement('span');
      marker.className = `slip-timeline-marker${corrected ? ' corrected' : ''}`;
      marker.style.left = `${Math.max(0, Math.min(100,
        finiteNumber(point.elapsed_s, 0) * 100 / playback.duration))}%`;
      marker.title = corrected
        ? `实际修正 ${formatValue(point.slip_correction_delta_mm, ' mm')}`
        : '检测到疑似滑移';
      dom.slipTimelineMarkers.appendChild(marker);
      previousConfirmed = confirmed;
    });
  }

  function syncPlaybackUI() {
    dom.playbackSlider.max = String(playback.duration);
    dom.playbackSlider.value = String(Math.min(playback.time, playback.duration));
    dom.playbackTime.textContent = `${formatTime(playback.time)} / ${formatTime(playback.duration)}`;
    dom.playbackToggle.textContent = playback.playing ? '暂停' : '播放';
    dom.liveView.disabled = !viewingHistory;
    syncSlipTimelineMarkers();
  }

  function rebuildRunSelect(selectLatest = false) {
    const previous = selectedRunId;
    dom.runSelect.innerHTML = '';
    if (!replayRuns.length) {
      const option = document.createElement('option');
      option.value = '';
      option.textContent = '暂无记录';
      dom.runSelect.appendChild(option);
      selectedRunId = null;
      playback.duration = 0;
      syncPlaybackUI();
      return;
    }
    replayRuns.forEach((run, index) => {
      const option = document.createElement('option');
      option.value = String(run.id);
      option.textContent = `第 ${index + 1} 趟 · ${statusName(run.status)} · ${durationOf(run).toFixed(2)}s`;
      dom.runSelect.appendChild(option);
    });
    const wanted = selectLatest ? replayRuns[replayRuns.length - 1].id : previous;
    selectedRunId = replayRuns.some((run) => String(run.id) === String(wanted)) ? wanted : replayRuns[replayRuns.length - 1].id;
    dom.runSelect.value = String(selectedRunId);
    playback.duration = durationOf(runById());
    playback.time = viewingHistory ? Math.min(playback.time, playback.duration) : playback.duration;
    syncPlaybackUI();
  }

  function enterHistory(runId, atEnd = true) {
    selectedRunId = runId;
    const run = runById();
    if (!run) return;
    viewingHistory = true;
    playback.playing = false;
    playback.duration = durationOf(run);
    playback.time = atEnd ? playback.duration : 0;
    dom.runSelect.value = String(run.id);
    syncPlaybackUI();
    hideTooltip();
    markDirty();
  }

  function returnToLive() {
    viewingHistory = false;
    playback.playing = false;
    const latest = replayRuns[replayRuns.length - 1];
    if (latest) {
      selectedRunId = latest.id;
      dom.runSelect.value = String(latest.id);
    }
    syncPlaybackUI();
    hideTooltip();
    markDirty();
  }

  function statusName(status) {
    return ({ running: '运行中', completed: '已完成', stopped: '已停止', error: '异常', imported: '已导入' })[status] || status || '未知';
  }

  function modeName(mode) {
    return ({
      idle: '空闲', record_prepare: '准备录制', recording: '正在录制', recorded: '录制完成',
      replay_prepare: '准备复现', replaying: '正在复现', finished: '复现完成',
      stopped: '已停止', error: '异常停止'
    })[mode] || mode || '未知';
  }

  function setMode(data) {
    runtimeMode = data || runtimeMode;
    const mode = runtimeMode.mode || 'idle';
    dom.modeBadge.textContent = modeName(mode);
    dom.modeBadge.className = `mode-badge mode-${mode}`;
    dom.modeReason.textContent = runtimeMode.reason || '';
    dom.modeReason.title = runtimeMode.reason || '';
  }

  function makeConsoleLine(entry) {
    if (!entry || !entry.text) return null;
    const line = document.createElement('div');
    const direction = entry.direction === 'tx' ? 'TX' : 'RX';
    const date = new Date(finiteNumber(entry.t, Date.now() / 1000) * 1000);
    line.className = `console-line console-${entry.direction === 'tx' ? 'tx' : 'rx'}`;
    const stamp = document.createElement('span');
    stamp.className = 'console-time';
    stamp.textContent = `[${date.toLocaleTimeString('zh-CN', { hour12: false })}] `;
    line.appendChild(stamp);
    line.appendChild(document.createTextNode(`${direction}  ${entry.text}`));
    return line;
  }

  function appendConsoleBatch(entries) {
    const valid = Array.isArray(entries) ? entries.filter((entry) => entry && entry.text) : [];
    if (!valid.length) return;
    const nearBottom = dom.console.scrollHeight - dom.console.scrollTop - dom.console.clientHeight < 40;
    const fragment = document.createDocumentFragment();
    valid.forEach((entry) => {
      const line = makeConsoleLine(entry);
      if (line) fragment.appendChild(line);
    });
    dom.console.appendChild(fragment);
    const overflow = dom.console.children.length - MAX_CONSOLE_LINES;
    for (let i = 0; i < overflow; i += 1) dom.console.firstElementChild.remove();
    if (nearBottom || valid.some((entry) => entry.direction === 'tx')) {
      dom.console.scrollTop = dom.console.scrollHeight;
    }
  }

  function appendConsole(entry) {
    appendConsoleBatch([entry]);
  }

  function replaceConsole(entries) {
    dom.console.innerHTML = '';
    appendConsoleBatch(Array.isArray(entries) ? entries.slice(-MAX_CONSOLE_LINES) : []);
    dom.console.scrollTop = dom.console.scrollHeight;
  }

  function sendCommand(command) {
    const value = String(command || '').trim();
    if (!value) return;
    socket.emit('send_serial_command', { command: value });
  }

  function updateSerialUI(connected, port, baudrate) {
    serialConnected = Boolean(connected);
    dom.connDot.classList.toggle('connected', serialConnected);
    dom.connDot.classList.toggle('disconnected', !serialConnected);
    dom.connText.textContent = serialConnected ? `${port} @ ${baudrate}` : '未连接';
    dom.connect.textContent = serialConnected ? '断开' : '连接';
    dom.port.disabled = serialConnected;
    dom.baudrate.disabled = serialConnected;
    syncPathUploadUI();
  }

  function syncPathUploadUI(statusText) {
    const planningReady = Boolean(geometryPlan && geometryPlan.points && geometryPlan.points.length >= 2);
    const externalReady = externalSegmentRoute.length >= 2;
    dom.planningUpload.disabled = !serialConnected || pathUploadActive || !planningReady;
    dom.externalUpload.disabled = !serialConnected || pathUploadActive || !externalReady;
    dom.planningUploadCancel.hidden = !pathUploadActive;
    dom.externalUploadCancel.hidden = !pathUploadActive;
    dom.planningUpload.closest('.flash-upload-controls').classList.toggle('uploading', pathUploadActive);
    dom.externalUpload.closest('.flash-upload-controls').classList.toggle('uploading', pathUploadActive);
    if (statusText) {
      dom.planningUploadStatus.textContent = statusText;
      dom.externalUploadStatus.textContent = statusText;
    } else if (!pathUploadActive) {
      dom.planningUploadStatus.textContent = !serialConnected ? '连接串口后写入' :
        (planningReady ? '可写入 · 5 mm 点距' : '请先生成几何规划路径');
      dom.externalUploadStatus.textContent = !serialConnected ? '连接串口后写入' :
        (externalReady ? `可写入切弯路径 · ${externalSegmentRoute.length} 点 · 5 mm 点距` : '导入数据中没有有效切弯路径');
    }
  }

  function prepareFlashUpload(kind) {
    if (!serialConnected) throw new Error('请先连接串口');
    if (!window.PathUpload) throw new Error('路径上传模块未加载');
    let source;
    let sourceName;
    if (kind === 'planning') {
      if (!geometryPlan || !geometryPlan.points || geometryPlan.points.length < 2) {
        throw new Error('请先生成几何规划路径');
      }
      source = firmwareSpeedPlan && firmwareSpeedPlan.points.length
        ? firmwareSpeedPlan.points : geometryPlan.points;
      sourceName = firmwareSpeedPlan ? 'geometry_with_speed' : 'geometry_fixed_speed';
    } else {
      source = externalSegmentRoute;
      if (source.length < 2) throw new Error('导入数据中没有有效的外部切弯路径');
      sourceName = 'external_planned';
    }
    const prepared = window.PathUpload.prepare(source, {
      spacingMm: 5,
      defaultSpeedMmS: 1200,
      phototubeNodes: (kind === 'external' || baselineSource === 'external')
        ? externalNetworkPoints : []
    });
    if (prepared.points.length > 32698) {
      throw new Error(`路径需要 ${prepared.points.length} 点，超过主控上限 32698 点`);
    }
    return {
      source: sourceName,
      totalDistanceMm: prepared.totalDistanceMm,
      markers: prepared.markers,
      phototube_zones: prepared.phototubeZones,
      points: prepared.points.map((point) => [
        Number(point.x_mm.toFixed(3)), Number(point.y_mm.toFixed(3)),
        Number(point.theta_rad.toFixed(6)), Number(point.speed_mm_s.toFixed(1))
      ])
    };
  }

  function uploadPathToFlash(kind) {
    try {
      const payload = prepareFlashUpload(kind);
      pathUploadActive = true;
      syncPathUploadUI(`准备发送 ${payload.points.length} 点 · ${(payload.totalDistanceMm / 1000).toFixed(2)} m`);
      socket.emit('upload_path_to_flash', payload);
    } catch (error) {
      showToast(error.message, 'error', 5000);
    }
  }

  function refreshPorts() { socket.emit('list_ports'); }

  function connectSerial() {
    if (!dom.port.value) {
      showToast('请先选择串口', 'warning');
      return;
    }
    socket.emit('connect_serial', { port: dom.port.value, baudrate: Number(dom.baudrate.value) });
  }

  function updatePointMeta(id, meta) {
    let changed = false;
    const update = (points) => {
      for (let index = points.length - 1; index >= 0; index -= 1) {
        if (String(points[index].id) === String(id)) {
          Object.assign(points[index], meta);
          changed = true;
          return;
        }
      }
    };
    update(realtimePath);
    replayRuns.forEach((run) => update(run.points));
    return changed;
  }

  function findOrCreateRun(runId, point) {
    let run = runById(runId);
    if (!run) {
      run = normalizeRun({ id: runId, status: 'running', reason: 'telemetry', points: [], lookahead: [] }, runId);
      replayRuns.push(run);
      rebuildRunSelect(true);
    }
    if (point && finiteNumber(point.elapsed_s) === null) {
      point.elapsed_s = run.points.length ? finiteNumber(run.points[run.points.length - 1].elapsed_s, 0) + 0.05 : 0;
    }
    return run;
  }

  function applyDataset(data, historyAtEnd = false, source = 'session') {
    slipTimelineKey = '';
    predefinedPath = toPointArray(data && data.predefined);
    importedFirmwarePath = toPointArray(data && (data.firmware_planned || data.planned_path));
    baselineSource = source;
    realtimePath = addElapsedTimes(toPointArray(data && data.realtime));
    lookaheadPoints = toPointArray(data && data.lookahead);
    replayRuns = normalizeRuns(data && data.replay_runs, realtimePath, lookaheadPoints);
    selectedRunId = replayRuns.length ? replayRuns[replayRuns.length - 1].id : null;
    viewingHistory = Boolean(historyAtEnd && replayRuns.length);
    rebuildRunSelect(true);
    if (viewingHistory) enterHistory(selectedRunId, true);
    scheduleGeometryPlan();
    hideTooltip();
    markDirty();
  }

  function serialPoint(data, deferUi = false) {
    let changed = false;
    let geometryChanged = false;
    if (data.type === 'P') {
      const point = toPoint(data.point, data.t);
      const importIsLocked = baselineSource === 'import' && runtimeMode.mode !== 'recording';
      if (point && !importIsLocked) {
        baselineSource = runtimeMode.mode === 'recording' ? 'recording' : 'serial';
        pushLimited(predefinedPath, point);
        changed = true;
        geometryChanged = true;
      }
    } else if (data.type === 'R') {
      const car = toPoint(data.car, data.t);
      const target = toPoint(data.lookahead, data.t);
      if (car) {
        pushLimited(realtimePath, car);
        changed = true;
        if (car.run_id !== undefined) {
          const run = findOrCreateRun(car.run_id, car);
          pushLimited(run.points, car);
          if (target) pushLimited(run.lookahead, target);
          playback.duration = durationOf(run);
        }
      }
      if (target) {
        pushLimited(lookaheadPoints, target);
        changed = true;
      }
    } else if (data.type === 'L') {
      const target = toPoint(data.lookahead, data.t);
      if (target) {
        pushLimited(lookaheadPoints, target);
        changed = true;
      }
    }
    if (!deferUi && changed) {
      if (geometryChanged) scheduleGeometryPlan();
      if (!viewingHistory) syncPlaybackUI();
      markDirty();
    }
    return { changed, geometryChanged };
  }

  function enqueueSerialEvent(event, data) {
    pendingSerialEvents.push({ event, data });
  }

  function enqueueSerialBatch(batch) {
    const events = batch && Array.isArray(batch.events) ? batch.events : [];
    events.forEach((item) => {
      if (item && typeof item.event === 'string') enqueueSerialEvent(item.event, item.data);
    });
  }

  function drainSerialEvents(limit) {
    if (!pendingSerialEvents.length) return;
    const count = Math.min(limit, pendingSerialEvents.length);
    const events = pendingSerialEvents.splice(0, count);
    const consoleEntries = [];
    let changed = false;
    let geometryChanged = false;

    events.forEach((item) => {
      if (item.event === 'serial_console') {
        consoleEntries.push(item.data);
      } else if (item.event === 'serial_data') {
        const result = serialPoint(item.data || {}, true);
        changed = changed || result.changed;
        geometryChanged = geometryChanged || result.geometryChanged;
      } else if (item.event === 'serial_point_update') {
        changed = updatePointMeta(item.data && item.data.id, (item.data && item.data.meta) || {}) || changed;
      }
    });

    appendConsoleBatch(consoleEntries);
    if (geometryChanged) scheduleGeometryPlan();
    if (changed) {
      if (!viewingHistory) syncPlaybackUI();
      markDirty();
    }
  }

  function showToast(message, type = 'info', timeout = 3200) {
    const toast = document.createElement('div');
    toast.className = `toast ${type}`;
    toast.textContent = message;
    dom.toasts.appendChild(toast);
    setTimeout(() => toast.remove(), timeout);
  }

  function formatValue(value, unit = '', digits = 1) {
    const number = finiteNumber(value);
    return number === null ? '-' : `${number.toFixed(digits)}${unit}`;
  }

  function tooltipHtml(info) {
    const point = info.point;
    const rows = [
      ['X', `${(point.x / 10).toFixed(2)} cm`], ['Y', `${(point.y / 10).toFixed(2)} cm`],
      ['时间', formatTime(point.elapsed_s)], ['姿态角', formatValue(point.theta_deg, ' deg')]
    ];
    if (info.layer === 'geometry') {
      rows.splice(2, 1);
      rows.push(
        ['几何类型', geometryTypeName(point.geometryType)],
        ['曲率', formatValue(point.curvature_per_mm, ' /mm', 6)],
        ['曲率半径', point.radius_mm == null ? '∞' : formatValue(point.radius_mm, ' mm', 1)],
        ['几何段', formatValue(point.geometrySegment, '', 0)]
      );
    }
    if (info.layer === 'firmware') {
      rows.splice(2, 1);
      rows.push(
        ['路程', formatValue(point.s_mm, ' mm', 1)],
        ['规划速度', formatValue(point.planned_speed_mm_s, ' mm/s', 0)],
        ['曲率约束', formatValue(point.curvature_speed_cap_mm_s, ' mm/s', 0)],
        ['轮速约束', formatValue(point.wheel_speed_cap_mm_s, ' mm/s', 0)],
        ['前向加速约束', formatValue(point.forward_speed_cap_mm_s, ' mm/s', 0)],
        ['反向制动约束', formatValue(point.backward_speed_cap_mm_s, ' mm/s', 0)]
      );
    }
    if (info.layer === 'external' || info.layer === 'external-network') {
      rows.splice(2, 1);
      rows.push(
        ['节点', point.name || point.external_node_id || '-'],
        ['节点 ID', point.id || point.external_node_id || '-']
      );
      if (info.layer === 'external') rows.push(
        ['路程', formatValue(point.s_mm, ' mm', 1)],
        ['规划速度', point.has_planned_speed === false ? '无（仿真使用固定速度）' : formatValue(point.speed_mm_s, ' mm/s', 0)]
      );
    }
    if (info.layer === 'pose') {
      rows.push(
        ['前瞻状态', trackModeInfo(point.track_mode).label],
        ['实际目标距离', formatValue(point.actual_target_distance_mm, ' mm', 0)],
        ['目标速度', formatValue(point.target_speed_mm_s, ' mm/s', 0)],
        ['左轮 目标 / 实际', `${formatValue(point.target_left_speed_mm_s, '', 0)} / ${formatValue(point.actual_left_speed_mm_s, ' mm/s', 0)}`],
        ['右轮 目标 / 实际', `${formatValue(point.target_right_speed_mm_s, '', 0)} / ${formatValue(point.actual_right_speed_mm_s, ' mm/s', 0)}`],
        ['左轮 PWM 目标 / 实际', `${formatValue(point.target_left_pwm, '', 0)} / ${formatValue(point.actual_left_pwm, '', 0)}`],
        ['右轮 PWM 目标 / 实际', `${formatValue(point.target_right_pwm, '', 0)} / ${formatValue(point.actual_right_pwm, '', 0)}`],
        ['横向误差', formatValue(point.cross_track_error_mm, ' mm')],
        ['角度误差', formatValue(point.angle_error_deg, ' deg')]
      );
      if (finiteNumber(point.slip_active) !== null) {
        rows.push(
          ['滑移判断', finiteNumber(point.slip_confirmed, 0) > 0 ? '已确认' : '未确认'],
          ['检测窗口 有效 / 总数', `${formatValue(point.slip_window_valid_count, '', 0)} / ${formatValue(point.slip_window_bucket_count, '', 0)}`],
          ['检测窗口时长', formatValue(point.slip_window_ms, ' ms', 0)],
          ['最近拒绝原因', (['已触发', '窗口未满', '有效样本不足', '累计转角不足', '兑现率正常', '残差方向无效', '修正额度已满', '已检测但修正置信不足'])[finiteNumber(point.slip_reject_reason, 0)] || '未知'],
          ['直道释放', finiteNumber(point.slip_straight_ready, 0) > 0 ? '允许' : '等待'],
          ['转角兑现率', formatValue(point.slip_yaw_realization_ratio, '', 3)],
          ['待修正', formatValue(point.slip_pending_correction_mm, ' mm', 2)],
          ['本帧修正', formatValue(point.slip_correction_delta_mm, ' mm', 2)],
          ['累计修正', formatValue(point.slip_total_correction_mm, ' mm', 2)],
          ['X / Y 修正', `${formatValue(point.slip_correction_x_mm, '', 2)} / ${formatValue(point.slip_correction_y_mm, ' mm', 2)}`]
        );
      }
      if (finiteNumber(point.phototube_correction_enabled) !== null) {
        rows.push(
          ['光电模式', finiteNumber(point.phototube_correction_enabled, 0) > 0 ? '修正已启用' : '仅观察 / 已关闭'],
          ['光电实际上电', finiteNumber(point.phototube_sensor_powered, 0) > 0 ? '是' : '否'],
          ['白线世界坐标', `${formatValue(point.phototube_line_world_x_mm, '', 1)} / ${formatValue(point.phototube_line_world_y_mm, ' mm', 1)}`],
          ['路径投影坐标', `${formatValue(point.phototube_projection_x_mm, '', 1)} / ${formatValue(point.phototube_projection_y_mm, ' mm', 1)}`],
          ['投影距离', formatValue(point.phototube_projection_distance_mm, ' mm', 1)],
          ['误差 X / Y', `${formatValue(point.phototube_error_x_mm, '', 1)} / ${formatValue(point.phototube_error_y_mm, ' mm', 1)}`],
          ['置信度 / 线宽', `${formatValue(point.phototube_confidence, '', 3)} / ${formatValue(point.phototube_active_width_mm, ' mm', 1)}`],
          ['形态 / 稳定门', `${finiteNumber(point.phototube_shape_pass, 0) > 0 ? '通过' : '拒绝'} / ${finiteNumber(point.phototube_gate_pass, 0) > 0 ? '通过' : '等待'}`],
          ['本帧实际修正', finiteNumber(point.phototube_correction_applied, 0) > 0 ? '是' : '否']
        );
      }
      if (dom.showDiagnostics.checked) {
        rows.push(
          ['目标角', formatValue(point.target_theta_deg, ' deg')],
          ['当前角速度', formatValue(point.yaw_rate_deg_s, ' deg/s')],
          ['目标角速度', formatValue(point.target_yaw_rate_deg_s, ' deg/s')],
          ['前瞻距离', formatValue(point.lookahead_distance_mm, ' mm')]
        );
      }
    }
    if (info.layer === 'target') {
      rows.push(
        ['前瞻状态', trackModeInfo(point.track_mode).label],
        ['配置前瞻距离', formatValue(point.lookahead_distance_mm, ' mm', 0)],
        ['实际目标距离', formatValue(point.actual_target_distance_mm, ' mm', 0)],
        ['目标索引', formatValue(point.target_index, '', 0)]
      );
    }
    return `<div class="tooltip-title">${info.label}</div><div class="tooltip-grid">${rows.map(([key, value]) => `<span class="tooltip-label">${key}</span><span class="tooltip-value">${value}</span>`).join('')}</div>`;
  }

  function nearestPoint(screenX, screenY) {
    const series = renderSeries();
    const layers = [];
    const firmwarePoints = firmwareSpeedPlan ? firmwareSpeedPlan.points : importedFirmwarePath;
    if (dom.showLookahead.checked) layers.push({
      points: dom.showLookaheadHistory.checked ? series.targets : series.targets.slice(-1),
      layer: 'target', label: '前瞻目标点', radius: 10
    });
    if (dom.showRealtime.checked) layers.push({ points: series.poses, layer: 'pose', label: '复现车体点', radius: 10 });
    if (dom.showFirmwarePlan.checked) layers.push({ points: firmwarePoints, layer: 'firmware', label: '主控速度规划点', radius: 10 });
    if (dom.showGeometryPlan.checked && geometryPlan) layers.push({ points: geometryPlan.points, layer: 'geometry', label: '几何规划点', radius: 10 });
    if (dom.showExternalNetwork.checked) layers.push({ points: externalNetworkPoints, layer: 'external-network', label: '原始节点', radius: 10 });
    if (dom.showExternalSegments.checked) layers.push({ points: externalSegmentRoute, layer: 'external', label: '外部切弯点', radius: 10 });
    if (dom.showExternalRoute.checked) layers.push({ points: externalRoute, layer: 'external', label: '外部节点', radius: 10 });
    if (dom.showPredefined.checked) layers.push({ points: predefinedPath, layer: 'path', label: '录制路径点', radius: 8 });
    let best = null;
    layers.forEach((layer) => {
      for (let index = layer.points.length - 1; index >= 0; index -= 1) {
        const point = layer.points[index];
        const screen = worldToScreen(point.x, point.y);
        const distance = Math.hypot(screen.x - screenX, screen.y - screenY);
        if (distance <= layer.radius && (!best || distance < best.distance)) best = { ...layer, point, distance };
      }
    });
    return best;
  }

  function showTooltip(info, clientX, clientY) {
    hovered = info;
    dom.tooltip.innerHTML = tooltipHtml(info);
    dom.tooltip.hidden = false;
    const rect = container.getBoundingClientRect();
    let left = clientX - rect.left + 14;
    let top = clientY - rect.top + 14;
    left = Math.max(7, Math.min(left, rect.width - dom.tooltip.offsetWidth - 7));
    top = Math.max(7, Math.min(top, rect.height - dom.tooltip.offsetHeight - 7));
    dom.tooltip.style.left = `${left}px`;
    dom.tooltip.style.top = `${top}px`;
    markDirty();
  }

  function hideTooltip() {
    hovered = null;
    dom.tooltip.hidden = true;
    markDirty();
  }

  function visiblePoints() {
    const series = renderSeries();
    const firmwarePoints = firmwareSpeedPlan ? firmwareSpeedPlan.points : importedFirmwarePath;
    return [
      ...(dom.showPredefined.checked ? predefinedPath : []),
      ...(dom.showGeometryPlan.checked && geometryPlan ? geometryPlan.points : []),
      ...(dom.showFirmwarePlan.checked ? firmwarePoints : []),
      ...(dom.showExternalRoute.checked ? externalRoute : []),
      ...(dom.showExternalSegments.checked ? externalSegmentRoute : []),
      ...(dom.showExternalNetwork.checked ? externalNetworkPoints : []),
      ...(dom.showExternalShoulders.checked ? externalShoulderBounds : []),
      ...(dom.showRealtime.checked ? series.poses : []),
      ...(dom.showLookahead.checked ? series.targets : [])
    ];
  }

  function autoFit(animated = true) {
    const points = visiblePoints();
    if (!points.length) return;
    let minX = Infinity; let minY = Infinity; let maxX = -Infinity; let maxY = -Infinity;
    points.forEach((point) => {
      minX = Math.min(minX, point.x); maxX = Math.max(maxX, point.x);
      minY = Math.min(minY, point.y); maxY = Math.max(maxY, point.y);
    });
    const width = container.clientWidth;
    const height = container.clientHeight;
    const padding = Math.max(38, Math.min(width, height) * 0.08);
    const targetScale = Math.max(view.minScale, Math.min(view.maxScale,
      Math.min((width - padding * 2) / Math.max(maxX - minX, 80), (height - padding * 2) / Math.max(maxY - minY, 80))));
    const targetOx = width / 2 - ((minX + maxX) / 2) * targetScale;
    const targetOy = height / 2 + ((minY + maxY) / 2) * targetScale;
    if (!animated) {
      Object.assign(view, { ox: targetOx, oy: targetOy, scale: targetScale });
      markDirty();
      return;
    }
    const start = { ox: view.ox, oy: view.oy, scale: view.scale, at: performance.now() };
    function frame(now) {
      const ratio = Math.min(1, (now - start.at) / 250);
      const eased = 1 - (1 - ratio) ** 3;
      view.ox = start.ox + (targetOx - start.ox) * eased;
      view.oy = start.oy + (targetOy - start.oy) * eased;
      view.scale = start.scale + (targetScale - start.scale) * eased;
      markDirty();
      if (ratio < 1) requestAnimationFrame(frame);
    }
    requestAnimationFrame(frame);
  }

  function zoomAt(factor, x, y) {
    const before = screenToWorld(x, y);
    view.scale = Math.max(view.minScale, Math.min(view.maxScale, view.scale * factor));
    view.ox = x - before.x * view.scale;
    view.oy = y + before.y * view.scale;
    markDirty();
  }

  function focusCoordinateOrigin() {
    view.ox = container.clientWidth / 2;
    view.oy = container.clientHeight / 2;
    view.scale = 0.4;
    hideTooltip();
    markDirty();
    showToast('坐标零点 (0, 0) 已回到画面中心', 'info');
  }

  function saveData() {
    const data = {
      version: '3.0', saved_at: new Date().toISOString(),
      predefined: predefinedPath, realtime: realtimePath, lookahead: lookaheadPoints,
      replay_runs: replayRuns,
      firmware_planned: importedFirmwarePath,
      external_graph: externalDocument ? externalDocument.raw : null
    };
    const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.download = `pathcapture_${new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19)}.json`;
    document.body.appendChild(link);
    link.click();
    link.remove();
    URL.revokeObjectURL(url);
    showToast('路径、全部复现趟次和时间轴已保存', 'success');
  }

  function exportRawLog() {
    const link = document.createElement('a');
    link.href = `/api/export-raw-log?t=${Date.now()}`;
    document.body.appendChild(link);
    link.click();
    link.remove();
    showToast('正在导出本次连接收到的完整原始串口数据', 'success');
  }

  function loadFile(file) {
    const extension = file.name.toLowerCase().split('.').pop();
    if (['dat', 'log', 'txt'].includes(extension)) {
      importRawLog(file);
      return;
    }
    const reader = new FileReader();
    reader.onload = (event) => {
      try {
        const data = JSON.parse(event.target.result);
        if (Array.isArray(data.nodes)) {
          loadExternalDocumentData(data, file.name);
          return;
        }
        if (!Array.isArray(data.predefined) && !Array.isArray(data.realtime) && !Array.isArray(data.replay_runs)) {
          throw new Error('文件中没有可识别的路径数据');
        }
        applyDataset(data, true, 'import');
        if (data.external_graph && Array.isArray(data.external_graph.nodes)) {
          loadExternalDocumentData(data.external_graph, `${file.name} 内的节点图`);
        }
        setTimeout(() => autoFit(true), 0);
        showToast(`已导入 ${file.name}：录制 ${predefinedPath.length} 点，复现 ${replayRuns.length} 趟`, 'success', 5000);
      } catch (error) {
        showToast(`导入失败：${error.message}`, 'error', 5000);
      }
    };
    reader.readAsText(file);
  }

  function externalTransformOptions() {
    const originId = dom.externalOrigin.value || (externalDocument && externalDocument.startNodeId);
    const offsetX = finiteNumber(dom.externalOffsetX.value, 0);
    const offsetY = finiteNumber(dom.externalOffsetY.value, 0);
    return {
      originNodeId: originId,
      offsetX,
      offsetY,
      targetOriginX: offsetX,
      targetOriginY: offsetY,
      startAtOrigin: true,
      alignFirstSegment: dom.externalAlign.checked,
      rotationDeg: finiteNumber(dom.externalRotation.value, 0),
      mirrorY: dom.externalMirrorY.checked,
      reverse: dom.externalReverse.checked
    };
  }

  function applyExternalTransform(fit = false) {
    if (!externalDocument || !window.ExternalPath) return;
    const options = externalTransformOptions();
    try {
      externalRoute = resampleExternalPath(window.ExternalPath.route(externalDocument, 'nodes', options));
    } catch (_) {
      externalRoute = [];
    }
    try {
      externalSegmentRoute = resampleExternalPath(window.ExternalPath.route(externalDocument, 'segments', options));
    } catch (_) {
      externalSegmentRoute = [];
    }
    const selected = dom.externalSource.value === 'segments' ? externalSegmentRoute : externalRoute;
    if (baselineSource === 'external' && selected.length >= 2) {
      predefinedPath = selected.map((point) => ({ ...point }));
      geometryPlan = null;
      if (geometryValidationActive()) {
        if (fit) computeGeometryPlan();
        else scheduleGeometryPlan();
      }
    }
    const shoulderDistance = Math.max(0, finiteNumber(dom.externalShoulderDistance.value, 150));
    const shoulderWidth = Math.max(1, finiteNumber(dom.externalShoulderWidth.value, 40));
    externalTrackSegments = window.ExternalPath.networkSegments(externalDocument, 'nodes', options);
    externalNetworkPoints = window.ExternalPath.networkNodes(externalDocument, 'nodes', options);
    externalShoulderPaths = window.ExternalPath.buildShoulderPaths(
      externalTrackSegments, shoulderDistance, shoulderWidth
    );
    externalShoulderSegments = window.ExternalPath.shoulderPathSegments(externalShoulderPaths);
    const shoulderHalfWidth = shoulderWidth * 0.5;
    externalShoulderBounds = externalShoulderPaths.flatMap((path) => path.points.flatMap((point) => [
      { x: point.x - shoulderHalfWidth, y: point.y - shoulderHalfWidth },
      { x: point.x + shoulderHalfWidth, y: point.y + shoulderHalfWidth }
    ]));
    const origin = externalDocument.nodeById.get(String(options.originNodeId));
    dom.externalOriginPosition.textContent = origin
      ? `导出起点 (${origin.x.toFixed(0)}, ${origin.y.toFixed(0)}) mm`
      : '导出起点 -';
    const directionName = ({ clockwise: '顺时针', counterclockwise: '逆时针', mixed_or_open: '混合/开放路径' })[externalDocument.travelDirection]
      || externalDocument.travelDirection || '未标注';
    const endNode = externalDocument.endNodeId && externalDocument.nodeById.get(externalDocument.endNodeId);
    const traversalSummary = externalDocument.hasTraversalMetadata
      ? `方向 ${directionName} · ${externalDocument.plannedLapCount} 圈 · 第 ${externalDocument.stopLap} 圈停 ${endNode?.name || externalDocument.endNodeId || '未标注'} · `
      : '';
    dom.externalSummary.textContent = `节点 ${externalDocument.nodes.length} 个 · 5mm 折线 ${externalRoute.length} 点 · ` +
      `5mm 切弯 ${externalSegmentRoute.length} 点 · 当前 ${(pathLength(selected) / 1000).toFixed(2)} m · ` +
      traversalSummary +
      `起点 (${options.targetOriginX.toFixed(0)}, ${options.targetOriginY.toFixed(0)}) mm · ` +
      `偏置 (${options.offsetX.toFixed(0)}, ${options.offsetY.toFixed(0)}) mm · ` +
      `路肩最小净距 ≥${shoulderDistance.toFixed(0)} mm · 宽 ${shoulderWidth.toFixed(0)} mm · ` +
      `封闭区域中线 · 节点赛道 ${externalTrackSegments.length} 段 · ` +
      `可用路肩 ${externalShoulderPaths.length} 条`;
    resetVehicleSimulation();
    markDirty();
    syncPathUploadUI();
    if (fit) setTimeout(() => autoFit(true), 0);
  }

  function loadExternalDocumentData(data, name = '节点图') {
    if (!window.ExternalPath) throw new Error('外部路径模块未加载');
    externalDocument = window.ExternalPath.parseDocument(data);
    dom.externalOrigin.innerHTML = '';
    externalDocument.nodes.forEach((node) => {
      const option = document.createElement('option');
      option.value = node.id;
      option.textContent = `${node.name || node.id} (${node.id})`;
      dom.externalOrigin.appendChild(option);
    });
    dom.externalOrigin.value = externalDocument.nodeById.has(externalDocument.startNodeId)
      ? externalDocument.startNodeId
      : externalDocument.nodes[0].id;
    const defaultOrigin = externalDocument.nodeById.get(String(dom.externalOrigin.value));
    dom.externalOffsetX.value = '0';
    dom.externalOffsetY.value = '0';
    dom.externalShoulderDistance.value = '150';
    dom.externalShoulderWidth.value = '40';
    applyExternalTransform(false);
    setWorkspace('external');
    autoFit(true);
    showToast(`已导入 ${name}：${externalDocument.nodes.length} 个节点`, 'success', 5000);
  }

  function loadExternalFile(file) {
    const reader = new FileReader();
    reader.onload = (event) => {
      try {
        loadExternalDocumentData(JSON.parse(event.target.result), file.name);
      } catch (error) {
        showToast(`节点图导入失败：${error.message}`, 'error', 6000);
      } finally {
        dom.externalFile.value = '';
      }
    };
    reader.readAsText(file);
  }

  async function importRawLog(file) {
    const form = new FormData();
    form.append('file', file);
    showToast(`正在解析 ${file.name}，大日志可能需要几秒`, 'info', 8000);
    try {
      const response = await fetch('/api/import-log', { method: 'POST', body: form });
      const data = await response.json();
      if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
      applyDataset(data, true, 'import');
      setTimeout(() => autoFit(true), 0);
      const stats = data.import_stats || {};
      showToast(
        `解析完成：路径 ${stats.path_points || 0} 点，复现 ${stats.replay_runs || 0} 趟，` +
        `过滤 ${stats.filtered_lines || 0} 行杂项`,
        'success', 7000
      );
    } catch (error) {
      showToast(`日志导入失败：${error.message}`, 'error', 7000);
    } finally {
      dom.file.value = '';
    }
  }

  function clearAll() {
    if (!predefinedPath.length && !realtimePath.length && !replayRuns.length && !externalDocument) return;
    if (window.confirm(
      `确定清空全部数据？\n录制路径 ${predefinedPath.length} 点\n复现记录 ${replayRuns.length} 趟` +
      `\n外部节点 ${externalDocument ? externalDocument.nodes.length : 0} 个`
    )) socket.emit('clear_data');
  }

  function resetLocalData() {
    pendingSerialEvents.length = 0;
    applyDataset({}, false);
    externalDocument = null;
    externalRoute = [];
    externalSegmentRoute = [];
    externalNetworkPoints = [];
    externalTrackSegments = [];
    externalShoulderPaths = [];
    externalShoulderSegments = [];
    externalShoulderBounds = [];
    geometryPlan = null;
    firmwareSpeedPlan = null;
    importedFirmwarePath = [];
    baselineSource = 'session';
    playback.playing = false;
    playback.time = 0;
    selectedRunId = null;
    viewingHistory = false;
    pathUploadActive = false;
    dom.console.innerHTML = '';
    dom.externalSummary.textContent = '尚未导入节点图';
    resetVehicleSimulation();
    syncPlaybackUI();
    syncPathUploadUI();
    setMode({ mode: 'idle', reason: 'local_reset', updated_at: Date.now() / 1000 });
    hideTooltip();
    markDirty();
  }

  async function forceClearAll() {
    const confirmed = window.confirm(
      `强制清空上位机全部数据？\n录制路径 ${predefinedPath.length} 点\n复现记录 ${replayRuns.length} 趟` +
      `\n外部节点 ${externalDocument ? externalDocument.nodes.length : 0} 个\n不会清除主控 Flash。`
    );
    if (!confirmed) return;

    resetLocalData();
    socket.emit('clear_data');
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 1500);
    try {
      const response = await fetch('/api/reset-data', { method: 'POST', signal: controller.signal });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      showToast('前端与后台数据已强制清空', 'success');
    } catch (_) {
      showToast('前端已清空；后台暂未响应，可继续操作或重启后台', 'warning', 5000);
    } finally {
      clearTimeout(timeout);
    }
  }

  socket.on('connect', () => refreshPorts());
  socket.on('disconnect', () => updateSerialUI(false, '', 0));
  socket.on('ports_list', (data) => {
    const selected = dom.port.value;
    dom.port.innerHTML = '<option value="">选择串口</option>';
    (data.ports || []).forEach((port) => {
      const option = document.createElement('option');
      option.value = port.device;
      option.textContent = `${port.device} · ${port.description || port.name}`;
      dom.port.appendChild(option);
    });
    if ([...dom.port.options].some((option) => option.value === selected)) dom.port.value = selected;
  });
  socket.on('serial_status', (data) => updateSerialUI(data.connected, data.port, data.baudrate));
  socket.on('serial_error', (data) => showToast(data.message || '串口错误', 'error', 5000));
  socket.on('command_result', (data) => {
    if (!data.success) showToast(data.error || '命令发送失败', 'error', 4500);
    else dom.command.value = '';
  });
  socket.on('path_upload_accepted', (data) => {
    if (!data.success) {
      pathUploadActive = false;
      syncPathUploadUI();
      showToast(data.error || '主控路径写入未启动', 'error', 5000);
    } else {
      syncPathUploadUI(`主控正在擦除 Flash · ${data.count} 点`);
    }
  });
  socket.on('path_upload_started', (data) => {
    pathUploadActive = true;
    syncPathUploadUI(`正在写入 0 / ${data.count} 点`);
    showToast(`主控已接受 ${data.count} 个路径点`, 'info');
  });
  socket.on('path_upload_progress', (data) => {
    syncPathUploadUI(`正在写入 ${data.completed} / ${data.count} 点 · ${data.percent.toFixed(1)}%`);
  });
  socket.on('path_upload_finished', (data) => {
    pathUploadActive = false;
    syncPathUploadUI();
    if (data.success) {
      const length = (finiteNumber(data.length_mm, 0) / 1000).toFixed(2);
      showToast(`Flash 写入成功：${data.count} 点，${length} m`, 'success', 6000);
    } else {
      showToast(`Flash 写入失败：${data.error || '未知错误'}`, 'error', 7000);
    }
  });
  socket.on('serial_batch', enqueueSerialBatch);
  socket.on('serial_console', (data) => enqueueSerialEvent('serial_console', data));
  socket.on('serial_data', (data) => enqueueSerialEvent('serial_data', data));
  socket.on('serial_point_update', (data) => enqueueSerialEvent('serial_point_update', data));
  socket.on('mode_status', setMode);
  socket.on('recording_started', () => {
    predefinedPath = [];
    geometryPlan = null;
    scheduleGeometryPlan();
    baselineSource = 'recording';
    hideTooltip();
    markDirty();
    showToast('小车已确认开始录制，旧录制路径已清空', 'success');
  });
  socket.on('replay_run_started', (data) => {
    const meta = data.run || {};
    const run = normalizeRun({ ...meta, points: [], lookahead: [] }, meta.id);
    replayRuns = replayRuns.filter((item) => String(item.id) !== String(run.id));
    replayRuns.push(run);
    realtimePath = [];
    lookaheadPoints = [];
    selectedRunId = run.id;
    viewingHistory = false;
    rebuildRunSelect(true);
    markDirty();
  });
  socket.on('replay_run_finished', (data) => {
    const run = runById(data.id);
    if (run) Object.assign(run, { status: data.status, reason: data.reason, end_t: data.end_t });
    rebuildRunSelect(false);
    viewingHistory = Boolean(run);
    if (run) enterHistory(run.id, true);
  });
  socket.on('sync_data', (data) => {
    applyDataset(data, false, 'session');
    setMode(data.mode);
    replaceConsole(data.console);
    setTimeout(() => autoFit(true), 0);
  });
  socket.on('data_cleared', resetLocalData);

  dom.refreshPorts.addEventListener('click', refreshPorts);
  dom.connect.addEventListener('click', () => serialConnected ? socket.emit('disconnect_serial') : connectSerial());
  dom.sendCommand.addEventListener('click', () => sendCommand(dom.command.value));
  dom.commandTemplate.addEventListener('change', () => {
    if (!dom.commandTemplate.value) return;
    dom.command.value = dom.commandTemplate.value;
    dom.command.focus();
    dom.command.select();
    dom.commandTemplate.value = '';
  });
  dom.command.addEventListener('keydown', (event) => {
    if (event.key === 'Enter') { event.preventDefault(); sendCommand(dom.command.value); }
  });
  dom.recordStart.addEventListener('click', () => sendCommand('vpath record'));
  dom.recordStop.addEventListener('click', () => sendCommand('vpath record stop'));
  dom.replayStart.addEventListener('click', () => sendCommand('vpath replay'));
  dom.phototubeCal.addEventListener('click', () => {
    const ready = window.confirm('确认车辆位于蓝底，前方 20-30 cm 有横向白线，且前方区域无障碍。标定会自动行驶两次并停车。');
    if (ready) sendCommand('ptcal drive 0.5');
  });
  dom.emergencyStop.addEventListener('click', () => sendCommand('vstop'));
  dom.clearConsole.addEventListener('click', () => {
    for (let index = pendingSerialEvents.length - 1; index >= 0; index -= 1) {
      if (pendingSerialEvents[index].event === 'serial_console') pendingSerialEvents.splice(index, 1);
    }
    dom.console.innerHTML = '';
  });
  dom.autoFit.addEventListener('click', () => autoFit(true));
  dom.originView.addEventListener('click', focusCoordinateOrigin);
  dom.zoomIn.addEventListener('click', () => zoomAt(1.3, container.clientWidth / 2, container.clientHeight / 2));
  dom.zoomOut.addEventListener('click', () => zoomAt(1 / 1.3, container.clientWidth / 2, container.clientHeight / 2));
  dom.save.addEventListener('click', saveData);
  dom.exportRaw.addEventListener('click', exportRawLog);
  dom.load.addEventListener('click', () => dom.file.click());
  dom.file.addEventListener('change', () => {
    if (dom.file.files[0]) loadFile(dom.file.files[0]);
    dom.file.value = '';
  });
  dom.clear.addEventListener('click', forceClearAll);
  [dom.showPredefined, dom.showRealtime, dom.showSlipCorrection, dom.showPhototubeObservation,
    dom.showLookahead, dom.showLookaheadHistory,
    dom.showSimulationLookahead, dom.showDiagnostics]
    .forEach((input) => input.addEventListener('change', () => { hideTooltip(); markDirty(); }));
  dom.showSimulationLookahead.addEventListener('click', (event) => event.stopPropagation());
  [dom.simulationSteeringLookahead, dom.simulationTangentDistance, dom.simulationSpeedPreview]
    .forEach((input) => input.addEventListener('input', () => {
      updateSimulationLookaheadStatus(renderSeries());
      hideTooltip();
      markDirty();
    }));
  dom.geometryValidation.addEventListener('change', () => {
    setGeometryValidation(dom.geometryValidation.checked);
    if (dom.geometryValidation.checked && workspaceMode === 'live') setWorkspace('planning');
  });
  dom.geometryAlgorithm.addEventListener('change', () => {
    dom.planningAlgorithm.value = dom.geometryAlgorithm.value;
    hideTooltip();
    syncGeometryConstraintControls();
    computeGeometryPlan();
  });
  dom.curvatureLimit.addEventListener('change', () => {
    dom.planningRadiusLimit.checked = dom.curvatureLimit.checked;
    syncGeometryConstraintControls();
    computeGeometryPlan();
  });
  dom.corridorOptimizer.addEventListener('change', computeGeometryPlan);
  dom.minimumRadius.addEventListener('input', () => {
    dom.planningRadius.value = dom.minimumRadius.value;
    scheduleGeometryPlan();
  });
  dom.minimumRadius.addEventListener('change', computeGeometryPlan);

  dom.workspaceTabs.forEach((button) => button.addEventListener('click', () => setWorkspace(button.dataset.workspace)));
  dom.planningAlgorithm.addEventListener('change', () => {
    dom.geometryAlgorithm.value = dom.planningAlgorithm.value;
    computeGeometryPlan();
  });
  dom.planningRadiusLimit.addEventListener('change', () => {
    dom.curvatureLimit.checked = dom.planningRadiusLimit.checked;
    syncGeometryConstraintControls();
    computeGeometryPlan();
  });
  dom.planningRadius.addEventListener('input', () => {
    dom.minimumRadius.value = dom.planningRadius.value;
    scheduleGeometryPlan();
  });
  dom.planningRadius.addEventListener('change', computeGeometryPlan);
  [dom.speedMax, dom.speedLateralAccel, dom.speedLongAccel].forEach((input) => {
    input.addEventListener('input', computeFirmwareSpeedPlan);
  });
  dom.speedPlanEnabled.addEventListener('change', computeFirmwareSpeedPlan);
  dom.planningUpload.addEventListener('click', () => uploadPathToFlash('planning'));
  dom.externalUpload.addEventListener('click', () => uploadPathToFlash('external'));
  [dom.planningUploadCancel, dom.externalUploadCancel].forEach((button) => {
    button.addEventListener('click', () => socket.emit('cancel_path_upload'));
  });
  [dom.lateralOffset, dom.steeringLookahead, dom.tangentDistance, dom.speedPreview].forEach((input) => {
    input.addEventListener('input', () => { syncPlanningProgress(); markDirty(); });
  });
  dom.planningProgress.addEventListener('input', () => { syncPlanningProgress(); hideTooltip(); markDirty(); });
  [dom.showGeometryPlan, dom.showFirmwarePlan, dom.showExternalRoute, dom.showExternalNetwork,
    dom.showExternalSegments, dom.showExternalShoulders].forEach((input) => {
    input.addEventListener('change', () => { hideTooltip(); markDirty(); });
  });

  dom.externalLoad.addEventListener('click', () => dom.externalFile.click());
  dom.externalFile.addEventListener('change', () => {
    if (dom.externalFile.files[0]) loadExternalFile(dom.externalFile.files[0]);
  });
  dom.externalPaste.addEventListener('click', () => {
    dom.externalPasteText.value = '';
    dom.externalPasteDialog.showModal();
    setTimeout(() => dom.externalPasteText.focus(), 0);
  });
  dom.externalPasteImport.addEventListener('click', (event) => {
    event.preventDefault();
    try {
      const text = dom.externalPasteText.value.trim();
      if (!text) throw new Error('粘贴内容为空');
      loadExternalDocumentData(JSON.parse(text), '粘贴内容');
      dom.externalPasteDialog.close();
    } catch (error) {
      showToast(`节点图导入失败：${error.message}`, 'error', 6000);
    }
  });
  [dom.externalSource, dom.externalAlign, dom.externalRotation,
    dom.externalMirrorY, dom.externalReverse, dom.externalOffsetX, dom.externalOffsetY].forEach((input) => {
    input.addEventListener('change', () => applyExternalTransform(true));
  });
  [dom.externalOffsetX, dom.externalOffsetY].forEach((input) => {
    input.addEventListener('input', () => applyExternalTransform(false));
  });
  dom.externalOrigin.addEventListener('change', () => {
    dom.externalOffsetX.value = '0';
    dom.externalOffsetY.value = '0';
    applyExternalTransform(true);
  });
  [dom.externalShoulderDistance, dom.externalShoulderWidth].forEach((input) => {
    input.addEventListener('input', () => applyExternalTransform(false));
    input.addEventListener('change', () => applyExternalTransform(true));
  });
  [dom.vehicleLength, dom.vehicleWidth, dom.vehiclePivot].forEach((input) => {
    input.addEventListener('input', resetVehicleSimulation);
    input.addEventListener('change', resetVehicleSimulation);
  });
  dom.vehicleSimEnabled.addEventListener('change', () => {
    resetVehicleSimulation();
    dom.vehicleSimPanel.hidden = !dom.vehicleSimEnabled.checked;
    if (dom.vehicleSimEnabled.checked && !externalShoulderSegments.length) {
      showToast('可以进行路径运动仿真；导入节点图后才会检测路肩碰撞', 'warning', 5000);
    }
    markDirty();
  });
  [dom.vehicleSimSource, dom.vehicleSimUseSpeed, dom.vehicleSimFixedSpeed].forEach((input) => {
    input.addEventListener('change', () => {
      resetVehicleSimulation();
      dom.vehicleSimPlay.textContent = '从起点播放';
      markDirty();
    });
  });
  dom.vehicleSimPlay.addEventListener('click', () => {
    if (vehicleSimulation.playing) {
      vehicleSimulation.playing = false;
      dom.vehicleSimPlay.textContent = '继续';
      updateVehicleSimulationPanel();
      markDirty();
      return;
    }
    dom.vehicleSimEnabled.checked = true;
    dom.vehicleSimPanel.hidden = false;
    syncVehicleSimulation(renderSeries());
    if (!vehicleSimulation.path || vehicleSimulation.path.points.length < 2) {
      showToast('所选路径没有足够的点，无法仿真', 'warning');
      return;
    }
    if (vehicleSimulation.elapsedSec >= vehicleSimulation.path.totalSec - 1e-6) {
      vehicleSimulation.elapsedSec = 0;
      vehicleSimulation.distanceMm = 0;
      vehicleSimulation.events = [];
      vehicleSimulation.colliding = false;
      const pose = window.VehicleSimulator.poseAtTime(vehicleSimulation.path, 0);
      if (pose) evaluateReplayVehiclePose(pose, 0, 0, false);
    }
    vehicleSimulation.playing = true;
    vehicleSimulation.lastFrame = performance.now();
    dom.vehicleSimPlay.textContent = '暂停';
    updateVehicleSimulationPanel();
    markDirty();
  });
  dom.vehicleSimProgress.addEventListener('input', () => {
    if (!vehicleSimulation.path) return;
    vehicleSimulation.playing = false;
    vehicleSimulation.lastFrame = performance.now();
    const elapsedSec = Math.max(0, Math.min(
      vehicleSimulation.path.totalSec,
      finiteNumber(dom.vehicleSimProgress.value, 0)
    ));
    if (vehicleSimulation.sourceKey.startsWith('replay:history:')) {
      playback.playing = false;
      playback.time = elapsedSec;
      syncPlaybackUI();
    }
    const pose = window.VehicleSimulator.poseAtTime(vehicleSimulation.path, elapsedSec);
    if (pose) evaluateReplayVehiclePose(pose, pose.s_mm, elapsedSec, false);
    dom.vehicleSimPlay.textContent = elapsedSec >= vehicleSimulation.path.totalSec - 1e-6
      ? '从起点播放' : '继续';
    updateVehicleSimulationPanel();
    hideTooltip();
    markDirty();
  });
  dom.vehicleSimClear.addEventListener('click', () => {
    vehicleSimulation.events = [];
    vehicleSimulation.colliding = Boolean(vehicleSimulation.result && vehicleSimulation.result.collided);
    updateVehicleSimulationPanel();
    markDirty();
  });
  dom.externalUseRaw.addEventListener('click', () => {
    const selected = dom.externalSource.value === 'segments' ? externalSegmentRoute : externalRoute;
    if (selected.length < 2) {
      showToast('当前外部路线没有足够的有效点', 'warning');
      return;
    }
    predefinedPath = selected.map((point) => ({ ...point }));
    baselineSource = 'external';
    geometryPlan = null;
    dom.geometryValidation.checked = true;
    setWorkspace('planning');
    computeGeometryPlan();
    showToast(`已将 ${selected.length} 个外部点作为规划原始路径`, 'success');
  });

  dom.runSelect.addEventListener('change', () => {
    if (dom.runSelect.value) enterHistory(dom.runSelect.value, true);
  });
  dom.liveView.addEventListener('click', returnToLive);
  dom.playbackReset.addEventListener('click', () => {
    if (!viewingHistory && selectedRunId !== null) enterHistory(selectedRunId, false);
    playback.playing = false;
    playback.time = 0;
    syncPlaybackUI();
    markDirty();
  });
  dom.playbackToggle.addEventListener('click', () => {
    if (!selectedRunId) return;
    if (!viewingHistory) enterHistory(selectedRunId, false);
    if (playback.time >= playback.duration) playback.time = 0;
    playback.playing = !playback.playing;
    playback.lastFrame = performance.now();
    syncPlaybackUI();
    markDirty();
  });
  dom.playbackSlider.addEventListener('input', () => {
    if (!selectedRunId) return;
    viewingHistory = true;
    playback.playing = false;
    playback.time = finiteNumber(dom.playbackSlider.value, 0);
    syncPlaybackUI();
    hideTooltip();
    markDirty();
  });
  dom.playbackSpeed.addEventListener('change', () => { playback.speed = finiteNumber(dom.playbackSpeed.value, 1); });

  dom.theme.addEventListener('click', () => {
    isDark = !isDark;
    document.documentElement.dataset.theme = isDark ? 'dark' : 'light';
    localStorage.setItem('pathcapture-theme', isDark ? 'dark' : 'light');
    drawCurvatureChart();
    drawSpeedChart();
    markDirty();
  });

  container.addEventListener('wheel', (event) => {
    event.preventDefault();
    const rect = container.getBoundingClientRect();
    zoomAt(event.deltaY < 0 ? 1.12 : 1 / 1.12, event.clientX - rect.left, event.clientY - rect.top);
    dom.zoomHint.hidden = true;
  }, { passive: false });
  container.addEventListener('mousedown', (event) => {
    if (event.button !== 0) return;
    drag.active = true;
    Object.assign(drag, { x: event.clientX, y: event.clientY, ox: view.ox, oy: view.oy });
    container.classList.add('panning');
    hideTooltip();
  });
  window.addEventListener('mousemove', (event) => {
    const rect = container.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;
    mouseWorld = screenToWorld(x, y);
    if (drag.active) {
      view.ox = drag.ox + event.clientX - drag.x;
      view.oy = drag.oy + event.clientY - drag.y;
      markDirty();
      return;
    }
    if (x < 0 || y < 0 || x > rect.width || y > rect.height) { hideTooltip(); return; }
    const next = nearestPoint(x, y);
    if (next) showTooltip(next, event.clientX, event.clientY);
    else hideTooltip();
    markDirty();
  });
  window.addEventListener('mouseup', () => { drag.active = false; container.classList.remove('panning'); });
  container.addEventListener('mouseleave', () => { if (!drag.active) hideTooltip(); });

  function initialize() {
    isDark = localStorage.getItem('pathcapture-theme') === 'dark';
    document.documentElement.dataset.theme = isDark ? 'dark' : 'light';
    resizeCanvas();
    view.ox = container.clientWidth / 2;
    view.oy = container.clientHeight / 2;
    rebuildRunSelect();
    setMode(runtimeMode);
    setGeometryValidation(false);
    setWorkspace('live');
    syncPlaybackUI();
    updateVehicleSimulationPanel();
    refreshPorts();
    if (window.ResizeObserver) new ResizeObserver(resizeCanvas).observe(container);
    if (window.ResizeObserver) new ResizeObserver(resizeCurvatureCanvas).observe(curvatureCanvas);
    if (window.ResizeObserver) new ResizeObserver(resizeSpeedCanvas).observe(speedCanvas);
    window.addEventListener('resize', resizeCanvas);
    window.addEventListener('resize', resizeCurvatureCanvas);
    window.addEventListener('resize', resizeSpeedCanvas);
    requestAnimationFrame(renderLoop);
  }

  initialize();
})();
