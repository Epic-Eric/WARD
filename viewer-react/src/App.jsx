import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { OrbitControls, Grid, Html, Line, Text } from "@react-three/drei";
import { useEffect, useMemo, useRef, useState } from "react";
import * as THREE from "three";
import AlertToast from "./components/AlertToast";
import CommandBar from "./components/CommandBar";
import ConfirmDialog from "./components/ConfirmDialog";
import HelpDialog from "./components/HelpDialog";
import { aimAtCluster, installBaselineBundle, setAutoClearDebris as apiSetAutoClearDebris } from "./lib/controlServerClient";

function emptyPayload(mode = "raw") {
  return {
    frame_id: null,
    points: [],
    samples: [],
    clusters: [],
    top_clusters: [],
    render_mode: mode,
    requested_mode: mode,
    baseline_available: false,
    residual_tolerance_mm: 25,
  };
}

async function fetchJson(url) {
  const response = await fetch(url);
  if (!response.ok) {
    throw new Error(`Request failed: ${response.status}`);
  }
  return response.json();
}

function sanitizePayload(payload, fallbackMode) {
  const nextPayload = payload && typeof payload === "object" ? payload : {};
  return {
    ...emptyPayload(fallbackMode),
    ...nextPayload,
    points: Array.isArray(nextPayload.points) ? nextPayload.points : [],
    samples: Array.isArray(nextPayload.samples) ? nextPayload.samples : [],
    clusters: Array.isArray(nextPayload.clusters) ? nextPayload.clusters : [],
    top_clusters: Array.isArray(nextPayload.top_clusters) ? nextPayload.top_clusters : [],
    render_mode: typeof nextPayload.render_mode === "string" ? nextPayload.render_mode : fallbackMode,
    requested_mode: typeof nextPayload.requested_mode === "string" ? nextPayload.requested_mode : fallbackMode,
  };
}

function normalizeImportedBundle(bundle, importedName = "imported_bundle.json") {
  if (!bundle || typeof bundle !== "object") {
    throw new Error("Imported file is not a valid bundle.");
  }
  if (bundle.format !== "ward-viewer-bundle") {
    throw new Error("Unsupported bundle format.");
  }

  const views = bundle.views && typeof bundle.views === "object" ? bundle.views : {};
  return {
    format: "ward-viewer-bundle",
    version: Number(bundle.version ?? 1),
    exported_at: bundle.exported_at ?? null,
    imported_name: importedName,
    status: bundle.status && typeof bundle.status === "object" ? bundle.status : null,
    views: {
      raw: views.raw ? sanitizePayload(views.raw, "raw") : null,
      residual: views.residual ? sanitizePayload(views.residual, "residual") : null,
      baseline: views.baseline ? sanitizePayload(views.baseline, "baseline") : null,
    },
  };
}

function chooseDefaultMode(bundle) {
  if (bundle?.views?.raw) {
    return "raw";
  }
  if (bundle?.views?.residual) {
    return "residual";
  }
  if (bundle?.views?.baseline) {
    return "baseline";
  }
  return "raw";
}

function selectImportedPayload(bundle, requestedMode, baselineHoldActive) {
  const effectiveMode = baselineHoldActive ? "baseline" : requestedMode;
  const selectedPayload = bundle?.views?.[effectiveMode];
  if (selectedPayload) {
    return selectedPayload;
  }
  if (bundle?.views?.raw) {
    return bundle.views.raw;
  }
  if (bundle?.views?.residual) {
    return bundle.views.residual;
  }
  if (bundle?.views?.baseline) {
    return bundle.views.baseline;
  }
  return emptyPayload("raw");
}

function downloadBundle(bundle, filename) {
  const blob = new Blob([JSON.stringify(bundle, null, 2)], {
    type: "application/json",
  });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement("a");
  anchor.href = url;
  anchor.download = filename;
  anchor.click();
  URL.revokeObjectURL(url);
}

function exportFilename() {
  const timestamp = new Date().toISOString().replaceAll(":", "-");
  return `ward_scan_bundle_${timestamp}.json`;
}

function useStatusData(intervalMs = 1000) {
  const [status, setStatus] = useState(null);

  useEffect(() => {
    let cancelled = false;
    let inFlight = false;
    let controller = null;

    async function load() {
      if (cancelled || inFlight) {
        return;
      }
      inFlight = true;
      controller = new AbortController();
      try {
        const response = await fetch("/api/status", { signal: controller.signal });
        const payload = await response.json();
        if (!cancelled) {
          setStatus(payload);
        }
      } catch (error) {
        if (error?.name === "AbortError") {
          return;
        }
        if (!cancelled) {
          setStatus(null);
        }
      } finally {
        inFlight = false;
        controller = null;
      }
    }

    load();
    const timer = window.setInterval(load, intervalMs);
    return () => {
      cancelled = true;
      controller?.abort();
      window.clearInterval(timer);
    };
  }, [intervalMs]);

  return status;
}

function useLivePoints(mode, residualTolerance, refreshNonce = 0, enabled = true, intervalMs = 500) {
  const [payload, setPayload] = useState(emptyPayload(mode));

  useEffect(() => {
    if (!enabled) {
      return undefined;
    }

    let cancelled = false;
    let inFlight = false;
    let controller = null;

    async function load() {
      if (cancelled || inFlight) {
        return;
      }
      inFlight = true;
      controller = new AbortController();
      try {
        const response = await fetch(
          `/api/live-points?mode=${encodeURIComponent(mode)}&tolerance_mm=${encodeURIComponent(residualTolerance)}`,
          { signal: controller.signal },
        );
        const nextPayload = await response.json();
        if (!cancelled) {
          setPayload(nextPayload);
        }
      } catch (error) {
        if (error?.name === "AbortError") {
          return;
        }
        if (!cancelled) {
          setPayload(emptyPayload(mode));
        }
      } finally {
        inFlight = false;
        controller = null;
      }
    }

    load();
    const timer = window.setInterval(load, intervalMs);
    return () => {
      cancelled = true;
      controller?.abort();
      window.clearInterval(timer);
    };
  }, [mode, residualTolerance, refreshNonce, enabled, intervalMs]);

  return payload;
}

function colorForDistance(distanceMm, renderMode) {
  const color = new THREE.Color();

  if (renderMode === "residual") {
    const magnitude = Math.min(1, Math.abs(distanceMm) / 250);
    color.setRGB(0.86, 0.32 + 0.28 * (1 - magnitude), 0.18);
    return color;
  }

  if (renderMode === "baseline") {
    const t = Math.min(1, Math.max(0, distanceMm / 1500));
    color.setRGB(0.72 + 0.14 * t, 0.56 + 0.18 * (1 - t), 0.36);
    return color;
  }

  const t = Math.min(1, Math.max(0, distanceMm / 1500));
  color.setRGB(0.1 + 0.7 * t, 0.75 - 0.35 * t, 0.3 + 0.4 * (1 - t));
  return color;
}

function colorMetricForPoint(point, renderMode) {
  if (renderMode === "residual") {
    return Number(point[5] ?? point[3] ?? 0);
  }
  return Number(point[3] ?? 0);
}

function colorMetricForSample(sample, renderMode) {
  if (renderMode === "residual") {
    return Number(sample.occlusion_distance_mm ?? sample.distance_mm ?? 0);
  }
  return Number(sample.distance_mm ?? 0);
}

function clusterAccentColor(clusterId, selectedClusterId) {
  if (!selectedClusterId || clusterId === selectedClusterId) {
    return "#b0493a";
  }
  return "#6d777c";
}

function formatClusterBadgeLabel(clusterId) {
  if (typeof clusterId !== "string" || clusterId.length === 0) {
    return "";
  }
  if (clusterId.startsWith("debris-")) {
    return clusterId.slice("debris-".length).replaceAll("-", "/");
  }
  return clusterId;
}

const LABEL_GROUP_PIXEL_RADIUS = 54;
const LABEL_GROUP_MODE_DISTANCE = 2.45;

function averageWorldPosition(entries) {
  return entries
    .reduce((accumulator, entry) => accumulator.add(entry.worldPosition), new THREE.Vector3(0, 0, 0))
    .multiplyScalar(1 / entries.length);
}

function strongestClusterFromEntries(entries) {
  return entries
    .map((entry) => entry.cluster)
    .sort((left, right) => (Number(right.score) || 0) - (Number(left.score) || 0))[0];
}

function averageScreenPosition(entries) {
  const total = entries.reduce(
    (accumulator, entry) => {
      accumulator.x += entry.screenX;
      accumulator.y += entry.screenY;
      return accumulator;
    },
    { x: 0, y: 0 },
  );
  return {
    x: total.x / entries.length,
    y: total.y / entries.length,
  };
}

function cleanupGroupedLabels(labels, suppressionRadiusPx) {
  if (!Array.isArray(labels) || labels.length === 0) {
    return [];
  }

  const groupedLabels = labels.filter((label) => label.type === "group");
  if (groupedLabels.length === 0) {
    return labels;
  }

  const coveredClusterIds = new Set(
    groupedLabels.flatMap((label) => Array.isArray(label.memberClusterIds) ? label.memberClusterIds : []),
  );

  return labels.filter((label) => {
    if (label.type !== "single") {
      return true;
    }
    if (coveredClusterIds.has(label.cluster?.cluster_id)) {
      return false;
    }

    return !groupedLabels.some((groupLabel) => {
      const deltaX = (groupLabel.screenX ?? 0) - (label.screenX ?? 0);
      const deltaY = (groupLabel.screenY ?? 0) - (label.screenY ?? 0);
      return (deltaX * deltaX + deltaY * deltaY) <= suppressionRadiusPx * suppressionRadiusPx;
    });
  });
}

function clusterLabelPosition(cluster) {
  const bbox = cluster?.bbox;
  if (!bbox) {
    return null;
  }
  const labelOffset = Math.max(Number(bbox.size_z_mm) / 2000 + 0.05, 0.06);
  return new THREE.Vector3(
    Number(bbox.center_x_mm) / 1000,
    Number(bbox.center_y_mm) / 1000,
    Number(bbox.center_z_mm) / 1000 + labelOffset,
  );
}

function buildClusterLabelGroups(clusters, camera, size, selectedClusterId) {
  if (!Array.isArray(clusters) || clusters.length === 0 || !camera || !size?.width || !size?.height) {
    return [];
  }

  const rawEntries = clusters
    .map((cluster) => {
      const worldPosition = clusterLabelPosition(cluster);
      if (!worldPosition) {
        return null;
      }
      const projected = worldPosition.clone().project(camera);
      if (projected.z < -1 || projected.z > 1) {
        return null;
      }
      return {
        cluster,
        worldPosition,
        screenX: ((projected.x + 1) / 2) * size.width,
        screenY: ((1 - projected.y) / 2) * size.height,
        selected: cluster.cluster_id === selectedClusterId,
      };
    })
    .filter(Boolean);

  const compactMode = camera.position.length() >= LABEL_GROUP_MODE_DISTANCE;
  const freeEntries = rawEntries;
  const visited = new Set();
  const groupingRadiusPx = compactMode ? LABEL_GROUP_PIXEL_RADIUS * 1.75 : LABEL_GROUP_PIXEL_RADIUS;
  const preliminaryGroups = [];

  for (let index = 0; index < freeEntries.length; index += 1) {
    if (visited.has(index)) {
      continue;
    }

    const seed = freeEntries[index];
    const stack = [index];
    const groupIndexes = [];
    visited.add(index);

    while (stack.length > 0) {
      const currentIndex = stack.pop();
      groupIndexes.push(currentIndex);
      const current = freeEntries[currentIndex];

      for (let candidateIndex = 0; candidateIndex < freeEntries.length; candidateIndex += 1) {
        if (visited.has(candidateIndex)) {
          continue;
        }
        const candidate = freeEntries[candidateIndex];
        const deltaX = candidate.screenX - current.screenX;
        const deltaY = candidate.screenY - current.screenY;
        if ((deltaX * deltaX + deltaY * deltaY) <= groupingRadiusPx * groupingRadiusPx) {
          visited.add(candidateIndex);
          stack.push(candidateIndex);
        }
      }
    }

    preliminaryGroups.push(groupIndexes.map((groupIndex) => freeEntries[groupIndex]));
  }

  const mergedGroups = [];
  const groupedIndexes = new Set();
  const mergeRadiusPx = compactMode ? groupingRadiusPx * 1.15 : groupingRadiusPx * 0.95;

  for (let index = 0; index < preliminaryGroups.length; index += 1) {
    if (groupedIndexes.has(index)) {
      continue;
    }

    const queue = [index];
    const mergedEntries = [];
    groupedIndexes.add(index);

    while (queue.length > 0) {
      const currentIndex = queue.pop();
      const currentGroup = preliminaryGroups[currentIndex];
      mergedEntries.push(...currentGroup);
      const currentCenter = averageScreenPosition(currentGroup);

      for (let candidateIndex = 0; candidateIndex < preliminaryGroups.length; candidateIndex += 1) {
        if (groupedIndexes.has(candidateIndex)) {
          continue;
        }
        const candidateGroup = preliminaryGroups[candidateIndex];
        const shouldConsiderMerge =
          compactMode || currentGroup.length > 1 || candidateGroup.length > 1;
        if (!shouldConsiderMerge) {
          continue;
        }
        const candidateCenter = averageScreenPosition(candidateGroup);
        const deltaX = candidateCenter.x - currentCenter.x;
        const deltaY = candidateCenter.y - currentCenter.y;
        if ((deltaX * deltaX + deltaY * deltaY) <= mergeRadiusPx * mergeRadiusPx) {
          groupedIndexes.add(candidateIndex);
          queue.push(candidateIndex);
        }
      }
    }

    mergedGroups.push(mergedEntries);
  }

  const multiGroups = mergedGroups.filter((entries) => entries.length > 1);
  const suppressionRadiusPx = groupingRadiusPx * 0.85;
  const groupedLabels = [];

  for (const entries of mergedGroups) {
    if (entries.length > 1) {
      const worldPosition = averageWorldPosition(entries);
      const strongestCluster = strongestClusterFromEntries(entries);
      groupedLabels.push({
        key: `group-${entries.map((entry) => entry.cluster.cluster_id).sort().join("-")}`,
        type: "group",
        count: entries.length,
        strongestScore: Number(strongestCluster?.score) || 0,
        strongestClusterId: strongestCluster?.cluster_id ?? "",
        strongestOcclusionMm: Number(strongestCluster?.max_occlusion_mm) || 0,
        worldPosition,
        screenX: averageScreenPosition(entries).x,
        screenY: averageScreenPosition(entries).y,
        memberClusterIds: entries.map((entry) => entry.cluster.cluster_id),
      });
      continue;
    }

    const entry = entries[0];
    const entryCenter = averageScreenPosition(entries);
    const suppressedByNearbyGroup = multiGroups.some((groupEntries) => {
      const groupCenter = averageScreenPosition(groupEntries);
      const deltaX = groupCenter.x - entryCenter.x;
      const deltaY = groupCenter.y - entryCenter.y;
      return (deltaX * deltaX + deltaY * deltaY) <= suppressionRadiusPx * suppressionRadiusPx;
    });
    if (suppressedByNearbyGroup) {
      continue;
    }

    groupedLabels.push({
      key: `single-${entry.cluster.cluster_id}`,
      type: "single",
      cluster: entry.cluster,
      worldPosition: entry.worldPosition,
      selected: entry.selected,
      screenX: entry.screenX,
      screenY: entry.screenY,
    });
  }

  return cleanupGroupedLabels(groupedLabels, suppressionRadiusPx);
}

function buildHoveredPoint(entry, renderedPositionMm = null) {
  if (!entry) {
    return null;
  }

  const point = Array.isArray(entry.point) ? entry.point : [];
  const sample = entry.sample && typeof entry.sample === "object" ? entry.sample : {};
  const renderedX = Number.isFinite(renderedPositionMm?.[0]) ? renderedPositionMm[0] : Number(point[0] ?? sample.x_mm);
  const renderedY = Number.isFinite(renderedPositionMm?.[1]) ? renderedPositionMm[1] : Number(point[1] ?? sample.y_mm);
  const renderedZ = Number.isFinite(renderedPositionMm?.[2]) ? renderedPositionMm[2] : Number(point[2] ?? sample.z_mm);

  return {
    ...sample,
    x_mm: renderedX,
    y_mm: renderedY,
    z_mm: renderedZ,
    target_x_mm: Number(point[0] ?? sample.x_mm ?? renderedX),
    target_y_mm: Number(point[1] ?? sample.y_mm ?? renderedY),
    target_z_mm: Number(point[2] ?? sample.z_mm ?? renderedZ),
  };
}

function PointCloud({ points, samples, renderMode, onHoverChange }) {
  const pointsRef = useRef(null);
  const geometryRef = useRef(null);
  const targetPositionsRef = useRef(new Float32Array(0));
  const animatedPositionsRef = useRef(new Float32Array(0));
  const drawableEntries = useMemo(() => {
    const count = Math.min(points.length, samples.length);
    const entries = [];
    for (let index = 0; index < count; index += 1) {
      const point = points[index];
      const sample = samples[index];
      if (Number(point?.[4] ?? 0) >= 0.5) {
        continue;
      }
      entries.push({ point, sample });
    }
    return entries;
  }, [points, samples]);

  useEffect(() => {
    const targetPositions = new Float32Array(drawableEntries.length * 3);
    const colorArray = new Float32Array(drawableEntries.length * 3);

    drawableEntries.forEach(({ point }, index) => {
      const [xMm, yMm, zMm] = point;
      targetPositions[index * 3 + 0] = xMm / 1000;
      targetPositions[index * 3 + 1] = yMm / 1000;
      targetPositions[index * 3 + 2] = zMm / 1000;

      const color = colorForDistance(colorMetricForPoint(point, renderMode), renderMode);
      colorArray[index * 3 + 0] = color.r;
      colorArray[index * 3 + 1] = color.g;
      colorArray[index * 3 + 2] = color.b;
    });

    const nextAnimatedPositions = new Float32Array(targetPositions.length);
    const previousAnimated = animatedPositionsRef.current;
    for (let index = 0; index < targetPositions.length; index += 3) {
      if (renderMode === "raw" && index < previousAnimated.length) {
        nextAnimatedPositions[index + 0] = previousAnimated[index + 0];
        nextAnimatedPositions[index + 1] = previousAnimated[index + 1];
        nextAnimatedPositions[index + 2] = previousAnimated[index + 2];
      } else if (renderMode === "raw") {
        nextAnimatedPositions[index + 0] = 0;
        nextAnimatedPositions[index + 1] = 0;
        nextAnimatedPositions[index + 2] = 0;
      } else {
        nextAnimatedPositions[index + 0] = targetPositions[index + 0];
        nextAnimatedPositions[index + 1] = targetPositions[index + 1];
        nextAnimatedPositions[index + 2] = targetPositions[index + 2];
      }
    }

    targetPositionsRef.current = targetPositions;
    animatedPositionsRef.current = nextAnimatedPositions;

    if (!geometryRef.current) {
      return;
    }

    geometryRef.current.setAttribute(
      "position",
      new THREE.BufferAttribute(animatedPositionsRef.current, 3),
    );
    geometryRef.current.setAttribute(
      "color",
      new THREE.BufferAttribute(colorArray, 3),
    );
    geometryRef.current.computeBoundingSphere();
  }, [drawableEntries, renderMode]);

  useFrame((_, deltaSeconds) => {
    if (renderMode !== "raw" || !geometryRef.current) {
      return;
    }

    const positionAttribute = geometryRef.current.getAttribute("position");
    if (!positionAttribute) {
      return;
    }

    const animatedPositions = animatedPositionsRef.current;
    const targetPositions = targetPositionsRef.current;
    const blend = 1 - Math.exp(-deltaSeconds * 10);
    let changed = false;

    for (let index = 0; index < targetPositions.length; index += 1) {
      const current = animatedPositions[index];
      const target = targetPositions[index];
      const next = current + (target - current) * blend;
      if (Math.abs(target - next) > 0.0005) {
        changed = true;
      }
      animatedPositions[index] = next;
    }

    positionAttribute.needsUpdate = true;
    if (changed) {
      geometryRef.current.computeBoundingSphere();
    }
  });

  useEffect(() => () => onHoverChange?.(null), [onHoverChange]);

  if (drawableEntries.length === 0) {
    return null;
  }

  return (
    <points
      ref={pointsRef}
      onPointerMove={(event) => {
        if (typeof event.index !== "number") {
          return;
        }
        const entry = drawableEntries[event.index];
        if (entry) {
          const renderedPosition = geometryRef.current?.getAttribute("position")
            ? [
                Number(geometryRef.current.getAttribute("position").getX(event.index) * 1000),
                Number(geometryRef.current.getAttribute("position").getY(event.index) * 1000),
                Number(geometryRef.current.getAttribute("position").getZ(event.index) * 1000),
              ]
            : null;
          onHoverChange?.(buildHoveredPoint(entry, renderedPosition));
        }
      }}
      onPointerOut={() => onHoverChange?.(null)}
    >
      <bufferGeometry ref={geometryRef} />
      <pointsMaterial size={0.022} vertexColors sizeAttenuation />
    </points>
  );
}

function SurfaceMesh({ samples, renderMode, opacityOverride = null }) {
  const geometry = useMemo(() => {
    if (samples.length < 4) {
      return null;
    }

    const yawValues = [...new Set(samples.map((sample) => Number(sample.yaw_deg.toFixed(2))))].sort(
      (left, right) => left - right,
    );
    const pitchValues = [...new Set(samples.map((sample) => Number(sample.pitch_deg.toFixed(2))))].sort(
      (left, right) => left - right,
    );
    if (yawValues.length < 2 || pitchValues.length < 2) {
      return null;
    }

    const sampleMap = new Map(
      samples.map((sample) => [
        `${Number(sample.yaw_deg).toFixed(2)}|${Number(sample.pitch_deg).toFixed(2)}`,
        sample,
      ]),
    );

    const positions = [];
    const colors = [];
    const indices = [];
    const vertexIndex = new Map();

    function ensureVertex(yawDeg, pitchDeg) {
      const key = `${yawDeg.toFixed(2)}|${pitchDeg.toFixed(2)}`;
      const sample = sampleMap.get(key);
      if (!sample || Boolean(sample.is_air)) {
        return null;
      }
      if (vertexIndex.has(key)) {
        return vertexIndex.get(key);
      }

      const index = positions.length / 3;
      positions.push(sample.x_mm / 1000, sample.y_mm / 1000, sample.z_mm / 1000);
      const color = colorForDistance(colorMetricForSample(sample, renderMode), renderMode);
      colors.push(color.r, color.g, color.b);
      vertexIndex.set(key, index);
      return index;
    }

    for (let yawIndex = 0; yawIndex < yawValues.length - 1; yawIndex += 1) {
      for (let pitchIndex = 0; pitchIndex < pitchValues.length - 1; pitchIndex += 1) {
        const a = ensureVertex(yawValues[yawIndex], pitchValues[pitchIndex]);
        const b = ensureVertex(yawValues[yawIndex + 1], pitchValues[pitchIndex]);
        const c = ensureVertex(yawValues[yawIndex], pitchValues[pitchIndex + 1]);
        const d = ensureVertex(yawValues[yawIndex + 1], pitchValues[pitchIndex + 1]);
        if ([a, b, c, d].some((value) => value == null)) {
          continue;
        }

        indices.push(a, b, c);
        indices.push(b, d, c);
      }
    }

    if (indices.length === 0) {
      return null;
    }

    const nextGeometry = new THREE.BufferGeometry();
    nextGeometry.setAttribute(
      "position",
      new THREE.Float32BufferAttribute(positions, 3),
    );
    nextGeometry.setAttribute(
      "color",
      new THREE.Float32BufferAttribute(colors, 3),
    );
    nextGeometry.setIndex(indices);
    nextGeometry.computeVertexNormals();
    return nextGeometry;
  }, [samples, renderMode]);

  if (!geometry) {
    return null;
  }

  return (
    <mesh geometry={geometry}>
      <meshStandardMaterial
        vertexColors
        side={THREE.DoubleSide}
        transparent
        opacity={
          opacityOverride ?? (renderMode === "residual" ? 0.9 : renderMode === "baseline" ? 0.72 : 0.76)
        }
      />
    </mesh>
  );
}

function SelectedClusterPoints({ points }) {
  const geometry = useMemo(() => {
    const drawablePoints = points.filter((point) => Number(point[4] ?? 0) < 0.5);
    if (drawablePoints.length === 0) {
      return null;
    }

    const positionArray = new Float32Array(drawablePoints.length * 3);
    drawablePoints.forEach((point, index) => {
      positionArray[index * 3 + 0] = Number(point[0]) / 1000;
      positionArray[index * 3 + 1] = Number(point[1]) / 1000;
      positionArray[index * 3 + 2] = Number(point[2]) / 1000;
    });

    const nextGeometry = new THREE.BufferGeometry();
    nextGeometry.setAttribute("position", new THREE.BufferAttribute(positionArray, 3));
    return nextGeometry;
  }, [points]);

  if (!geometry) {
    return null;
  }

  return (
    <points geometry={geometry}>
      <pointsMaterial size={0.04} color="#b0493a" sizeAttenuation />
    </points>
  );
}

function HoveredPointMarker({ sample }) {
  if (!sample) {
    return null;
  }

  return (
    <group position={[Number(sample.x_mm) / 1000, Number(sample.y_mm) / 1000, Number(sample.z_mm) / 1000]}>
      <mesh>
        <sphereGeometry args={[0.018, 18, 18]} />
        <meshStandardMaterial
          color="#fff3d4"
          emissive="#ffb347"
          emissiveIntensity={0.75}
        />
      </mesh>
      <mesh>
        <sphereGeometry args={[0.028, 18, 18]} />
        <meshBasicMaterial color="#ff8c42" wireframe transparent opacity={0.85} />
      </mesh>
    </group>
  );
}

function DebrisClusterBox({ cluster, selectedClusterId, onContextMenu }) {
  const bbox = cluster?.bbox;
  const size = useMemo(() => {
    if (!bbox) return null;
    return {
      x: Math.max(Number(bbox.size_x_mm) / 1000, 0.02),
      y: Math.max(Number(bbox.size_y_mm) / 1000, 0.02),
      z: Math.max(Number(bbox.size_z_mm) / 1000, 0.02),
    };
  }, [bbox]);

  const edgesGeometry = useMemo(() => {
    if (!size) return null;
    return new THREE.EdgesGeometry(new THREE.BoxGeometry(size.x, size.y, size.z));
  }, [size]);

  if (!bbox || !size || !edgesGeometry) {
    return null;
  }

  const selected = cluster.cluster_id === selectedClusterId;
  const color = clusterAccentColor(cluster.cluster_id, selectedClusterId);
  const center = [
    Number(bbox.center_x_mm) / 1000,
    Number(bbox.center_y_mm) / 1000,
    Number(bbox.center_z_mm) / 1000,
  ];
  return (
    <group position={center}>
      <lineSegments geometry={edgesGeometry}>
        <lineBasicMaterial color={color} linewidth={selected ? 2 : 1} />
      </lineSegments>
      {onContextMenu && (
        <mesh
          onContextMenu={(e) => {
            e.stopPropagation();
            e.nativeEvent.preventDefault();
            onContextMenu(e, cluster);
          }}
        >
          <boxGeometry args={[size.x, size.y, size.z]} />
          <meshBasicMaterial transparent opacity={0} depthWrite={false} />
        </mesh>
      )}
    </group>
  );
}

function DebrisClusterLabels({ clusters, selectedClusterId }) {
  const { camera, size } = useThree();
  const [labels, setLabels] = useState([]);
  const signatureRef = useRef("");

  useFrame(() => {
    const nextLabels = buildClusterLabelGroups(clusters, camera, size, selectedClusterId);
    const nextSignature = JSON.stringify(
      nextLabels.map((label) => ({
        key: label.key,
        type: label.type,
        count: label.count ?? 1,
        clusterId: label.cluster?.cluster_id ?? null,
        score: label.cluster?.score ?? label.strongestScore ?? null,
        x: Number(label.worldPosition.x.toFixed(3)),
        y: Number(label.worldPosition.y.toFixed(3)),
        z: Number(label.worldPosition.z.toFixed(3)),
      })),
    );
    if (nextSignature === signatureRef.current) {
      return;
    }
    signatureRef.current = nextSignature;
    setLabels(nextLabels);
  });

  if (labels.length === 0) {
    return null;
  }

  return (
    <group>
      {labels.map((label) => (
        <Html
          key={label.key}
          position={label.worldPosition.toArray()}
          center
          distanceFactor={10}
          zIndexRange={[2, 0]}
        >
          {label.type === "group" ? (
            <div
              className="cluster-label cluster-label--group"
              title={`${label.count} debris · strongest ${label.strongestClusterId} · score ${label.strongestScore.toFixed(1)} · occ ${label.strongestOcclusionMm.toFixed(1)} mm`}
            >
              <strong>{label.count}</strong>
            </div>
          ) : (
            <div
              className={`cluster-label ${label.selected ? "active" : ""}`}
              title={label.cluster.cluster_id}
            >
              <strong>{Number(label.cluster.score).toFixed(1)}</strong>
              <span>{formatClusterBadgeLabel(label.cluster.cluster_id)}</span>
            </div>
          )}
        </Html>
      ))}
    </group>
  );
}

function DebrisClusterBoxes({ clusters, selectedClusterId, onClusterContextMenu }) {
  if (!Array.isArray(clusters) || clusters.length === 0) {
    return null;
  }

  return (
    <group>
      {clusters.map((cluster) => (
        <DebrisClusterBox
          key={cluster.cluster_id}
          cluster={cluster}
          selectedClusterId={selectedClusterId}
          onContextMenu={onClusterContextMenu}
        />
      ))}
      <DebrisClusterLabels clusters={clusters} selectedClusterId={selectedClusterId} />
    </group>
  );
}

function facingVector(yawDeg, pitchDeg, length = 0.02) {
  if (!Number.isFinite(yawDeg) || !Number.isFinite(pitchDeg)) {
    return [length, 0, 0];
  }

  const yawRad = (yawDeg * Math.PI) / 180;
  const pitchRad = (pitchDeg * Math.PI) / 180;
  const cosPitch = Math.cos(pitchRad);
  return [
    length * cosPitch * Math.cos(yawRad),
    length * cosPitch * Math.sin(yawRad),
    length * Math.sin(pitchRad),
  ];
}

function SensorRig({ yawDeg, pitchDeg }) {
  const facing = facingVector(yawDeg, pitchDeg);

  return (
    <group>
      <mesh position={[0, 0, 0]}>
        <sphereGeometry args={[0.007, 24, 24]} />
        <meshStandardMaterial color="hsl(242, 68%, 54%)" />
      </mesh>
      <Line points={[[0, 0, 0], facing]} color="#20343b" lineWidth={2} />
    </group>
  );
}

function MiniViewCubeFace({ position, rotation, color, label, onClick }) {
  return (
    <group position={position} rotation={rotation} onClick={(event) => {
      event.stopPropagation();
      onClick?.();
    }}>
      <mesh>
        <planeGeometry args={[0.52, 0.52]} />
        <meshStandardMaterial color={color} />
      </mesh>
      <Text
        position={[0, 0, 0.02]}
        fontSize={0.14}
        color="#1f2b32"
        anchorX="center"
        anchorY="middle"
      >
        {label}
      </Text>
    </group>
  );
}

function MiniViewCube({ onCommand, viewPositionRef, mainCameraRef, mainControlsRef }) {
  const controlsRef = useRef(null);
  const { camera } = useThree();
  const draggingRef = useRef(false);

  useEffect(() => {
    camera.position.set(0, 0, 4);
    camera.lookAt(0, 0, 0);
    camera.updateProjectionMatrix();
  }, [camera]);

  useFrame(() => {
    if (draggingRef.current) {
      if (!mainCameraRef?.current) {
        return;
      }
      const mainCamera = mainCameraRef.current;
      const radius = mainCamera.position.length() || 1.0;
      const nextPosition = camera.position.clone().normalize().multiplyScalar(radius);
      mainCamera.position.copy(nextPosition);
      mainCamera.up.set(0, 0, 1);
      mainCamera.lookAt(0, 0, 0);
      mainCamera.updateProjectionMatrix();
      if (mainControlsRef?.current) {
        mainControlsRef.current.target.set(0, 0, 0);
        mainControlsRef.current.update();
      }
      return;
    }

    if (!viewPositionRef?.current) {
      return;
    }
    const nextPosition = viewPositionRef.current.clone().normalize().multiplyScalar(4);
    camera.position.copy(nextPosition);
    camera.lookAt(0, 0, 0);
    camera.updateProjectionMatrix();
    if (controlsRef.current) {
      controlsRef.current.target.set(0, 0, 0);
      controlsRef.current.update();
    }
  });

  return (
    <>
      <ambientLight intensity={1.25} />
      <directionalLight position={[3, 4, 5]} intensity={1.3} />
      <group>
        <mesh>
          <boxGeometry args={[0.24, 0.24, 0.24]} />
          <meshBasicMaterial color="#b4a590" wireframe transparent opacity={0.18} />
        </mesh>
        <MiniViewCubeFace
          position={[0, -0.28, 0]}
          rotation={[Math.PI / 2, 0, 0]}
          color="#f6f1e8"
          label="F"
          onClick={() => onCommand("front")}
        />
        <MiniViewCubeFace
          position={[0, 0.28, 0]}
          rotation={[-Math.PI / 2, 0, 0]}
          color="#e7ddd0"
          label="B"
          onClick={() => onCommand("back")}
        />
        <MiniViewCubeFace
          position={[0.28, 0, 0]}
          rotation={[0, Math.PI / 2, 0]}
          color="#cfe2de"
          label="R"
          onClick={() => onCommand("right")}
        />
        <MiniViewCubeFace
          position={[-0.28, 0, 0]}
          rotation={[0, -Math.PI / 2, 0]}
          color="#d9e7e4"
          label="L"
          onClick={() => onCommand("left")}
        />
        <MiniViewCubeFace
          position={[0, 0, 0.28]}
          color="#ece2d1"
          label="T"
          onClick={() => onCommand("top")}
        />
        <MiniViewCubeFace
          position={[0, 0, -0.28]}
          rotation={[0, Math.PI, 0]}
          color="#e3d5c2"
          label="D"
          onClick={() => onCommand("bottom")}
        />
      </group>
      <OrbitControls
        ref={controlsRef}
        enablePan={false}
        enableZoom={false}
        rotateSpeed={0.85}
        onStart={() => {
          draggingRef.current = true;
        }}
        onEnd={() => {
          draggingRef.current = false;
        }}
      />
    </>
  );
}

function CameraOrientationTracker({ cameraRef, viewPositionRef }) {
  const { camera } = useThree();

  useFrame(() => {
    if (cameraRef) {
      cameraRef.current = camera;
    }
    if (viewPositionRef?.current) {
      viewPositionRef.current.copy(camera.position);
    }
  });

  return null;
}

function ZUpScene() {
  const { camera, raycaster } = useThree();

  useEffect(() => {
    camera.up.set(0, 0, 1);
    camera.updateProjectionMatrix();
  }, [camera]);

  useEffect(() => {
    const previousThreshold = raycaster.params.Points.threshold;
    raycaster.params.Points.threshold = 0.03;
    return () => {
      raycaster.params.Points.threshold = previousThreshold;
    };
  }, [raycaster]);

  return null;
}

function CameraDockController({ controlsRef, cameraCommand }) {
  const { camera } = useThree();

  useEffect(() => {
    if (!cameraCommand?.type) {
      return;
    }

    const radius = 2.35;
    let nextPosition = null;

    switch (cameraCommand.type) {
      case "recenter":
      case "iso":
        nextPosition = new THREE.Vector3(1.6, -1.7, 1.2);
        break;
      case "front":
        nextPosition = new THREE.Vector3(0, -radius, 0.28);
        break;
      case "back":
        nextPosition = new THREE.Vector3(0, radius, 0.28);
        break;
      case "left":
        nextPosition = new THREE.Vector3(-radius, 0, 0.28);
        break;
      case "right":
        nextPosition = new THREE.Vector3(radius, 0, 0.28);
        break;
      case "top":
        nextPosition = new THREE.Vector3(0, 0, radius);
        break;
      case "bottom":
        nextPosition = new THREE.Vector3(0, 0, -radius);
        break;
      default:
        break;
    }

    if (!nextPosition) {
      return;
    }

    camera.position.copy(nextPosition);
    camera.up.set(0, 0, 1);
    camera.lookAt(0, 0, 0);
    camera.updateProjectionMatrix();

    if (controlsRef?.current) {
      controlsRef.current.target.set(0, 0, 0);
      controlsRef.current.update();
    }
  }, [camera, cameraCommand, controlsRef]);

  return null;
}

function toneForConnectionState(connectionState) {
  switch (connectionState) {
    case "idle":
      return "ok";
    case "scanning":
      return "warn";
    case "recovering_sensor":
      return "warn";
    case "stale":
      return "danger";
    default:
      return "neutral";
  }
}

function toneForRobotState(robotState) {
  switch (robotState) {
    case "idle":
      return "ok";
    case "scanning":
      return "warn";
    case "stopping":
    case "hard_stopping":
    case "releasing":
      return "danger";
    default:
      return "neutral";
  }
}

function toneForSensorStatus(sensorStatus) {
  if (!sensorStatus) {
    return "neutral";
  }
  if (sensorStatus === "READY") {
    return "ok";
  }
  if (sensorStatus === "SCANNING" || sensorStatus === "RECOVERING_SENSOR") {
    return "warn";
  }
  if (
    sensorStatus === "SENSOR_TIMEOUT" ||
    sensorStatus === "STALE_LINK" ||
    sensorStatus.includes("FAILED")
  ) {
    return "danger";
  }
  return "neutral";
}

function yawProgressPercent(yawDeg, scanDegrees) {
  if (!Number.isFinite(yawDeg)) {
    return 0;
  }
  const maxDegrees = Number.isFinite(scanDegrees) && scanDegrees > 0 ? scanDegrees : 360;
  return Math.max(0, Math.min(100, (Math.max(0, yawDeg) / maxDegrees) * 100));
}

function formatPointNumber(value, digits = 1) {
  return Number.isFinite(value) ? Number(value).toFixed(digits) : "-";
}

function formatClockSeconds(totalSeconds) {
  if (!Number.isFinite(totalSeconds) || totalSeconds < 0) {
    return "-:--";
  }
  const rounded = Math.max(0, Math.round(totalSeconds));
  const minutes = Math.floor(rounded / 60);
  const seconds = rounded % 60;
  return `${minutes}:${String(seconds).padStart(2, "0")}`;
}

export default function App() {
  const liveStatus = useStatusData();
  const fileInputRef = useRef(null);
  const scanDegreesInputRef = useRef(null);
  const orbitControlsRef = useRef(null);
  const mainCameraRef = useRef(null);
  const viewCubeCameraPositionRef = useRef(new THREE.Vector3(1.6, -1.7, 1.2));
  const [leftColumnOpen, setLeftColumnOpen] = useState(false);
  const [rightColumnOpen, setRightColumnOpen] = useState(false);
  const [requestedMode, setRequestedMode] = useState("raw");
  const [baselineHoldActive, setBaselineHoldActive] = useState(false);
  const [baselinePinned, setBaselinePinned] = useState(false);
  const [residualTolerance, setResidualTolerance] = useState(25);
  const [scanDegreesInput, setScanDegreesInput] = useState("20");
  const [importedBundle, setImportedBundle] = useState(null);
  const [importMessage, setImportMessage] = useState("");
  const [commandMessage, setCommandMessage] = useState("");
  const [alarm, setAlarm] = useState(null);
  const [confirmDialog, setConfirmDialog] = useState(null);
  const [helpOpen, setHelpOpen] = useState(false);
  const [exporting, setExporting] = useState(false);
  const [installingBaseline, setInstallingBaseline] = useState(false);
  const [selectedClusterId, setSelectedClusterId] = useState(null);
  const [hoveredSample, setHoveredSample] = useState(null);
  const [cameraCommand, setCameraCommand] = useState({ type: "iso", nonce: 0 });
  const [clusterRefreshNonce, setClusterRefreshNonce] = useState(0);
  const [clusterContextMenu, setClusterContextMenu] = useState(null);
  const [shootingCluster, setShootingCluster] = useState(false);
  const [autoClearDebris, setAutoClearDebris] = useState(true);
  const effectiveMode = baselineHoldActive || baselinePinned ? "baseline" : requestedMode;
  const residualPollingEnabled =
    !importedBundle &&
    Boolean(liveStatus?.baseline_exists) &&
    requestedMode === "residual";
  const livePayload = useLivePoints(
    effectiveMode,
    residualTolerance,
    clusterRefreshNonce,
    !importedBundle,
    500,
  );
  const residualLivePayload = useLivePoints(
    "residual",
    residualTolerance,
    clusterRefreshNonce,
    residualPollingEnabled,
    900,
  );
  const status = importedBundle?.status ?? liveStatus;
  const activePayload = importedBundle
    ? selectImportedPayload(importedBundle, requestedMode, baselineHoldActive)
    : livePayload;
  const residualSourcePayload = importedBundle
    ? (importedBundle.views.residual ?? null)
    : residualLivePayload;
  const points = activePayload.points ?? [];
  const samples = activePayload.samples ?? [];
  const clusters = Array.isArray(activePayload.clusters) ? activePayload.clusters : [];
  const fallbackStatusClusters = Array.isArray(status?.last_debris_clusters)
    ? status.last_debris_clusters
    : [];
  const leaderboardClusters = Array.isArray(residualSourcePayload?.clusters)
    ? residualSourcePayload.clusters
    : fallbackStatusClusters;
  const topClusters =
    Array.isArray(residualSourcePayload?.top_clusters) &&
    residualSourcePayload.top_clusters.length > 0
      ? residualSourcePayload.top_clusters
      : leaderboardClusters.slice(0, 3);
  const visiblePointCount = points.filter((point) => Number(point[4] ?? 0) < 0.5).length;
  const renderMode = activePayload.render_mode ?? "raw";
  const rawAvailable = importedBundle ? Boolean(importedBundle.views.raw) : true;
  const residualAvailable = importedBundle
    ? Boolean(importedBundle.views.residual)
    : Boolean(livePayload.baseline_available);
  const baselineAvailable = importedBundle
    ? Boolean(importedBundle.views.baseline)
    : Boolean(livePayload.baseline_available);
  const isResidualMode = renderMode === "residual";
  const isBaselineMode = renderMode === "baseline";
  const viewerTitle = isBaselineMode
    ? "3D Baseline Viewer"
    : isResidualMode
      ? "3D Residual Viewer"
      : "3D Point Cloud Viewer";
  const viewerSubtitle = isBaselineMode
    ? importedBundle
      ? "Imported baseline snapshot from a saved viewer bundle."
      : "Hold-to-view baseline snapshot from the FastAPI robot server."
    : isResidualMode
    ? importedBundle
      ? "Imported residual snapshot filtered against its saved baseline."
      : "Filtered live raw scan against the saved baseline. Only rays with current range smaller than baseline are shown."
    : importedBundle
      ? "Imported raw point-cloud snapshot from a saved viewer bundle."
      : "Live raw point-cloud view driven by the FastAPI robot server.";
  const pointsLabel = isBaselineMode ? "Baseline points" : isResidualMode ? "Foreground points" : "Points";
  const fileLabel = importedBundle
    ? "Bundle"
    : isBaselineMode || isResidualMode
      ? "Baseline"
      : "CSV";
  const fileValue = importedBundle
    ? importedBundle.imported_name
    : isBaselineMode || isResidualMode
      ? status?.baseline_path ?? "-"
      : status?.current_capture ?? status?.last_capture ?? "-";
  const selectedCluster = selectedClusterId
    ? leaderboardClusters.find((cluster) => cluster.cluster_id === selectedClusterId) ?? null
    : null;
  const hoveredCluster = hoveredSample?.cluster_id
    ? clusters.find((cluster) => cluster.cluster_id === hoveredSample.cluster_id)
      ?? leaderboardClusters.find((cluster) => cluster.cluster_id === hoveredSample.cluster_id)
      ?? null
    : null;
  const currentYawDeg = Number(status?.current_yaw_deg);
  const currentPitchDeg = Number(status?.current_pitch_deg);
  const configuredScanDegrees = Number(status?.configured_scan_degrees);
  const yawPercent = yawProgressPercent(currentYawDeg, configuredScanDegrees);
  const scanInProgress = Boolean(status?.scan_in_progress);
  const currentFrameId = status?.current_frame_id ?? activePayload.frame_id ?? null;
  const [scanStartedAtMs, setScanStartedAtMs] = useState(null);
  const [scanElapsedSeconds, setScanElapsedSeconds] = useState(0);
  const [smoothedEstimatedTotalSeconds, setSmoothedEstimatedTotalSeconds] = useState(0);
  const smoothedEstimateRef = useRef(null);
  const smoothedEstimateUpdatedAtRef = useRef(null);
  const selectedClusterPoints = selectedClusterId
    ? points.filter((point) => point[6] === selectedClusterId)
    : [];
  const selectedClusterSamples = selectedClusterId
    ? samples.filter((sample) => sample.cluster_id === selectedClusterId)
    : [];

  function raiseAlarm(nextAlarm) {
    if (!nextAlarm) {
      return;
    }
    setAlarm({
      id: Date.now(),
      tone: nextAlarm.tone ?? "neutral",
      title: nextAlarm.title ?? "Notice",
      detail: nextAlarm.detail ?? "",
      timeoutMs:
        typeof nextAlarm.timeoutMs === "number" ? nextAlarm.timeoutMs : 4400,
    });
  }

  function requestConfirm(options) {
    return new Promise((resolve) => {
      setConfirmDialog({
        ...options,
        resolve,
      });
    });
  }

  function resolveConfirm(result) {
    setConfirmDialog((currentDialog) => {
      if (currentDialog?.resolve) {
        currentDialog.resolve(result);
      }
      return null;
    });
  }

  function runCameraCommand(type) {
    setCameraCommand({ type, nonce: Date.now() });
  }

  function handleThreeClusterContextMenu(threeEvent, cluster) {
    threeEvent.nativeEvent.preventDefault();
    setClusterContextMenu({ x: threeEvent.clientX, y: threeEvent.clientY, cluster });
  }

  function handleLeaderboardClusterContextMenu(domEvent, cluster) {
    domEvent.preventDefault();
    setClusterContextMenu({ x: domEvent.clientX, y: domEvent.clientY, cluster });
  }

  async function handleShootCluster(cluster) {
    const centroid = Array.isArray(cluster.centroid_mm) && cluster.centroid_mm.length === 3
      ? cluster.centroid_mm
      : [cluster.bbox.center_x_mm, cluster.bbox.center_y_mm, cluster.bbox.center_z_mm];
    setClusterContextMenu(null);
    const confirmed = await requestConfirm({
      title: `Shoot laser at ${cluster.cluster_id}?`,
      detail:
        "The robot will move the laser toward this debris cluster. Confirm that the area is clear before continuing.",
      tone: "warn",
      confirmLabel: "Shoot laser",
      cancelLabel: "Cancel",
    });
    if (!confirmed) {
      setCommandMessage(`Laser shot cancelled for ${cluster.cluster_id}.`);
      return;
    }
    setShootingCluster(true);
    raiseAlarm({
      tone: "warn",
      title: `Aiming laser at ${cluster.cluster_id}`,
      detail: "Laser command sent to the robot. Waiting for the move to finish.",
      timeoutMs: 0,
    });
    try {
      await aimAtCluster(centroid);
      setCommandMessage(`Aimed laser at ${cluster.cluster_id}.`);
      raiseAlarm({
        title: `Laser aimed at ${cluster.cluster_id}`,
        detail: "The viewer sent the targeting command successfully.",
        timeoutMs: 5000,
      });
    } catch (error) {
      setCommandMessage(`Aim failed: ${error instanceof Error ? error.message : String(error)}`);
      raiseAlarm({
        tone: "danger",
        title: `Laser aim failed for ${cluster.cluster_id}`,
        detail: error instanceof Error ? error.message : String(error),
        timeoutMs: 7000,
      });
    } finally {
      setShootingCluster(false);
    }
  }

  useEffect(() => {
    if (!clusterContextMenu) return;
    function onKeyDown(e) {
      if (e.key === "Escape") setClusterContextMenu(null);
    }
    document.addEventListener("keydown", onKeyDown);
    return () => document.removeEventListener("keydown", onKeyDown);
  }, [clusterContextMenu]);

  function triggerResidualClustering() {
    if (importedBundle) {
      setCommandMessage("Imported bundles use their saved clustering snapshot.");
      return;
    }
    setBaselineHoldActive(false);
    setBaselinePinned(false);
    setRequestedMode("residual");
    setSelectedClusterId(null);
    setClusterRefreshNonce((value) => value + 1);
    setCommandMessage("Residual clustering refreshed.");
  }

  useEffect(() => {
    if (liveStatus?.auto_clear_debris != null) {
      setAutoClearDebris(Boolean(liveStatus.auto_clear_debris));
    }
  }, [liveStatus?.auto_clear_debris]);

  useEffect(() => {
    function releaseBaselineHold() {
      setBaselineHoldActive(false);
    }

    window.addEventListener("pointerup", releaseBaselineHold);
    return () => {
      window.removeEventListener("pointerup", releaseBaselineHold);
    };
  }, []);

  useEffect(() => {
    if (!alarm) {
      return undefined;
    }
    if (!Number.isFinite(alarm.timeoutMs) || alarm.timeoutMs <= 0) {
      return undefined;
    }
    const timer = window.setTimeout(() => setAlarm(null), alarm.timeoutMs);
    return () => window.clearTimeout(timer);
  }, [alarm]);

  useEffect(() => {
    if (!confirmDialog) {
      return undefined;
    }

    function handleKeyDown(event) {
      if (event.key === "Escape") {
        resolveConfirm(false);
      }
    }

    window.addEventListener("keydown", handleKeyDown);
    return () => window.removeEventListener("keydown", handleKeyDown);
  }, [confirmDialog]);

  useEffect(() => {
    if (!helpOpen) {
      return undefined;
    }

    function handleKeyDown(event) {
      if (event.key === "Escape") {
        setHelpOpen(false);
      }
    }

    window.addEventListener("keydown", handleKeyDown);
    return () => window.removeEventListener("keydown", handleKeyDown);
  }, [helpOpen]);

  useEffect(() => {
    if (selectedClusterId && !clusters.some((cluster) => cluster.cluster_id === selectedClusterId)) {
      setSelectedClusterId(null);
    }
  }, [clusters, selectedClusterId]);

  useEffect(() => {
    if (
      liveStatus?.configured_scan_degrees != null &&
      document.activeElement !== scanDegreesInputRef.current
    ) {
      setScanDegreesInput(String(Number(liveStatus.configured_scan_degrees)));
    }
  }, [liveStatus?.configured_scan_degrees]);

  useEffect(() => {
    if (scanInProgress) {
      setScanStartedAtMs((previousValue) => previousValue ?? Date.now());
      return;
    }
    setScanStartedAtMs(null);
    setScanElapsedSeconds(0);
    setSmoothedEstimatedTotalSeconds(0);
    smoothedEstimateRef.current = null;
    smoothedEstimateUpdatedAtRef.current = null;
  }, [scanInProgress, currentFrameId]);

  useEffect(() => {
    if (!scanInProgress || !scanStartedAtMs) {
      return undefined;
    }

    function updateElapsed() {
      setScanElapsedSeconds((Date.now() - scanStartedAtMs) / 1000);
    }

    updateElapsed();
    const timer = window.setInterval(updateElapsed, 250);
    return () => window.clearInterval(timer);
  }, [scanInProgress, scanStartedAtMs]);

  const progressFraction = Math.max(0.01, yawPercent / 100);
  useEffect(() => {
    if (!scanInProgress) {
      return;
    }

    const rawEstimatedTotalSeconds = scanElapsedSeconds / progressFraction;
    if (!Number.isFinite(rawEstimatedTotalSeconds) || rawEstimatedTotalSeconds <= 0) {
      return;
    }

    const nowMs = Date.now();
    const previousEstimate = smoothedEstimateRef.current;
    const previousUpdatedAtMs = smoothedEstimateUpdatedAtRef.current;

    let nextEstimate = rawEstimatedTotalSeconds;
    if (Number.isFinite(previousEstimate) && Number.isFinite(previousUpdatedAtMs)) {
      const deltaSeconds = Math.max(0.001, (nowMs - previousUpdatedAtMs) / 1000);
      const smoothingTimeConstantSeconds = 1.8;
      const alpha = 1 - Math.exp(-deltaSeconds / smoothingTimeConstantSeconds);
      nextEstimate = previousEstimate + alpha * (rawEstimatedTotalSeconds - previousEstimate);
    }

    smoothedEstimateRef.current = nextEstimate;
    smoothedEstimateUpdatedAtRef.current = nowMs;
    setSmoothedEstimatedTotalSeconds(nextEstimate);
  }, [scanElapsedSeconds, progressFraction, scanInProgress]);

  const estimatedTotalSeconds = scanInProgress
    ? (smoothedEstimatedTotalSeconds > 0 ? smoothedEstimatedTotalSeconds : scanElapsedSeconds / progressFraction)
    : 0;

  async function handleExport() {
    setExporting(true);
    setImportMessage("");
    try {
      const bundle = importedBundle ?? (() => null)();
      if (bundle) {
        downloadBundle(bundle, exportFilename());
        setImportMessage(`Exported ${bundle.imported_name}`);
        return;
      }

      const statusPayload = await fetchJson("/api/status");
      const rawPayload = sanitizePayload(
        await fetchJson(
          `/api/live-points?mode=raw&tolerance_mm=${encodeURIComponent(residualTolerance)}`,
        ),
        "raw",
      );

      let baselinePayload = null;
      let residualPayload = null;
      if (statusPayload.baseline_exists) {
        [baselinePayload, residualPayload] = await Promise.all([
          fetchJson(
            `/api/live-points?mode=baseline&tolerance_mm=${encodeURIComponent(residualTolerance)}`,
          ).then((payload) => sanitizePayload(payload, "baseline")),
          fetchJson(
            `/api/live-points?mode=residual&tolerance_mm=${encodeURIComponent(residualTolerance)}`,
          ).then((payload) => sanitizePayload(payload, "residual")),
        ]);
      }

      downloadBundle(
        {
          format: "ward-viewer-bundle",
          version: 1,
          exported_at: new Date().toISOString(),
          status: statusPayload,
          views: {
            raw: rawPayload,
            residual: residualPayload,
            baseline: baselinePayload,
          },
        },
        exportFilename(),
      );
      setImportMessage("Exported live scan bundle.");
    } catch (error) {
      setImportMessage(`Export failed: ${error.message}`);
    } finally {
      setExporting(false);
    }
  }

  function handleImportClick() {
    fileInputRef.current?.click();
  }

  async function handleImportChange(event) {
    const file = event.target.files?.[0];
    if (!file) {
      return;
    }

    try {
      const bundle = normalizeImportedBundle(
        JSON.parse(await file.text()),
        file.name,
      );
      setImportedBundle(bundle);
      setRequestedMode(chooseDefaultMode(bundle));
      setBaselineHoldActive(false);
      if (bundle.status?.residual_tolerance_mm != null) {
        const importedTolerance = Number(bundle.status.residual_tolerance_mm);
        if (Number.isFinite(importedTolerance) && importedTolerance >= 0) {
          setResidualTolerance(importedTolerance);
        }
      }
      setImportMessage(`Imported ${file.name}`);
    } catch (error) {
      setImportMessage(`Import failed: ${error.message}`);
    } finally {
      event.target.value = "";
    }
  }

  function returnToLive() {
    setImportedBundle(null);
    setImportMessage("Returned to live server data.");
  }

  async function installImportedBaseline() {
    if (!importedBundle) {
      setImportMessage("Import a bundle first.");
      return;
    }

    setInstallingBaseline(true);
    try {
      await installBaselineBundle(
        importedBundle,
        importedBundle.imported_name,
        importedBundle.views.baseline ? "baseline" : "raw",
      );
      setImportMessage(`Installed baseline on server from ${importedBundle.imported_name}`);
    } catch (error) {
      setImportMessage(`Install baseline failed: ${error.message}`);
    } finally {
      setInstallingBaseline(false);
    }
  }

  function resetRender() {
    setHoveredSample(null);
    setSelectedClusterId(null);
    setBaselineHoldActive(false);
    setBaselinePinned(false);
    runCameraCommand("recenter");

    if (importedBundle) {
      setImportedBundle((currentBundle) => {
        if (!currentBundle) {
          return currentBundle;
        }
        return normalizeImportedBundle(
          {
            ...currentBundle,
            views: {
              raw: currentBundle.views.raw,
              residual: currentBundle.views.residual,
              baseline: currentBundle.views.baseline,
            },
          },
          currentBundle.imported_name,
        );
      });
      setRequestedMode(importedBundle.views.residual ? "residual" : chooseDefaultMode(importedBundle));
      setCommandMessage(
        importedBundle.views.residual
          ? "Render reset and imported residual restored."
          : "Render reset for imported bundle.",
      );
      return;
    }

    setRequestedMode(baselineAvailable ? "residual" : "raw");
    setClusterRefreshNonce((value) => value + 1);
    setCommandMessage(
      baselineAvailable
        ? "Render reset and residual clustering refreshed."
        : "Render reset.",
    );
  }

  return (
    <div className="app-shell">
      <article className="viewer-stage">
        <Canvas camera={{ position: [1.6, -1.7, 1.2], fov: 45 }}>
          <ZUpScene />
          <CameraOrientationTracker cameraRef={mainCameraRef} viewPositionRef={viewCubeCameraPositionRef} />
          <CameraDockController controlsRef={orbitControlsRef} cameraCommand={cameraCommand} />
          <color attach="background" args={["#f2ece3"]} />
          <ambientLight intensity={1.15} />
          <directionalLight position={[3, 3, 4]} intensity={1.4} />
          <Grid
            args={[4, 4]}
            position={[0, 0, -0.35]}
            rotation={[Math.PI / 2, 0, 0]}
            cellSize={0.1}
            cellThickness={0.5}
            sectionSize={0.5}
            sectionThickness={1}
            cellColor="#c9beb0"
            sectionColor="#9d8f7d"
            fadeDistance={8}
            fadeStrength={1}
            infiniteGrid
          />
          <axesHelper args={[0.6]} />
          <SensorRig yawDeg={currentYawDeg} pitchDeg={currentPitchDeg} />
          <SurfaceMesh
            samples={samples}
            renderMode={renderMode}
            opacityOverride={selectedClusterId && isResidualMode ? 0.42 : null}
          />
          <PointCloud
            points={points}
            samples={samples}
            renderMode={renderMode}
            onHoverChange={setHoveredSample}
          />
          <HoveredPointMarker sample={hoveredSample} />
          {isResidualMode ? (
            <>
              <DebrisClusterBoxes
                clusters={clusters}
                selectedClusterId={selectedClusterId}
                onClusterContextMenu={handleThreeClusterContextMenu}
              />
              {selectedClusterId ? (
                <>
                  <SurfaceMesh
                    samples={selectedClusterSamples}
                    renderMode={renderMode}
                    opacityOverride={0.96}
                  />
                  <SelectedClusterPoints points={selectedClusterPoints} />
                </>
              ) : null}
            </>
          ) : null}
          <OrbitControls ref={orbitControlsRef} makeDefault />
        </Canvas>
      </article>

      {!hoveredSample ? (
        <div className="camera-dock" aria-label="Camera controls">
          <div className="view-cube" role="group" aria-label="View selector">
            <div className="view-cube__canvas-shell">
              <Canvas
                className="view-cube__canvas"
                orthographic
                camera={{ position: [0, 0, 4], zoom: 120 }}
                gl={{ alpha: true, antialias: true }}
                style={{ background: "transparent" }}
              >
                <MiniViewCube
                  onCommand={runCameraCommand}
                  viewPositionRef={viewCubeCameraPositionRef}
                  mainCameraRef={mainCameraRef}
                  mainControlsRef={orbitControlsRef}
                />
              </Canvas>
            </div>
          </div>
          <button
            type="button"
            className="camera-dock__home"
            onClick={() => runCameraCommand("recenter")}
            aria-label="Recenter camera"
          >
            Home
          </button>
        </div>
      ) : null}

      <button
        type="button"
        className={`side-toggle left ${leftColumnOpen ? "open" : ""}`}
        onClick={() => setLeftColumnOpen((value) => !value)}
        aria-label={leftColumnOpen ? "Hide controls" : "Show controls"}
      >
        {leftColumnOpen ? "×" : "◀"}
      </button>

      <button
        type="button"
        className={`side-toggle right ${rightColumnOpen ? "open" : ""}`}
        onClick={() => setRightColumnOpen((value) => !value)}
        aria-label={rightColumnOpen ? "Hide info" : "Show info"}
      >
        {rightColumnOpen ? "×" : "▶"}
      </button>

      <div className={`overlay-column left ${leftColumnOpen ? "open" : ""}`}>
        <details className="overlay-panel hero-panel" open>
          <summary>{viewerTitle}</summary>
          <div className="panel-body">
            <p className="subtle">{viewerSubtitle}</p>
            <div className="viewer-actions">
              <button type="button" onClick={resetRender}>
                Reset Render
              </button>
              <button type="button" onClick={handleExport} disabled={exporting}>
                {exporting ? "Saving..." : "Save Bundle"}
              </button>
              <button type="button" onClick={handleImportClick}>
                Import Bundle
              </button>
              <button
                type="button"
                onClick={installImportedBaseline}
                disabled={!importedBundle || installingBaseline}
              >
                {installingBaseline ? "Installing..." : "Use As Server Baseline"}
              </button>
              <button type="button" onClick={returnToLive} disabled={!importedBundle}>
                Return Live
              </button>
              <input
                ref={fileInputRef}
                type="file"
                accept="application/json,.json"
                className="hidden-input"
                onChange={handleImportChange}
              />
            </div>
            <label className="inline-number">
              Monitoring Tolerance
              <input
                type="number"
                min="0"
                step="5"
                value={residualTolerance}
                onChange={(event) => setResidualTolerance(Number(event.target.value) || 0)}
              />
            </label>
            <label className="command-field command-field--toggle">
              <input
                type="checkbox"
                checked={autoClearDebris}
                onChange={(event) => {
                  const enabled = event.target.checked;
                  setAutoClearDebris(enabled);
                  apiSetAutoClearDebris(enabled).catch(() => {});
                }}
              />
              <span>Auto-clear debris</span>
            </label>
            {importMessage ? <p className="subtle import-note">{importMessage}</p> : null}
          </div>
        </details>
      </div>

      <div className={`overlay-column right ${rightColumnOpen ? "open" : ""}`}>
        <div className="overlay-stack">
          <details className="overlay-panel info-panel" open>
            <summary>Frame</summary>
            <div className="panel-body">
              <div className="stat-row"><span>Frame ID</span><strong>{activePayload.frame_id ?? "-"}</strong></div>
              <div className="stat-row"><span>{pointsLabel}</span><strong>{visiblePointCount}</strong></div>
              <div className="stat-row"><span>Mode</span><strong>{renderMode}</strong></div>
              <div className="stat-row"><span>Requested view</span><strong>{effectiveMode}</strong></div>
              <div className="stat-row"><span>Monitoring tolerance</span><strong>{residualTolerance} mm</strong></div>
              <div className="stat-row"><span>Debris clusters</span><strong>{leaderboardClusters.length || "-"}</strong></div>
              <div className="stat-row"><span>Timeout</span><strong>{status?.sensor_timeout ? "Yes" : "No"}</strong></div>
              <div className="stat-row"><span>{fileLabel}</span><strong>{fileValue}</strong></div>
            </div>
          </details>

          <details className="overlay-panel info-panel" open>
            <summary>Top Debris</summary>
            <div className="panel-body">
              <div className="viewer-actions">
                <button
                  type="button"
                  onClick={triggerResidualClustering}
                  disabled={Boolean(importedBundle) || !residualAvailable}
                >
                  Cluster Residuals
                </button>
              </div>
              {topClusters.length > 0 ? (
                <div className="cluster-list">
                  <button
                    type="button"
                    className={`cluster-item ${selectedClusterId == null ? "active" : ""}`}
                    onClick={() => {
                      setRequestedMode("residual");
                      setBaselineHoldActive(false);
                      setSelectedClusterId(null);
                    }}
                  >
                    <strong>All debris</strong>
                    <div className="cluster-meta">
                      <span>{leaderboardClusters.length} clusters</span>
                      <span>Open residual</span>
                    </div>
                  </button>
                  {topClusters.map((cluster, index) => (
                    <button
                      key={cluster.cluster_id}
                      type="button"
                      className={`cluster-item ${selectedClusterId === cluster.cluster_id ? "active" : ""}`}
                      onClick={() => {
                        setRequestedMode("residual");
                        setBaselineHoldActive(false);
                        setSelectedClusterId(
                          selectedClusterId === cluster.cluster_id ? null : cluster.cluster_id,
                        );
                      }}
                      onContextMenu={(e) => handleLeaderboardClusterContextMenu(e, cluster)}
                    >
                      <strong>#{index + 1} · Score {Number(cluster.score).toFixed(1)}</strong>
                      <div className="cluster-meta">
                        <span>{cluster.point_count} pts</span>
                        <span>Occ {Number(cluster.max_occlusion_mm).toFixed(1)} mm</span>
                      </div>
                      <div className="cluster-meta">
                        <span>
                          {Number(cluster.bbox.size_x_mm).toFixed(0)} × {Number(cluster.bbox.size_y_mm).toFixed(0)} × {Number(cluster.bbox.size_z_mm).toFixed(0)} mm
                        </span>
                      </div>
                    </button>
                  ))}
                </div>
              ) : (
                <p className="event-copy">No debris clusters detected from the residual view.</p>
              )}
              {selectedCluster ? (
                <div className="cluster-summary">
                  <div className="stat-row"><span>Selected</span><strong>{selectedCluster.cluster_id}</strong></div>
                  <div className="stat-row"><span>Score</span><strong>{Number(selectedCluster.score).toFixed(1)}</strong></div>
                  <div className="stat-row"><span>Mean occlusion</span><strong>{Number(selectedCluster.mean_occlusion_mm).toFixed(1)} mm</strong></div>
                  <div className="stat-row"><span>Max occlusion</span><strong>{Number(selectedCluster.max_occlusion_mm).toFixed(1)} mm</strong></div>
                </div>
              ) : null}
            </div>
          </details>

          <details className="overlay-panel info-panel">
            <summary>Event</summary>
            <div className="panel-body">
              <p className="event-copy">
                {importedBundle
                  ? `Imported ${importedBundle.imported_name}${importedBundle.exported_at ? ` · exported ${importedBundle.exported_at}` : ""}`
                  : status?.last_event ?? "Waiting for server state..."}
              </p>
              {status?.last_error ? <p className="event-copy error">{status.last_error}</p> : null}
            </div>
          </details>
        </div>
      </div>

      {hoveredSample ? (
        <div className={`hover-info-panel ${scanInProgress ? "hover-info-panel--raised" : ""}`}>
          <div className="hover-info-panel__title">Point</div>
          <div className="hover-info-grid">
            <div><span>X</span><strong>{formatPointNumber(hoveredSample.x_mm)} mm</strong></div>
            <div><span>Y</span><strong>{formatPointNumber(hoveredSample.y_mm)} mm</strong></div>
            <div><span>Z</span><strong>{formatPointNumber(hoveredSample.z_mm)} mm</strong></div>
            <div><span>R</span><strong>{formatPointNumber(hoveredSample.distance_mm)} mm</strong></div>
            <div><span>Φ</span><strong>{formatPointNumber(hoveredSample.yaw_deg, 2)} deg</strong></div>
            <div><span>Θ</span><strong>{formatPointNumber(hoveredSample.pitch_deg, 2)} deg</strong></div>
            {hoveredCluster ? (
              <>
                <div className="hover-info-grid__wide">
                  <span>Debris</span>
                  <strong>{hoveredCluster.cluster_id}</strong>
                </div>
                <div><span>Score</span><strong>{formatPointNumber(hoveredCluster.score, 1)}</strong></div>
                <div><span>Occ Max</span><strong>{formatPointNumber(hoveredCluster.max_occlusion_mm, 1)} mm</strong></div>
              </>
            ) : null}
          </div>
        </div>
      ) : null}

      <div className="alert-stack">
        <AlertToast alert={alarm} onClose={() => setAlarm(null)} />
      </div>

      <ConfirmDialog dialog={confirmDialog} onResolve={resolveConfirm} />
      <HelpDialog open={helpOpen} onClose={() => setHelpOpen(false)} />

      <div className="bottom-shell">
        {scanInProgress ? (
          <div className="scan-progress-strip">
            <div className="scan-progress-strip__row">
              <div className="scan-progress-strip__track">
                <div className="scan-progress-strip__fill" style={{ width: `${yawPercent}%` }} />
              </div>
              <strong className="scan-progress-strip__time">
                <span className="scan-progress-strip__label">Scanning</span>
                <span className="scan-progress-strip__separator">·</span>
                <span>{formatClockSeconds(scanElapsedSeconds)}/{formatClockSeconds(estimatedTotalSeconds)}</span>
              </strong>
            </div>
          </div>
        ) : null}

        <div className="bottom-bar">
          <div className="bottom-bar__mode">
            <div className="mode-toggle persistent">
              <button
                type="button"
                className={effectiveMode === "raw" ? "active" : ""}
                onClick={() => {
                  setBaselineHoldActive(false);
                  setBaselinePinned(false);
                  setRequestedMode("raw");
                }}
                disabled={!rawAvailable}
              >
                Raw
              </button>
              <button
                type="button"
                className={effectiveMode === "residual" ? "active" : ""}
                onClick={() => {
                  setBaselineHoldActive(false);
                  setBaselinePinned(false);
                  setRequestedMode("residual");
                }}
                disabled={!residualAvailable}
              >
                Residual
              </button>
              <button
                type="button"
                className={effectiveMode === "baseline" ? "active" : ""}
                disabled={!baselineAvailable}
                onPointerDown={(event) => {
                  if (!baselineAvailable) {
                    return;
                  }
                  event.preventDefault();
                  if (baselinePinned) {
                    return;
                  }
                  setBaselineHoldActive(true);
                }}
                onPointerUp={() => {
                  if (!baselinePinned) {
                    setBaselineHoldActive(false);
                  }
                }}
                onPointerLeave={() => {
                  if (!baselinePinned) {
                    setBaselineHoldActive(false);
                  }
                }}
                onPointerCancel={() => {
                  if (!baselinePinned) {
                    setBaselineHoldActive(false);
                  }
                }}
                onDoubleClick={() => {
                  if (!baselineAvailable) {
                    return;
                  }
                  setBaselineHoldActive(false);
                  setBaselinePinned((value) => !value);
                }}
              >
                Baseline
              </button>
            </div>
          </div>
          <div className="bottom-bar__commands">
            <CommandBar
              status={liveStatus}
              scanDegrees={scanDegreesInput}
              setScanDegrees={setScanDegreesInput}
              autoClearDebris={autoClearDebris}
              scanDegreesInputRef={scanDegreesInputRef}
              onAlarm={raiseAlarm}
              onCommandMessage={setCommandMessage}
              onOpenHelp={() => setHelpOpen(true)}
              requestConfirm={requestConfirm}
            />
          </div>
          <div className="bottom-bar__robot">
            <div className="robot-strip">
              <div className="robot-strip__item">
                <span>Link</span>
                <span
                  className={`status-light status-light--${toneForConnectionState(status?.robot_connection_state)}`}
                  title={`Link: ${status?.robot_connected ? (status?.robot_connection_state ?? "connected") : "disconnected"}`}
                  aria-label={`Link ${status?.robot_connected ? (status?.robot_connection_state ?? "connected") : "disconnected"}`}
                />
              </div>
              <div className="robot-strip__item">
                <span>Robot</span>
                <span
                  className={`status-light status-light--${toneForRobotState(status?.robot_state)}`}
                  title={`Robot: ${status?.robot_state ?? "-"}`}
                  aria-label={`Robot ${status?.robot_state ?? "-"}`}
                />
              </div>
              <div className="robot-strip__item">
                <span>Sensor</span>
                <span
                  className={`status-light status-light--${toneForSensorStatus(status?.sensor_status)}`}
                  title={`Sensor: ${status?.sensor_status ?? "-"}`}
                  aria-label={`Sensor ${status?.sensor_status ?? "-"}`}
                />
              </div>
              <div className="robot-strip__item">
                <span>Yaw</span>
                <strong>{status?.current_yaw_deg != null ? `${status.current_yaw_deg.toFixed(2)} deg` : "-"}</strong>
              </div>
              <div className="robot-strip__item">
                <span>Pitch</span>
                <strong>{status?.current_pitch_deg != null ? `${status.current_pitch_deg.toFixed(2)} deg` : "-"}</strong>
              </div>
            </div>
          </div>
          {commandMessage ? <div className="command-message">{commandMessage}</div> : null}
        </div>
      </div>

      {clusterContextMenu ? (
        <>
          <div
            className="cluster-context-backdrop"
            onClick={() => setClusterContextMenu(null)}
          />
          <div
            className="cluster-context-menu"
            style={{ left: clusterContextMenu.x, top: clusterContextMenu.y }}
          >
            <div className="cluster-context-menu__header">
              <span className="cluster-context-menu__id">{clusterContextMenu.cluster.cluster_id}</span>
              <span>Score {Number(clusterContextMenu.cluster.score).toFixed(1)} · Occ {Number(clusterContextMenu.cluster.max_occlusion_mm).toFixed(1)} mm</span>
            </div>
            <button
              type="button"
              className="cluster-context-menu__shoot"
              onClick={() => handleShootCluster(clusterContextMenu.cluster)}
              disabled={shootingCluster || !status?.robot_connected}
            >
              {shootingCluster ? "Aiming…" : "Shoot laser"}
            </button>
          </div>
        </>
      ) : null}
    </div>
  );
}
