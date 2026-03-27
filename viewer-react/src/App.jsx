import { Canvas, useThree } from "@react-three/fiber";
import { OrbitControls, Grid, Line } from "@react-three/drei";
import { useEffect, useMemo, useRef, useState } from "react";
import * as THREE from "three";

function emptyPayload(mode = "raw") {
  return {
    frame_id: null,
    points: [],
    samples: [],
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

function useStatusData() {
  const [status, setStatus] = useState(null);

  useEffect(() => {
    let cancelled = false;

    async function load() {
      try {
        const response = await fetch("/api/status");
        const payload = await response.json();
        if (!cancelled) {
          setStatus(payload);
        }
      } catch {
        if (!cancelled) {
          setStatus(null);
        }
      }
    }

    load();
    const timer = window.setInterval(load, 1000);
    return () => {
      cancelled = true;
      window.clearInterval(timer);
    };
  }, []);

  return status;
}

function useLivePoints(mode, residualTolerance) {
  const [payload, setPayload] = useState(emptyPayload(mode));

  useEffect(() => {
    let cancelled = false;

    async function load() {
      try {
        const response = await fetch(
          `/api/live-points?mode=${encodeURIComponent(mode)}&tolerance_mm=${encodeURIComponent(residualTolerance)}`,
        );
        const nextPayload = await response.json();
        if (!cancelled) {
          setPayload(nextPayload);
        }
      } catch {
        if (!cancelled) {
          setPayload(emptyPayload(mode));
        }
      }
    }

    load();
    const timer = window.setInterval(load, 500);
    return () => {
      cancelled = true;
      window.clearInterval(timer);
    };
  }, [mode, residualTolerance]);

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

function PointCloud({ points, renderMode }) {
  const geometry = useMemo(() => {
    const drawablePoints = points.filter((point) => Number(point[4] ?? 0) < 0.5);
    const positionArray = new Float32Array(drawablePoints.length * 3);
    const colorArray = new Float32Array(drawablePoints.length * 3);

    drawablePoints.forEach((point, index) => {
      const [xMm, yMm, zMm] = point;
      positionArray[index * 3 + 0] = xMm / 1000;
      positionArray[index * 3 + 1] = yMm / 1000;
      positionArray[index * 3 + 2] = zMm / 1000;

      const color = colorForDistance(colorMetricForPoint(point, renderMode), renderMode);
      colorArray[index * 3 + 0] = color.r;
      colorArray[index * 3 + 1] = color.g;
      colorArray[index * 3 + 2] = color.b;
    });

    const nextGeometry = new THREE.BufferGeometry();
    nextGeometry.setAttribute("position", new THREE.BufferAttribute(positionArray, 3));
    nextGeometry.setAttribute("color", new THREE.BufferAttribute(colorArray, 3));
    return nextGeometry;
  }, [points, renderMode]);

  if (geometry.getAttribute("position")?.count === 0) {
    return null;
  }

  return (
    <points geometry={geometry}>
      <pointsMaterial size={0.022} vertexColors sizeAttenuation />
    </points>
  );
}

function SurfaceMesh({ samples, renderMode }) {
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
        opacity={renderMode === "residual" ? 0.9 : renderMode === "baseline" ? 0.72 : 0.76}
      />
    </mesh>
  );
}

function SensorRig() {
  return (
    <group>
      <mesh position={[0, 0, 0]}>
        <sphereGeometry args={[0.035, 24, 24]} />
        <meshStandardMaterial color="#d95d39" />
      </mesh>
      <Line points={[[0, 0, 0], [0.5, 0, 0]]} color="#20343b" lineWidth={2} />
    </group>
  );
}

function ZUpScene() {
  const { camera } = useThree();

  useEffect(() => {
    camera.up.set(0, 0, 1);
    camera.updateProjectionMatrix();
  }, [camera]);

  return null;
}

export default function App() {
  const liveStatus = useStatusData();
  const fileInputRef = useRef(null);
  const [requestedMode, setRequestedMode] = useState("raw");
  const [baselineHoldActive, setBaselineHoldActive] = useState(false);
  const [residualTolerance, setResidualTolerance] = useState(25);
  const [importedBundle, setImportedBundle] = useState(null);
  const [importMessage, setImportMessage] = useState("");
  const [exporting, setExporting] = useState(false);
  const effectiveMode = baselineHoldActive ? "baseline" : requestedMode;
  const livePayload = useLivePoints(effectiveMode, residualTolerance);
  const status = importedBundle?.status ?? liveStatus;
  const activePayload = importedBundle
    ? selectImportedPayload(importedBundle, requestedMode, baselineHoldActive)
    : livePayload;
  const points = activePayload.points ?? [];
  const samples = activePayload.samples ?? [];
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

  useEffect(() => {
    function releaseBaselineHold() {
      setBaselineHoldActive(false);
    }

    window.addEventListener("pointerup", releaseBaselineHold);
    return () => {
      window.removeEventListener("pointerup", releaseBaselineHold);
    };
  }, []);

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

  return (
    <div className="app-shell">
      <section className="hero-card">
        <div>
          <p className="eyebrow">WARD</p>
          <h1>{viewerTitle}</h1>
          <p className="subtle">{viewerSubtitle}</p>
          <div className="mode-toggle">
            <button
              type="button"
              className={requestedMode === "raw" ? "active" : ""}
              onClick={() => setRequestedMode("raw")}
              disabled={!rawAvailable}
            >
              Raw
            </button>
            <button
              type="button"
              className={requestedMode === "residual" ? "active" : ""}
              onClick={() => setRequestedMode("residual")}
              disabled={!residualAvailable}
            >
              Residual
            </button>
            <button
              type="button"
              className={isBaselineMode ? "active" : ""}
              disabled={!baselineAvailable}
              onPointerDown={(event) => {
                if (!baselineAvailable) {
                  return;
                }
                event.preventDefault();
                setBaselineHoldActive(true);
              }}
              onPointerUp={() => setBaselineHoldActive(false)}
              onPointerLeave={() => setBaselineHoldActive(false)}
              onPointerCancel={() => setBaselineHoldActive(false)}
            >
              Baseline
            </button>
          </div>
          <div className="viewer-actions">
            <button type="button" onClick={handleExport} disabled={exporting}>
              {exporting ? "Saving..." : "Save Bundle"}
            </button>
            <button type="button" onClick={handleImportClick}>
              Import Bundle
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
          {importMessage ? <p className="subtle import-note">{importMessage}</p> : null}
        </div>
        <div className={`connection-pill ${status?.robot_connected ? "online" : "offline"}`}>
          {importedBundle
            ? "Imported bundle"
            : status?.robot_connected
              ? `Robot ${status.robot_state}`
              : "Robot disconnected"}
        </div>
      </section>

      <section className="layout">
        <article className="viewer-card">
          <Canvas camera={{ position: [1.6, -1.7, 1.2], fov: 45 }}>
            <ZUpScene />
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
            <SensorRig />
            <SurfaceMesh samples={samples} renderMode={renderMode} />
            <PointCloud points={points} renderMode={renderMode} />
            <OrbitControls makeDefault />
          </Canvas>
        </article>

        <aside className="sidebar">
          <section className="info-card">
            <h2>Robot</h2>
            <div className="stat-row"><span>Address</span><strong>{status?.robot_address ?? "-"}</strong></div>
            <div className="stat-row"><span>Yaw</span><strong>{status?.current_yaw_deg != null ? `${status.current_yaw_deg.toFixed(2)} deg` : "-"}</strong></div>
            <div className="stat-row"><span>Pitch</span><strong>{status?.current_pitch_deg != null ? `${status.current_pitch_deg.toFixed(2)} deg` : "-"}</strong></div>
            <div className="stat-row"><span>Sensor</span><strong>{status?.sensor_status ?? "-"}</strong></div>
          </section>

          <section className="info-card">
            <h2>Frame</h2>
            <div className="stat-row"><span>Frame ID</span><strong>{activePayload.frame_id ?? "-"}</strong></div>
            <div className="stat-row"><span>{pointsLabel}</span><strong>{visiblePointCount}</strong></div>
            <div className="stat-row"><span>Mode</span><strong>{renderMode}</strong></div>
            <div className="stat-row"><span>Requested view</span><strong>{effectiveMode}</strong></div>
            <div className="stat-row"><span>Monitoring tolerance</span><strong>{residualTolerance} mm</strong></div>
            <div className="stat-row"><span>Timeout</span><strong>{status?.sensor_timeout ? "Yes" : "No"}</strong></div>
            <div className="stat-row"><span>{fileLabel}</span><strong>{fileValue}</strong></div>
          </section>

          <section className="info-card">
            <h2>Event</h2>
            <p className="event-copy">
              {importedBundle
                ? `Imported ${importedBundle.imported_name}${importedBundle.exported_at ? ` · exported ${importedBundle.exported_at}` : ""}`
                : status?.last_event ?? "Waiting for server state..."}
            </p>
            {status?.last_error ? <p className="event-copy error">{status.last_error}</p> : null}
          </section>
        </aside>
      </section>
    </div>
  );
}
