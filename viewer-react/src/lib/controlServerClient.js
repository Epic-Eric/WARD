async function requestJson(path, options = {}) {
  const response = await fetch(path, options);
  let payload = null;

  try {
    payload = await response.json();
  } catch {
    payload = null;
  }

  if (!response.ok) {
    const detail =
      payload && typeof payload.detail === "string"
        ? payload.detail
        : `Request failed: ${response.status}`;
    throw new Error(detail);
  }

  return payload;
}

function jsonRequest(path, method = "POST", body = null) {
  return requestJson(path, {
    method,
    headers: body ? { "Content-Type": "application/json" } : undefined,
    body: body ? JSON.stringify(body) : undefined,
  });
}

export function startScan(scanDegrees) {
  return jsonRequest("/api/robot/start-scan", "POST", {
    scan_degrees: scanDegrees,
  });
}

export function stopScan() {
  return jsonRequest("/api/robot/stop-scan");
}

export function hardStop() {
  return jsonRequest("/api/robot/hard-stop");
}

export function releaseMotors() {
  return jsonRequest("/api/robot/release-motors");
}

export function zeroTurret() {
  return jsonRequest("/api/robot/zero-turret");
}

export function saveBaseline() {
  return jsonRequest("/api/baseline/save", "POST");
}

export function deleteBaseline() {
  return jsonRequest("/api/baseline", "DELETE");
}

export function startMonitoring(scanDegrees, autoClearDebris = true) {
  return jsonRequest("/api/monitoring/start", "POST", {
    scan_degrees: scanDegrees,
    auto_clear_debris: autoClearDebris,
  });
}

export function setAutoClearDebris(enabled) {
  return jsonRequest("/api/monitoring/set-auto-clear", "POST", { enabled });
}

export function stopMonitoring() {
  return jsonRequest("/api/monitoring/stop");
}

export function aimAtCluster(centroidMm) {
  return jsonRequest("/api/robot/aim-at-cluster", "POST", { centroid_mm: centroidMm });
}

export function installBaselineBundle(bundle, sourceName, sourceView = "baseline") {
  return jsonRequest("/api/baseline/import", "POST", {
    bundle,
    source_name: sourceName,
    source_view: sourceView,
  });
}
