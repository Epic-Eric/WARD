import {
  deleteBaseline,
  hardStop,
  releaseMotors,
  saveBaseline,
  startMonitoring,
  startScan,
  stopMonitoring,
  stopScan,
  zeroTurret,
} from "../lib/controlServerClient";
import { useEffect, useRef, useState } from "react";

function formatCommandError(error) {
  if (error instanceof Error) {
    return error.message;
  }
  return String(error);
}

export default function CommandBar({
  status,
  scanDegrees,
  setScanDegrees,
  autoClearDebris,
  onAlarm,
  onCommandMessage,
  requestConfirm,
  scanDegreesInputRef,
}) {
  const [moreOpen, setMoreOpen] = useState(false);
  const [baselineAfterScan, setBaselineAfterScan] = useState(false);
  const prevClearingSeqRef = useRef(status?.clearing_debris_sequence ?? 0);
  const robotConnected = Boolean(status?.robot_connected);
  const scanInProgress = Boolean(status?.scan_in_progress);
  const previousScanInProgressRef = useRef(scanInProgress);
  const monitoringActive = Boolean(status?.monitoring_active);
  const pendingOperation = status?.pending_operation != null;
  const baselineExists = Boolean(status?.baseline_exists);
  const latestCaptureExists = Boolean(status?.last_scan_result?.capture_path || status?.last_capture);
  const controlsDisabled = !robotConnected;
  const scanDegreesValue = Number(scanDegrees);
  const scanDegreesValid =
    Number.isFinite(scanDegreesValue) &&
    scanDegreesValue > 0 &&
    scanDegreesValue <= 360;

  useEffect(() => {
    const scanJustFinished = previousScanInProgressRef.current && !scanInProgress;
    previousScanInProgressRef.current = scanInProgress;

    if (!scanJustFinished || !baselineAfterScan) {
      return;
    }

    const hasCapture = Boolean(status?.last_scan_result?.capture_path || status?.last_capture);
    if (!hasCapture) {
      setBaselineAfterScan(false);
      onCommandMessage?.("Baseline capture skipped because no scan capture was available.");
      return;
    }

    setBaselineAfterScan(false);
    run("Save scan as baseline", saveBaseline);
  }, [baselineAfterScan, onCommandMessage, scanInProgress, status?.last_capture, status?.last_scan_result?.capture_path]);

  useEffect(() => {
    const seq = status?.clearing_debris_sequence ?? 0;
    if (seq > prevClearingSeqRef.current) {
      prevClearingSeqRef.current = seq;
      const count = (status?.last_debris_clusters ?? []).length;
      onAlarm?.({
        title: `Clearing ${count} debris cluster${count !== 1 ? "s" : ""}`,
        detail: "Robot is autonomously moving to each debris centroid.",
        tone: "warn",
      });
    }
  }, [status?.clearing_debris_sequence, status?.last_debris_clusters, onAlarm]);

  async function run(label, action) {
    try {
      await action();
      setMoreOpen(false);
      onCommandMessage?.(`${label} sent.`);
    } catch (error) {
      onCommandMessage?.(`${label} failed: ${formatCommandError(error)}`);
    }
  }

  function handleStop() {
    if (monitoringActive) {
      return run("Stop monitoring", stopMonitoring);
    }
    return run("Stop scan", stopScan);
  }

  function handleMonitor() {
    if (!baselineExists) {
      onAlarm?.({
        title: "Baseline Needed",
        detail: "Press Start, then choose to save that completed scan as the baseline before monitoring.",
        tone: "warn",
      });
      return;
    }
    return run("Start monitoring", () => startMonitoring(scanDegreesValue, autoClearDebris));
  }

  async function handleStart() {
    let shouldSaveAsBaseline = false;
    if (!baselineExists) {
      shouldSaveAsBaseline = await requestConfirm?.({
        title: "No baseline saved yet.",
        detail: "Start this scan and save the last captured result as the baseline when it stops?",
        confirmLabel: "Start And Save",
        cancelLabel: "Start Only",
        tone: "accent",
      });
    }

    try {
      await startScan(scanDegreesValue);
      setBaselineAfterScan(shouldSaveAsBaseline);
      onCommandMessage?.(
        shouldSaveAsBaseline
          ? "Start scan sent. Baseline will be saved after completion."
          : "Start scan sent.",
      );
    } catch (error) {
      setBaselineAfterScan(false);
      onCommandMessage?.(`Start scan failed: ${formatCommandError(error)}`);
    }
  }

  function handleSaveBaseline() {
    const execute = async () => {
      if (baselineExists) {
        const confirmed = await requestConfirm?.({
          title: "Replace current baseline?",
          detail: "This will overwrite the existing baseline with the most recent captured scan.",
          confirmLabel: "Replace Baseline",
          cancelLabel: "Keep Current",
          tone: "danger",
        });
        if (!confirmed) {
          return;
        }
      }
      return run("Save scan as baseline", saveBaseline);
    };
    return execute();
  }

  function handleDeleteBaseline() {
    const execute = async () => {
      const confirmed = await requestConfirm?.({
        title: "Delete baseline?",
        detail: "Residual monitoring and residual view will be unavailable until a new baseline is saved.",
        confirmLabel: "Delete Baseline",
        cancelLabel: "Cancel",
        tone: "danger",
      });
      if (!confirmed) {
        return;
      }
      return run("Delete baseline", deleteBaseline);
    };
    return execute();
  }

  return (
    <div className="command-bar">
      <div className="command-group command-group--meta">
        <label className="command-field">
          <span>Sweep</span>
          <input
            ref={scanDegreesInputRef}
            type="number"
            min="1"
            max="360"
            step="1"
            value={scanDegrees}
            onChange={(event) => setScanDegrees(event.target.value)}
          />
        </label>
      </div>

      <div className="command-group">
        <button
          type="button"
          onClick={handleStart}
          disabled={controlsDisabled || !scanDegreesValid || scanInProgress || monitoringActive || pendingOperation}
        >
          Start
        </button>
        <button
          type="button"
          onClick={handleMonitor}
          className={!baselineExists ? "soft-disabled" : ""}
          disabled={controlsDisabled || !scanDegreesValid || scanInProgress || monitoringActive || pendingOperation}
        >
          Monitor
        </button>
        <button
          type="button"
          onClick={handleStop}
          disabled={controlsDisabled || (!scanInProgress && !monitoringActive)}
        >
          Stop
        </button>
        <button
          type="button"
          className={`more-button ${moreOpen ? "open" : ""}`}
          onClick={() => setMoreOpen((value) => !value)}
        >
          ⋯
        </button>
        {moreOpen ? (
          <div className="more-menu">
            <button
              type="button"
              onClick={handleSaveBaseline}
              disabled={scanInProgress || monitoringActive || pendingOperation || !latestCaptureExists}
            >
              Save Scan as Baseline
            </button>
            <button
              type="button"
              className="danger"
              onClick={() => run("Hard stop", hardStop)}
              disabled={controlsDisabled}
            >
              Hard Stop
            </button>
            <button
              type="button"
              onClick={() => run("Release motors", releaseMotors)}
              disabled={controlsDisabled}
            >
              Release
            </button>
            <button
              type="button"
              onClick={() => run("Zero turret", zeroTurret)}
              disabled={controlsDisabled || scanInProgress || monitoringActive || pendingOperation}
            >
              Zero
            </button>
            <button
              type="button"
              onClick={handleDeleteBaseline}
              disabled={pendingOperation || monitoringActive || !baselineExists}
            >
              Delete Baseline
            </button>
          </div>
        ) : null}
      </div>
    </div>
  );
}
