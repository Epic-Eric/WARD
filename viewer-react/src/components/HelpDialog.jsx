import { useMemo, useState } from "react";

const HELP_TABS = [
  {
    id: "overview",
    label: "Overview",
    eyebrow: "Start Here",
    title: "How the viewer is laid out",
    intro:
      "Use the bottom bar for the main robot actions and view switching. The left and right drawers hold deeper tools, scan details, and debris review.",
    highlights: [
      {
        heading: "Bottom bar",
        detail: "Start scans, monitor, stop, change Sweep degrees, switch Raw / Residual / Baseline, and open Help.",
      },
      {
        heading: "Left drawer",
        detail: "Reset the render, save or import bundles, and push an imported baseline back to the live server.",
      },
      {
        heading: "Right drawer",
        detail: "Review the current frame, trigger residual clustering, and inspect the top debris leaderboard.",
      },
    ],
  },
  {
    id: "views",
    label: "Views",
    eyebrow: "What You See",
    title: "Raw, Residual, and Baseline",
    intro:
      "Each view is for a different job. The buttons on the left side of the bottom bar swap between them instantly.",
    highlights: [
      {
        heading: "Raw",
        detail: "Shows the live scan itself. Incoming raw points animate outward from the sensor as the sweep progresses.",
      },
      {
        heading: "Residual",
        detail: "Shows only new foreground blockers that are closer than the saved baseline by more than the monitoring tolerance.",
      },
      {
        heading: "Baseline",
        detail: "Hold Baseline to peek at the saved reference. Double-click it to keep baseline view pinned until you switch away.",
      },
    ],
  },
  {
    id: "scan",
    label: "Scan",
    eyebrow: "Robot Control",
    title: "Running scans and monitoring",
    intro:
      "The viewer can drive the robot directly while the Python control server is active.",
    highlights: [
      {
        heading: "Start",
        detail: "Runs one scan using the current Sweep value. If there is no baseline yet, the app can offer to save the last scan as baseline when it stops.",
      },
      {
        heading: "Monitor",
        detail: "Repeats scans over time and compares them to the baseline. This stays unavailable until a baseline exists.",
      },
      {
        heading: "More",
        detail: "Opens extra actions like Save Last Scan as Baseline, Hard Stop, Release, Zero, and Delete Baseline.",
      },
    ],
  },
  {
    id: "debris",
    label: "Debris",
    eyebrow: "Detection",
    title: "Understanding debris clusters",
    intro:
      "Residuals are clustered into debris candidates. Each cluster gets a box, score, and leaderboard entry for quick review.",
    highlights: [
      {
        heading: "Boxes and labels",
        detail: "Residual clusters draw bounding boxes in the scene. When you zoom out, overlapping labels collapse into compact grouped badges.",
      },
      {
        heading: "Top Debris",
        detail: "The right drawer ranks the strongest clusters. Clicking one switches to residual view and highlights that cluster.",
      },
      {
        heading: "Hover details",
        detail: "Hover a point to see X, Y, Z, R, Φ, Θ, plus the debris cluster that point belongs to.",
      },
    ],
  },
  {
    id: "camera",
    label: "Camera",
    eyebrow: "Navigation",
    title: "Moving around the scene",
    intro:
      "The main scene behaves like a lightweight CAD viewer, and the mini cube mirrors the current camera orientation.",
    highlights: [
      {
        heading: "Orbit and zoom",
        detail: "Drag in the main scene to orbit. Use the mouse wheel or trackpad to zoom in and out.",
      },
      {
        heading: "Mini cube",
        detail: "Drag the cube to rotate the camera, click a face to snap to that side, or press Home to recenter.",
      },
      {
        heading: "Reset Render",
        detail: "Found in the left drawer. It clears selection, recenters the camera, and rebuilds the residual render state.",
      },
    ],
  },
  {
    id: "files",
    label: "Files",
    eyebrow: "Import / Export",
    title: "Saving and reopening scan bundles",
    intro:
      "Bundles let you save scans from the viewer and reload them later without needing the robot connected.",
    highlights: [
      {
        heading: "Save Bundle",
        detail: "Exports the currently available raw, residual, and baseline views into a single JSON file.",
      },
      {
        heading: "Import Bundle",
        detail: "Loads a saved bundle back into the viewer so you can inspect it offline with its saved clustering snapshot.",
      },
      {
        heading: "Use As Server Baseline",
        detail: "Pushes an imported baseline back to the live control server so future residual scans compare against it.",
      },
    ],
  },
];

export default function HelpDialog({ open, onClose }) {
  const [activeTab, setActiveTab] = useState(HELP_TABS[0].id);

  const currentTab = useMemo(
    () => HELP_TABS.find((tab) => tab.id === activeTab) ?? HELP_TABS[0],
    [activeTab],
  );

  if (!open) {
    return null;
  }

  return (
    <div className="help-dialog-backdrop" onClick={onClose}>
      <section
        className="help-dialog"
        onClick={(event) => event.stopPropagation()}
        role="dialog"
        aria-modal="true"
        aria-labelledby="help-dialog-title"
      >
        <div className="help-dialog__header">
          <div>
            <p className="help-dialog__eyebrow">Viewer Help</p>
            <h2 id="help-dialog-title">How to use the 3D scanner viewer</h2>
            <p className="help-dialog__intro">
              Quick guidance for scanning, monitoring, debris review, navigation, and saved bundles.
            </p>
          </div>
          <button
            type="button"
            className="help-dialog__close"
            onClick={onClose}
            aria-label="Close help"
          >
            ×
          </button>
        </div>

        <div className="help-dialog__body">
          <nav className="help-tabs" aria-label="Help sections">
            {HELP_TABS.map((tab) => (
              <button
                key={tab.id}
                type="button"
                className={tab.id === currentTab.id ? "active" : ""}
                onClick={() => setActiveTab(tab.id)}
              >
                {tab.label}
              </button>
            ))}
          </nav>

          <div className="help-panel">
            <p className="help-panel__eyebrow">{currentTab.eyebrow}</p>
            <h3>{currentTab.title}</h3>
            <p className="help-panel__summary">{currentTab.intro}</p>
            <div className="help-highlight-list">
              {currentTab.highlights.map((item) => (
                <article key={`${currentTab.id}-${item.heading}`} className="help-highlight">
                  <strong>{item.heading}</strong>
                  <span>{item.detail}</span>
                </article>
              ))}
            </div>
          </div>
        </div>
      </section>
    </div>
  );
}
