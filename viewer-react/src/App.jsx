import { Canvas } from "@react-three/fiber";
import { OrbitControls, Grid, Line } from "@react-three/drei";
import { useEffect, useMemo, useState } from "react";
import * as THREE from "three";

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

function useLivePoints() {
  const [payload, setPayload] = useState({ frame_id: null, points: [] });

  useEffect(() => {
    let cancelled = false;

    async function load() {
      try {
        const response = await fetch("/api/live-points");
        const nextPayload = await response.json();
        if (!cancelled) {
          setPayload(nextPayload);
        }
      } catch {
        if (!cancelled) {
          setPayload({ frame_id: null, points: [] });
        }
      }
    }

    load();
    const timer = window.setInterval(load, 500);
    return () => {
      cancelled = true;
      window.clearInterval(timer);
    };
  }, []);

  return payload;
}

function PointCloud({ points }) {
  const geometry = useMemo(() => {
    const positionArray = new Float32Array(points.length * 3);
    const colorArray = new Float32Array(points.length * 3);
    const color = new THREE.Color();

    points.forEach((point, index) => {
      const [xMm, yMm, zMm, distanceMm] = point;
      positionArray[index * 3 + 0] = xMm / 1000;
      positionArray[index * 3 + 1] = yMm / 1000;
      positionArray[index * 3 + 2] = zMm / 1000;

      const t = Math.min(1, Math.max(0, distanceMm / 1500));
      color.setRGB(0.1 + 0.7 * t, 0.75 - 0.35 * t, 0.3 + 0.4 * (1 - t));
      colorArray[index * 3 + 0] = color.r;
      colorArray[index * 3 + 1] = color.g;
      colorArray[index * 3 + 2] = color.b;
    });

    const nextGeometry = new THREE.BufferGeometry();
    nextGeometry.setAttribute("position", new THREE.BufferAttribute(positionArray, 3));
    nextGeometry.setAttribute("color", new THREE.BufferAttribute(colorArray, 3));
    return nextGeometry;
  }, [points]);

  if (points.length === 0) {
    return null;
  }

  return (
    <points geometry={geometry}>
      <pointsMaterial size={0.022} vertexColors sizeAttenuation />
    </points>
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

export default function App() {
  const status = useStatusData();
  const livePayload = useLivePoints();
  const points = livePayload.points ?? [];

  return (
    <div className="app-shell">
      <section className="hero-card">
        <div>
          <p className="eyebrow">WARD</p>
          <h1>3D Point Cloud Viewer</h1>
          <p className="subtle">
            Live Three.js view driven by the FastAPI robot server.
          </p>
        </div>
        <div className={`connection-pill ${status?.robot_connected ? "online" : "offline"}`}>
          {status?.robot_connected ? `Robot ${status.robot_state}` : "Robot disconnected"}
        </div>
      </section>

      <section className="layout">
        <article className="viewer-card">
          <Canvas camera={{ position: [1.6, -1.7, 1.2], fov: 45 }}>
            <color attach="background" args={["#f2ece3"]} />
            <ambientLight intensity={1.15} />
            <directionalLight position={[3, 3, 4]} intensity={1.4} />
            <Grid
              args={[4, 4]}
              position={[0, 0, -0.35]}
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
            <PointCloud points={points} />
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
            <div className="stat-row"><span>Frame ID</span><strong>{livePayload.frame_id ?? "-"}</strong></div>
            <div className="stat-row"><span>Points</span><strong>{points.length}</strong></div>
            <div className="stat-row"><span>Timeout</span><strong>{status?.sensor_timeout ? "Yes" : "No"}</strong></div>
            <div className="stat-row"><span>CSV</span><strong>{status?.current_capture ?? status?.last_capture ?? "-"}</strong></div>
          </section>

          <section className="info-card">
            <h2>Event</h2>
            <p className="event-copy">{status?.last_event ?? "Waiting for server state..."}</p>
            {status?.last_error ? <p className="event-copy error">{status.last_error}</p> : null}
          </section>
        </aside>
      </section>
    </div>
  );
}
