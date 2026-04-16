import { useEffect, useState } from "react";
import init, {
  classify_separation,
  type StateVector,
  type ProximityConfig,
  type MissionPhase,
} from "rpo-wasm";

// Hardcoded from examples/classify.json — temporary smoke test.
const chief: StateVector = {
  epoch: "2024-01-01T00:00:00 UTC",
  position_eci_km: [5876.261, 3392.661, 0.0],
  velocity_eci_km_s: [-2.380512, 4.123167, 6.006917],
};

const deputy: StateVector = {
  epoch: "2024-01-01T00:00:00 UTC",
  position_eci_km: [5199.839421, 4281.648523, 1398.070066],
  velocity_eci_km_s: [-3.993103, 2.970313, 5.764540],
};

const config: ProximityConfig = { roe_threshold: 0.005 };

type Status =
  | { kind: "loading" }
  | { kind: "ok"; phase: MissionPhase }
  | { kind: "err"; message: string };

export default function App() {
  const [status, setStatus] = useState<Status>({ kind: "loading" });

  useEffect(() => {
    init()
      .then(() => {
        try {
          const phase = classify_separation(chief, deputy, config);
          setStatus({ kind: "ok", phase });
        } catch (e) {
          setStatus({ kind: "err", message: String(e) });
        }
      })
      .catch((e) => setStatus({ kind: "err", message: String(e) }));
  }, []);

  return (
    <main style={{ fontFamily: "system-ui", padding: "2rem" }}>
      <h1>rpo-app</h1>
      <p>WASM smoke test: <code>classify_separation</code></p>
      {status.kind === "loading" && <p>loading…</p>}
      {status.kind === "err" && (
        <pre style={{ color: "crimson" }}>Error: {status.message}</pre>
      )}
      {status.kind === "ok" && (
        <pre style={{ background: "#f5f5f5", padding: "1rem", overflow: "auto" }}>
          {JSON.stringify(status.phase, null, 2)}
        </pre>
      )}
    </main>
  );
}
