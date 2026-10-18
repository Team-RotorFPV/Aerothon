import { useEffect, useState } from "react";
import ReactECharts from "echarts-for-react/lib/core";
import * as echarts from "echarts/core";
import { LineChart } from "echarts/charts";
import {
  GridComponent,
  TooltipComponent,
  LegendComponent,
} from "echarts/components";
import { SVGRenderer } from "echarts/renderers";
import {
  graphs,
  historyCsv,
  seriesPoints,
  type GraphGroup,
  type HistoryRow,
  type TelemetryHistory,
} from "./telemetryHistory";

echarts.use([
  LineChart,
  GridComponent,
  TooltipComponent,
  LegendComponent,
  SVGRenderer,
]);
const groups: [GraphGroup, string][] = [
  ["flight", "Flight"],
  ["navigation", "Navigation"],
  ["perception", "Perception"],
  ["payload", "Payload"],
  ["sensors", "Sensors"],
];

export default function Graphs({
  history,
  theme,
  live,
}: {
  history: TelemetryHistory;
  theme: string;
  live: boolean;
}) {
  const [group, setGroup] = useState<GraphGroup>("flight");
  const [windowSeconds, setWindowSeconds] = useState(120);
  const [paused, setPaused] = useState<HistoryRow[] | null>(null);
  const [selectedTraces, setSelectedTraces] = useState<
    Record<string, Record<string, boolean>>
  >({});
  const [, redraw] = useState(0);
  useEffect(() => {
    const timer = window.setInterval(() => redraw((n) => n + 1), 500);
    return () => window.clearInterval(timer);
  }, []);
  const all = paused || history.rows;
  const end = all.length ? all[all.length - 1].elapsed : 0;
  const start = Math.max(0, end - windowSeconds);
  const rows = all.filter((r) => r.elapsed >= start);
  const dark = theme === "dark";
  const text = dark ? "#eeeeee" : "#242424";
  const muted = dark ? "#bcbcbc" : "#595959";
  const line = dark ? "#494a4b" : "#d4d4d0";
  const colors = dark
    ? ["#ededed", "#b7b7b7", "#999999"]
    : ["#242424", "#565656", "#767676"];
  function exportCsv() {
    const url = URL.createObjectURL(
      new Blob([historyCsv(rows)], { type: "text/csv;charset=utf-8" }),
    );
    const a = document.createElement("a");
    a.href = url;
    a.download = `aerothon-trends-${new Date().toISOString().replace(/[:.]/g, "-")}.csv`;
    a.click();
    window.setTimeout(() => URL.revokeObjectURL(url), 1000);
  }
  return (
    <section className="graphs" aria-label="Telemetry graphs">
      <div className="graph-controls">
        <label>
          Graph group
          <select
            value={group}
            onChange={(e) => setGroup(e.target.value as GraphGroup)}
          >
            {groups.map(([id, label]) => (
              <option value={id} key={id}>
                {label}
              </option>
            ))}
          </select>
        </label>
        <label>
          Time window
          <select
            value={windowSeconds}
            onChange={(e) => setWindowSeconds(Number(e.target.value))}
          >
            <option value={30}>30 seconds</option>
            <option value={120}>2 minutes</option>
            <option value={600}>10 minutes</option>
          </select>
        </label>
        <button
          aria-pressed={!!paused}
          onClick={() => setPaused(paused ? null : history.rows.slice())}
        >
          {paused ? "Resume live view" : "Pause view"}
        </button>
        <button onClick={exportCsv} disabled={!rows.length}>
          Export window CSV
        </button>
      </div>
      {group === "sensors" && (
        <div className="magnetic-reference">
          <button
            disabled={
              !!paused ||
              !history.canCaptureMagneticReference(performance.now())
            }
            onClick={() => {
              history.captureMagneticReference(performance.now());
              redraw((n) => n + 1);
            }}
          >
            Capture magnetic reference
          </button>
          <p className="view-note">
            {history.magneticReference
              ? `Reference ${history.magneticReference.strength.toFixed(2)} µT · frame ${history.magneticReference.frame}.`
              : "No magnetic reference. Capture one with fresh compass telemetry while the FCU reports disarmed."}{" "}
            This observes telemetry and sends no aircraft command.
          </p>
        </div>
      )}
      <p className="view-note" role="status">
        {paused
          ? "View paused; collection continues."
          : live
            ? "Live telemetry"
            : "Telemetry unavailable; showing retained history."}{" "}
        · Last 10 minutes retained in this browser, up to 5 snapshots/s. Reload
        clears history.
      </p>
      <p className="view-note">
        Horizontal axis: elapsed steady seconds since the first browser sample.
        Gaps mean missing, stale or paused-clock data. Hover for values; each
        legend toggles a trace.
      </p>
      <div className="graph-grid">
        {graphs
          .filter((g) => g.group === group)
          .map((g) => {
            const traces = g.traces.map((s, i) => ({
              name: s.label,
              type: "line" as const,
              showSymbol: false,
              connectNulls: false,
              animation: false,
              data: seriesPoints(rows, s.key),
              lineStyle: { width: 2, type: ["solid", "dashed", "dotted"][i] },
              itemStyle: { color: colors[i] },
            }));
            const hasData = rows.some((r) =>
              g.traces.some(
                (s) =>
                  r.values[s.key] !== null && r.values[s.key] !== undefined,
              ),
            );
            const latest = rows.length ? rows[rows.length - 1] : null;
            return (
              <figure key={g.title} className="trend-figure">
                <figcaption>
                  {g.title} <span>{g.unit}</span>
                </figcaption>
                <ReactECharts
                  echarts={echarts}
                  onEvents={{
                    legendselectchanged: (event: {
                      selected: Record<string, boolean>;
                    }) =>
                      setSelectedTraces((previous) => ({
                        ...previous,
                        [g.title]: event.selected,
                      })),
                  }}
                  lazyUpdate
                  style={{ height: 215 }}
                  opts={{ renderer: "svg" }}
                  option={{
                    animation: false,
                    color: colors,
                    grid: { left: 52, right: 16, top: 48, bottom: 28 },
                    legend: {
                      selected: selectedTraces[g.title] || {},
                      top: 2,
                      left: 0,
                      textStyle: { color: text, fontSize: 11 },
                      itemWidth: 22,
                      itemHeight: 2,
                    },
                    tooltip: {
                      trigger: "axis",
                      renderMode: "richText",
                      backgroundColor: dark ? "#252627" : "#ffffff",
                      borderColor: line,
                      textStyle: { color: text },
                      valueFormatter: (v: unknown) =>
                        typeof v === "number"
                          ? `${v.toFixed(2)} ${g.unit}`
                          : "Unavailable",
                    },
                    xAxis: {
                      type: "value",
                      minInterval: 1,
                      min: start,
                      max: Math.max(end, start + 1),
                      axisLabel: {
                        color: muted,
                        formatter: (v: number) => `${Math.round(v)}s`,
                      },
                      axisLine: { lineStyle: { color: line } },
                      splitLine: { show: false },
                    },
                    yAxis: {
                      type: "value",
                      min: g.title === "Battery remaining" ? 0 : undefined,
                      max: g.title === "Battery remaining" ? 100 : undefined,
                      scale: true,
                      axisLabel: { color: muted },
                      splitLine: { lineStyle: { color: line, type: "dotted" } },
                    },
                    series: traces,
                  }}
                />
                <p className="graph-reading">
                  {!hasData
                    ? "No measured data in this window."
                    : g.traces
                        .map((s) => {
                          const n = latest?.values[s.key];
                          return `${s.label}: ${typeof n === "number" ? n.toFixed(2) + " " + g.unit : "Unavailable"}`;
                        })
                        .join(" · ")}
                </p>
                {g.note && <p className="view-note">{g.note}</p>}
              </figure>
            );
          })}
      </div>
    </section>
  );
}
