import { useEffect, useMemo, useState } from "react";
import type { DimensionField, Dimensions, Telemetry } from "./types";
import schema from "./arenaSchema.json";
import { validateDraft } from "./arenaForm";

// The organisers give the arena in feet and inches; the aircraft works in
// metres. Each field keeps its own unit so a value can be typed exactly as
// it was announced ("10 ft", "3 in") and is converted once, here.
const UNITS: Record<string, number> = {
  m: 1,
  cm: 0.01,
  ft: 0.3048,
  in: 0.0254,
};
const GROUP_TITLES: Record<string, string> = {
  corridor: "Corridor (hall)",
  hall: "Hall / arena",
  banner: "Banner",
  qr: "QR codes",
  red_zone: "Red zone",
  payload: "Payload",
  obstacle: "Exit-corridor obstacles",
};

type Props = {
  S: Telemetry | null;
  connected: boolean;
  send: (cmd: string, args?: any, danger?: boolean) => void;
  onClose: () => void;
};

export default function ArenaDimensions({
  S,
  connected,
  send,
  onClose,
}: Props) {
  const fields: DimensionField[] = S?.dimension_fields ?? schema.fields;
  const presets = S?.dimension_presets ?? {
    builtin: schema.presets as Record<string, Dimensions>,
    saved: {} as Record<string, Dimensions>,
  };
  const status = S?.arena_dimensions ?? null;
  const canApply = connected && S?.mission?.armed === false;

  // The form: metres per field, plus the chosen unit per field.
  const [values, setValues] = useState<Dimensions>({});
  const [units, setUnits] = useState<Record<string, string>>({});
  const [presetName, setPresetName] = useState("");
  const [chosen, setChosen] = useState("");

  // Start from what the mission last applied, else the defaults.
  useEffect(() => {
    if (Object.keys(values).length || !fields.length) return;
    const start: Dimensions = {};
    for (const f of fields) (start[f.group] ??= {})[f.key] = f.default;
    start.banner = { ...start.banner, placement: "suspended" };
    start.corridor = { ...start.corridor, return_side: "right" };
    let draft: Dimensions | null = null;
    try {
      draft = JSON.parse(localStorage.getItem("gcs-arena-draft-v1") || "null");
    } catch {
      /* Ignore damaged local drafts. */
    }
    setValues(
      merge(
        start,
        draft && !validateDraft(draft, fields).length
          ? draft
          : (status?.applied ?? null),
      ),
    );
  }, [fields.length]);
  useEffect(() => {
    if (Object.keys(values).length)
      localStorage.setItem("gcs-arena-draft-v1", JSON.stringify(values));
  }, [values]);
  const errors = validateDraft(values, fields);
  // Escape closes the dialog; the draft is kept.
  useEffect(() => {
    const key = (e: KeyboardEvent) => e.key === "Escape" && onClose();
    window.addEventListener("keydown", key);
    return () => window.removeEventListener("keydown", key);
  }, [onClose]);

  const groups = useMemo(() => {
    const out: Record<string, DimensionField[]> = {};
    for (const f of fields) (out[f.group] ??= []).push(f);
    return out;
  }, [fields]);

  const id = (f: DimensionField) => `${f.group}.${f.key}`;
  const isLength = (f: DimensionField) => f.key.endsWith("_m");
  const unitOf = (f: DimensionField) =>
    isLength(f) ? (units[id(f)] ?? "m") : "";
  const shown = (f: DimensionField) => {
    const stored = values[f.group]?.[f.key] ?? f.default;
    if (stored === "") return "";
    const v = Number(stored);
    const u = unitOf(f);
    return u ? +(v / UNITS[u]).toFixed(4) : v;
  };
  const setField = (f: DimensionField, typed: string) => {
    const n = Number(typed);
    const u = unitOf(f);
    setValues((v) => ({
      ...v,
      [f.group]: {
        ...v[f.group],
        [f.key]:
          Number.isFinite(n) && typed !== "" ? (u ? n * UNITS[u] : n) : typed,
      },
    }));
  };
  const setChoice = (group: string, key: string, value: string) =>
    setValues((v) => ({ ...v, [group]: { ...v[group], [key]: value } }));

  const load = (name: string) => {
    setChosen(name);
    const dims = presets.builtin[name] ?? presets.saved[name];
    if (dims) {
      setValues(merge(values, dims));
      setPresetName(presets.saved[name] ? name : "");
    }
  };

  const appliedOk = status?.ok;
  const verdict = !status
    ? "launch values (none supplied)"
    : appliedOk
      ? "APPLIED by the mission"
      : status.reason
        ? `NOT applied · ${status.reason}`
        : "sent · awaiting the mission";

  return (
    <div className="dims-overlay" role="dialog" aria-label="Arena dimensions">
      <div className="card dims">
        <div className="hd">
          Arena dimensions
          <span className={`tag ${appliedOk ? "ok" : ""}`}>{verdict}</span>
          <button className="dims-close" onClick={onClose} aria-label="Close">
            ✕
          </button>
        </div>

        <div className="dims-presets">
          <select
            aria-label="Preset"
            value={chosen}
            onChange={(e) => load(e.target.value)}
          >
            <option value="" disabled>
              Load preset…
            </option>
            <optgroup label="Built-in">
              {Object.keys(presets.builtin).map((n) => (
                <option key={n} value={n}>
                  {n}
                </option>
              ))}
            </optgroup>
            {Object.keys(presets.saved).length > 0 && (
              <optgroup label="Saved">
                {Object.keys(presets.saved).map((n) => (
                  <option key={n} value={n}>
                    {n}
                  </option>
                ))}
              </optgroup>
            )}
          </select>
          <input
            aria-label="Preset name"
            placeholder="preset name"
            value={presetName}
            onChange={(e) => setPresetName(e.target.value)}
          />
          <button
            disabled={!canApply || !presetName.trim() || errors.length > 0}
            onClick={() =>
              send("save_dimension_preset", {
                name: presetName.trim(),
                dimensions: values,
              })
            }
          >
            SAVE PRESET
          </button>
          <button
            disabled={!canApply || !presets.saved[chosen]}
            onClick={() =>
              send("delete_dimension_preset", { name: chosen }, true)
            }
          >
            DELETE
          </button>
        </div>

        <p className="view-note">
          {connected
            ? "Edit the draft, then apply and wait for aircraft confirmation."
            : "Offline draft. Values are configuration defaults, not measured arena dimensions. Connect a disarmed aircraft to apply."}
        </p>
        {errors.length > 0 && <p role="alert">{errors.join(" · ")}</p>}
        <div className="dims-grid">
          {Object.entries(groups).map(([group, list]) => (
            <fieldset key={group}>
              <legend>{GROUP_TITLES[group] ?? group}</legend>
              {list.map((f) => (
                <label
                  key={id(f)}
                  className="dims-row"
                  title={
                    f.used_by
                      ? `Used for: ${f.used_by}`
                      : "Recorded only: no algorithm reads it yet"
                  }
                >
                  <span className="dims-label">
                    {f.label}
                    {!f.used_by && <em className="dims-rec"> rec.</em>}
                  </span>
                  <input
                    type="number"
                    step="any"
                    value={shown(f)}
                    onChange={(e) => setField(f, e.target.value)}
                  />
                  {isLength(f) ? (
                    <select
                      aria-label={`${f.label} unit`}
                      value={unitOf(f)}
                      onChange={(e) =>
                        setUnits((u) => ({ ...u, [id(f)]: e.target.value }))
                      }
                    >
                      {Object.keys(UNITS).map((u) => (
                        <option key={u}>{u}</option>
                      ))}
                    </select>
                  ) : (
                    <span className="dims-unit">
                      {f.key.endsWith("_frac") ? "× width" : "kg"}
                    </span>
                  )}
                </label>
              ))}
              {group === "banner" && (
                <label className="dims-row">
                  <span className="dims-label">Placement</span>
                  <select
                    value={String(values.banner?.placement ?? "suspended")}
                    onChange={(e) =>
                      setChoice("banner", "placement", e.target.value)
                    }
                  >
                    <option value="suspended">suspended</option>
                    <option value="ground">ground</option>
                    <option value="both">both</option>
                  </select>
                </label>
              )}
              {group === "corridor" && (
                <label className="dims-row">
                  <span className="dims-label">Return lane side</span>
                  <select
                    value={String(values.corridor?.return_side ?? "right")}
                    onChange={(e) =>
                      setChoice("corridor", "return_side", e.target.value)
                    }
                  >
                    <option value="right">right</option>
                    <option value="left">left</option>
                  </select>
                </label>
              )}
            </fieldset>
          ))}
        </div>

        <div className="dims-actions">
          <span className="dims-note">
            Applied on the ground only: the mission rebuilds with these while
            disarmed. Fields marked <em>rec.</em> are recorded for the operator;
            no algorithm reads them yet.
          </span>
          <button
            className="primary"
            disabled={!canApply || errors.length > 0}
            title={
              !canApply ? "Connect a disarmed aircraft to apply dimensions" : ""
            }
            onClick={() => send("set_arena_dimensions", { dimensions: values })}
          >
            APPLY TO AIRCRAFT
          </button>
        </div>
      </div>
    </div>
  );
}

function merge(base: Dimensions, over: Dimensions | null): Dimensions {
  if (!over) return base;
  const out: Dimensions = { ...base };
  for (const [g, vals] of Object.entries(over))
    out[g] = { ...(out[g] ?? {}), ...vals };
  return out;
}
