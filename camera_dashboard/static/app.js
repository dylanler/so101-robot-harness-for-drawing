const grid = document.getElementById("grid");
const tiles = new Map();

function $(id) {
  return document.getElementById(id);
}

function fmt(value, fallback = "—") {
  if (value === null || value === undefined || value === "") return fallback;
  return String(value);
}

function usb(stream) {
  return stream.usb_ids || "—";
}

function healthDot(health) {
  if (health === "live") return "live";
  if (health === "stale") return "stale";
  if (health === "error") return "error";
  return "idle";
}

function clockTick() {
  $("clock").textContent = new Date().toLocaleTimeString();
}

function snapshotUrl(streamId) {
  return `/snapshot/${encodeURIComponent(streamId)}?t=${Date.now()}`;
}

function ensureTile(stream) {
  let tile = tiles.get(stream.id);
  if (tile) return tile;
  const el = document.createElement("article");
  el.className = "tile";
  el.dataset.id = stream.id;
  el.innerHTML = `
    <div class="tile-head">
      <div>
        <h2></h2>
        <p class="role"></p>
      </div>
      <div class="badge"></div>
    </div>
    <div class="stage">
      <img alt="" />
      <div class="overlay"><div class="hud fps"></div><div class="hud res"></div></div>
      <div class="error-banner"></div>
    </div>
    <div class="depth-readouts" hidden>
      <div><span>Min mm</span><b class="dmin">—</b></div>
      <div><span>Mean mm</span><b class="dmean">—</b></div>
      <div><span>Max mm</span><b class="dmax">—</b></div>
    </div>
    <div class="tile-meta"></div>
  `;
  const img = el.querySelector("img");
  img.alt = stream.name;
  img.src = snapshotUrl(stream.id);
  grid.appendChild(el);
  tile = { el, img };
  tiles.set(stream.id, tile);
  return tile;
}

function pruneTiles(ids) {
  for (const [id, tile] of tiles) {
    if (!ids.has(id)) {
      tile.el.remove();
      tiles.delete(id);
    }
  }
}

function renderMeta(stream) {
  const rows = [
    ["Device id", stream.device_id || stream.unique_id],
    ["Resolution", stream.width && stream.height ? `${stream.width}×${stream.height}` : "—"],
    ["FPS", stream.fps ? stream.fps.toFixed(1) : "—"],
    ["Backend", stream.backend],
    ["USB IDs", usb(stream)],
    ["Kind", stream.kind === "depth" ? "DEPTH" : "RGB"],
    ["Health", stream.health],
    ["Last frame", stream.last_frame_age_s != null ? `${stream.last_frame_age_s.toFixed(2)}s ago` : "none"],
  ];
  if (stream.usb_speed) rows.push(["USB speed", stream.usb_speed]);
  if (stream.serial) rows.push(["Serial", stream.serial]);
  if (stream.intrinsics) {
    const i = stream.intrinsics;
    rows.push(["Intrinsics", `fx ${i.fx?.toFixed(1)} fy ${i.fy?.toFixed(1)} ppx ${i.ppx?.toFixed(1)} ppy ${i.ppy?.toFixed(1)}`]);
  }
  return rows
    .map(([label, value]) => `<div><span>${label}</span><b>${fmt(value)}</b></div>`)
    .join("");
}

function updateTile(stream) {
  const label = `${stream.name || ""} ${stream.raw_name || ""}`.toLowerCase();
  if (label.includes("facetime")) return;
  const tile = ensureTile(stream);
  const { el } = tile;
  el.classList.toggle("is-depth", stream.kind === "depth");
  el.querySelector("h2").textContent = stream.name;
  el.querySelector(".role").textContent = stream.role || stream.raw_name || "";
  const badge = el.querySelector(".badge");
  badge.textContent = stream.kind === "depth" ? "DEPTH" : "RGB";
  badge.className = `badge ${stream.kind === "depth" ? "depth" : "rgb"}`;
  el.querySelector(".fps").textContent = `${stream.health.toUpperCase()} · ${stream.fps ? stream.fps.toFixed(1) : "0.0"} FPS`;
  el.querySelector(".res").textContent = stream.width ? `${stream.width}×${stream.height}` : "NO FRAME";
  el.querySelector(".tile-meta").innerHTML = renderMeta(stream);
  const banner = el.querySelector(".error-banner");
  if (stream.error && stream.health !== "live") {
    banner.textContent = stream.error;
    banner.classList.add("show");
  } else {
    banner.classList.remove("show");
  }
  const readouts = el.querySelector(".depth-readouts");
  if (stream.kind === "depth") {
    readouts.hidden = false;
    el.querySelector(".dmin").textContent = stream.min_depth_mm != null ? stream.min_depth_mm.toFixed(0) : "—";
    el.querySelector(".dmean").textContent = stream.mean_depth_mm != null ? stream.mean_depth_mm.toFixed(0) : "—";
    el.querySelector(".dmax").textContent = stream.max_depth_mm != null ? stream.max_depth_mm.toFixed(0) : "—";
  }
}

function refreshFrames() {
  for (const [id, tile] of tiles) {
    if (!tile.img.complete) continue;
    tile.img.src = snapshotUrl(id);
  }
}

async function poll() {
  try {
    const [sysRes, streamRes] = await Promise.all([fetch("/api/system", { cache: "no-store" }), fetch("/api/streams", { cache: "no-store" })]);
    const sys = await sysRes.json();
    const payload = await streamRes.json();
    $("url").textContent = sys.url || "http://127.0.0.1:8090";
    $("live-count").textContent = `${sys.live_count || 0} / ${sys.stream_count || 0}`;
    $("live-dot").className = `dot ${sys.live_count ? "live" : "idle"}`;
    const depth = (payload.streams || []).find((s) => s.kind === "depth");
    $("depth-state").textContent = depth?.health === "live" ? "LIVE" : depth?.error ? "NO SDK" : "WAIT";
    $("depth-dot").className = `dot ${healthDot(depth?.health)}`;
    $("arm-state").textContent = sys.so101_present ? "SERIAL UP" : "MISSING";
    $("arm-dot").className = `dot ${sys.so101_present ? "live" : "error"}`;
    $("meta-strip").textContent = [
      `uptime ${sys.uptime_s}s`,
      `pyrealsense2 ${sys.pyrealsense2 ? "yes" : "no"}`,
      `sdk devices ${sys.realsense_sdk_devices ?? "n/a"}`,
      `arm ${sys.so101_serial}`,
      `${(sys.profiler_cameras || []).length} macOS cameras`,
    ].join("   ·   ");
    const visible = (payload.streams || []).filter(
      (s) => !`${s.name || ""} ${s.raw_name || ""}`.toLowerCase().includes("facetime")
    );
    pruneTiles(new Set(visible.map((s) => s.id)));
    visible.forEach(updateTile);
  } catch (err) {
    $("live-count").textContent = "offline";
    $("live-dot").className = "dot error";
  }
}

clockTick();
setInterval(clockTick, 1000);
poll();
setInterval(poll, 750);
setInterval(refreshFrames, 120);
