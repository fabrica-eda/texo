//! Self-contained HTML/SVG renderer for physical implementation checkpoints.

use std::collections::BTreeMap;
use std::error::Error;
use std::fs::File;
use std::io::{self, BufReader, BufWriter, Write};

use serde_json::{Value, json};

const HTML: &str = r##"<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Texo physical design</title>
<style>
:root{color-scheme:dark;--bg:#080b11;--panel:#0e131d;--line:#253044;--text:#e8edf7;--muted:#8994a8;--accent:#6ee7ff;--danger:#ff627d}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:13px/1.45 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;overflow:hidden}
#app{height:100vh;display:grid;grid-template-rows:auto 1fr;grid-template-columns:1fr 320px}.toolbar{grid-column:1/-1;display:flex;align-items:center;gap:12px;min-height:52px;padding:8px 14px;background:#0c111a;border-bottom:1px solid var(--line);box-shadow:0 5px 24px #0008;z-index:3}.brand{font-weight:800;letter-spacing:.1em;color:var(--accent)}
button,input{font:inherit;color:var(--text);background:#151c28;border:1px solid #303c51;border-radius:5px;padding:6px 8px}button{cursor:pointer}button:hover{border-color:var(--accent)}input[type=search]{width:min(32vw,360px)}input[type=number]{width:75px}.check{display:flex;align-items:center;gap:5px;color:var(--muted);white-space:nowrap}.check input{accent-color:var(--accent)}
#stage{position:relative;min-width:0;min-height:0;background:radial-gradient(circle at 50% 30%,#111827,#070a10 70%)}svg{display:block;width:100%;height:100%;touch-action:none;cursor:grab}svg.dragging{cursor:grabbing}.grid-line{stroke:#253044;stroke-width:.45;vector-effect:non-scaling-stroke}.route{fill:none;stroke-linecap:round;stroke-linejoin:round;stroke-width:1.15;opacity:.18;vector-effect:non-scaling-stroke;cursor:pointer}.route:hover{opacity:.72;stroke-width:2}.route.selected{opacity:1!important;stroke:#fff!important;stroke-width:3.2!important;filter:drop-shadow(0 0 3px #6ee7ff)}.route.violating{opacity:.48}.route.filtered,.fixed-route.filtered{display:none}.fixed-route{fill:none;stroke:#fff;stroke-width:.65;stroke-dasharray:1.5 2;opacity:.28;vector-effect:non-scaling-stroke;pointer-events:none}.cell{stroke:#08101a;stroke-width:.65;vector-effect:non-scaling-stroke;cursor:pointer}.cell:hover,.cell.selected{stroke:#fff;stroke-width:2;filter:drop-shadow(0 0 3px #fff)}.cell.filtered{display:none}
.side{min-height:0;background:var(--panel);border-left:1px solid var(--line);display:grid;grid-template-rows:auto auto 1fr;overflow:hidden}.summary,.detail{padding:14px;border-bottom:1px solid var(--line)}.summary h1{font:700 16px/1.3 inherit;margin:0 0 8px;overflow-wrap:anywhere}.muted{color:var(--muted)}.stats{display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-top:10px}.stat{background:#151c28;padding:7px;border-radius:4px}.stat b{display:block;font-size:15px;color:#fff}.detail{min-height:116px;overflow-wrap:anywhere}.detail h2{font:700 13px/1.3 inherit;margin:0 0 8px;color:var(--accent)}.detail dl{display:grid;grid-template-columns:auto 1fr;margin:0;gap:3px 9px}.detail dt{color:var(--muted)}.detail dd{margin:0;text-align:right;overflow-wrap:anywhere}
#matches{overflow:auto;padding:8px}.match{width:100%;display:grid;grid-template-columns:52px 1fr auto;gap:7px;text-align:left;border:0;background:transparent;padding:7px}.match:hover{background:#182131}.match .type{color:var(--muted)}.match .name{overflow:hidden;text-overflow:ellipsis}.bad{color:var(--danger)}
#tooltip{position:absolute;display:none;pointer-events:none;z-index:5;max-width:460px;padding:7px 9px;border:1px solid #46566f;border-radius:5px;background:#0a0e16ee;box-shadow:0 5px 25px #000b;white-space:pre-line;overflow-wrap:anywhere}.legend{position:absolute;left:12px;bottom:12px;display:grid;gap:5px;padding:8px 10px;background:#0a0e16dd;border:1px solid var(--line);border-radius:5px;color:var(--muted);pointer-events:none}.legend-row{display:flex;align-items:center;flex-wrap:wrap;gap:0}.legend-label{width:54px;color:var(--text);font-weight:700;font-size:11px;letter-spacing:.06em}.key{display:inline-block;width:9px;height:9px;margin:0 4px 0 9px;border-radius:2px}.legend-label+.key{margin-left:0}.route-key{display:inline-block;width:25px;height:3px;margin:0 5px 0 11px;border-radius:2px;background:#fff}.legend-label+.route-key{margin-left:0}.net-key{background:linear-gradient(90deg,#67e8f9,#a78bfa,#fbbf24)}.bad-key{background:#ff627d}.selected-key{background:#fff;box-shadow:0 0 4px #6ee7ff}.fixed-key{height:0;border-top:2px dashed #fff;background:none}
@media(max-width:800px){#app{grid-template-columns:1fr;grid-template-rows:auto 1fr 190px}.toolbar{gap:7px;overflow-x:auto}.side{border-left:0;border-top:1px solid var(--line);grid-template-columns:220px 1fr 1fr;grid-template-rows:1fr}.summary,.detail{border-bottom:0;border-right:1px solid var(--line);overflow:auto}.legend{display:none}}
</style>
</head>
<body>
<div id="app">
  <div class="toolbar">
    <span class="brand">TEXO</span>
    <input id="search" type="search" placeholder="net / cell / BEL  (press /)">
    <button id="fit">Fit</button>
    <label class="check"><input id="routes-toggle" type="checkbox" checked>routes</label>
    <label class="check"><input id="cells-toggle" type="checkbox" checked>cells</label>
    <label class="check"><input id="grid-toggle" type="checkbox" checked>grid</label>
    <label class="check"><input id="critical-toggle" type="checkbox">timing ≤</label>
    <input id="threshold" type="number" value="100" step="25" title="Slack threshold in ps">
    <span class="muted">ps</span>
  </div>
  <main id="stage">
    <svg id="canvas" aria-label="FPGA placement and routes"><g id="grid"></g><g id="routes"></g><g id="fixed"></g><g id="cells"></g></svg>
    <div id="tooltip"></div>
    <div class="legend">
      <div class="legend-row"><span class="legend-label">CELLS</span><span class="key" style="background:#a78bfa"></span>LUT <span class="key" style="background:#67e8f9"></span>FF <span class="key" style="background:#fbbf24"></span>carry <span class="key" style="background:#4ade80"></span>IO <span class="key" style="background:#3b82f6"></span>clock <span class="key" style="background:#fb923c"></span>BRAM <span class="key" style="background:#94a3b8"></span>constant</div>
      <div class="legend-row"><span class="legend-label">ROUTES</span><span class="route-key net-key"></span>net identity <span class="route-key bad-key"></span>violation (&lt;0 ps) <span class="route-key selected-key"></span>selected <span class="route-key fixed-key"></span>fixed PIP</div>
    </div>
  </main>
  <aside class="side">
    <section class="summary"><h1 id="design"></h1><div id="target" class="muted"></div><div class="stats" id="stats"></div></section>
    <section class="detail" id="detail"><span class="muted">Click a cell or route.</span></section>
    <section id="matches"></section>
  </aside>
</div>
<script>
"use strict";
const DATA=__TEXO_DATA__;
const NS="http://www.w3.org/2000/svg",U=14,M=10;
const svg=document.querySelector("#canvas"),gridLayer=document.querySelector("#grid"),routeLayer=document.querySelector("#routes"),fixedLayer=document.querySelector("#fixed"),cellLayer=document.querySelector("#cells"),tip=document.querySelector("#tooltip"),detail=document.querySelector("#detail"),matches=document.querySelector("#matches");
const full={x:-M,y:-M,w:(DATA.extent.x+1)*U+2*M,h:(DATA.extent.y+1)*U+2*M};let view={...full},drag=null,selected=null;
const el=(name,attrs={})=>{const node=document.createElementNS(NS,name);for(const[k,v]of Object.entries(attrs))node.setAttribute(k,v);return node};
const esc=s=>String(s??"").replace(/[&<>"']/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"})[c]);
const slack=v=>v===null?"—":`${v} ps`;
const colorFor=name=>{let h=2166136261;for(const c of name){h^=c.charCodeAt(0);h=Math.imul(h,16777619)}return `hsl(${Math.abs(h)%360} 78% 63%)`};
const kindColor=k=>({lut4:"#a78bfa",lut:"#a78bfa",flip_flop:"#67e8f9",carry_slice:"#fbbf24",port:"#4ade80",global_clock:"#3b82f6",block_ram:"#fb923c",constant:"#94a3b8"}[k]||"#fb7185");
function setView(){svg.setAttribute("viewBox",`${view.x} ${view.y} ${view.w} ${view.h}`)}
function segmentPath(segments,onlyFixed=false){let d="";for(const s of segments){if(onlyFixed&&!s[4])continue;d+=`M${s[0]*U+U/2} ${s[1]*U+U/2}L${s[2]*U+U/2} ${s[3]*U+U/2}` }return d}
function labelSlack(item){const values=[item.setup_slack_ps,item.hold_slack_ps].filter(v=>v!==null);return values.length?Math.min(...values):null}
function showTip(text,event){tip.textContent=text;tip.style.display="block";const box=document.querySelector("#stage").getBoundingClientRect();tip.style.left=`${Math.min(event.clientX-box.left+12,box.width-tip.offsetWidth-8)}px`;tip.style.top=`${Math.min(event.clientY-box.top+12,box.height-tip.offsetHeight-8)}px`}
function hideTip(){tip.style.display="none"}
function clearSelection(){if(selected?.node)selected.node.classList.remove("selected");selected=null;detail.innerHTML='<span class="muted">Click a cell or route.</span>'}
function selectItem(type,item,node){clearSelection();selected={type,item,node};node?.classList.add("selected");if(type==="route"){detail.innerHTML=`<h2>ROUTE</h2><dl><dt>net</dt><dd>${esc(item.name)}</dd><dt>net id</dt><dd>${item.id}</dd><dt>PIPs</dt><dd>${item.pip_count} (${item.segments.length} visible)</dd><dt>setup</dt><dd class="${item.setup_slack_ps<0?'bad':''}">${slack(item.setup_slack_ps)}</dd><dt>hold</dt><dd class="${item.hold_slack_ps<0?'bad':''}">${slack(item.hold_slack_ps)}</dd></dl>`}else{detail.innerHTML=`<h2>CELL</h2><dl><dt>cell</dt><dd>${esc(item.name)}</dd><dt>kind</dt><dd>${esc(item.kind)}</dd><dt>BEL</dt><dd>${esc(item.bel)}</dd><dt>location</dt><dd>R${item.y}C${item.x}</dd><dt>setup</dt><dd class="${item.setup_slack_ps<0?'bad':''}">${slack(item.setup_slack_ps)}</dd><dt>hold</dt><dd class="${item.hold_slack_ps<0?'bad':''}">${slack(item.hold_slack_ps)}</dd></dl>`}}
function cellGeometry(cell){if(cell.kind==="global_clock")return [3,3,8,8];if(cell.kind==="block_ram")return [2,2,10,10];const logic=cell.bel.match(/SLICE([A-D])\.(K|FF)([01])/);if(logic){const column=logic[1].charCodeAt(0)-65,row=(logic[2]==="FF"?2:0)+Number(logic[3]);return [1+column*3,1+row*3,2.5,2.5]}const io=cell.bel.match(/PIO([A-D])/);if(io){const n=io[1].charCodeAt(0)-65;return [1+(n%2)*6,3+Math.floor(n/2)*6,5,5]}const n=cell.id%16;return [1+(n%4)*3,1+Math.floor(n/4)*3,2.5,2.5]}
function build(){
  for(let x=0;x<=DATA.extent.x+1;x++){gridLayer.append(el("line",{x1:x*U,y1:0,x2:x*U,y2:(DATA.extent.y+1)*U,class:"grid-line"}))}
  for(let y=0;y<=DATA.extent.y+1;y++){gridLayer.append(el("line",{x1:0,y1:y*U,x2:(DATA.extent.x+1)*U,y2:y*U,class:"grid-line"}))}
  for(const route of DATA.routes){const path=el("path",{d:segmentPath(route.segments),class:`route${labelSlack(route)<0?' violating':''}`,stroke:labelSlack(route)<0?"#ff627d":colorFor(route.name)});route.node=path;path.addEventListener("pointermove",e=>showTip(`${route.name}\n${route.pip_count} PIPs · setup ${slack(route.setup_slack_ps)} · hold ${slack(route.hold_slack_ps)}`,e));path.addEventListener("pointerleave",hideTip);path.addEventListener("click",e=>{e.stopPropagation();selectItem("route",route,path)});routeLayer.append(path);const fixed=segmentPath(route.segments,true);route.fixedNode=fixed?el("path",{d:fixed,class:"fixed-route"}):null;if(route.fixedNode)fixedLayer.append(route.fixedNode)}
  for(const cell of DATA.cells){const[oX,oY,w,h]=cellGeometry(cell),rect=el("rect",{x:cell.x*U+oX,y:cell.y*U+oY,width:w,height:h,rx:.7,fill:kindColor(cell.kind),class:"cell"});cell.node=rect;rect.addEventListener("pointermove",e=>showTip(`${cell.name}\n${cell.kind} @ ${cell.bel}`,e));rect.addEventListener("pointerleave",hideTip);rect.addEventListener("click",e=>{e.stopPropagation();selectItem("cell",cell,rect)});cellLayer.append(rect)}
}
function updateFilter(){const critical=document.querySelector("#critical-toggle").checked,threshold=Number(document.querySelector("#threshold").value);for(const route of DATA.routes){const hide=critical&&(labelSlack(route)===null||labelSlack(route)>threshold);route.node.classList.toggle("filtered",hide);route.fixedNode?.classList.toggle("filtered",hide)}}
function search(){const q=document.querySelector("#search").value.trim().toLowerCase();matches.replaceChildren();if(!q)return;const found=[];for(const item of DATA.routes)if(item.name.toLowerCase().includes(q))found.push(["route",item]);for(const item of DATA.cells)if(item.name.toLowerCase().includes(q)||item.bel.toLowerCase().includes(q)||item.kind.toLowerCase().includes(q))found.push(["cell",item]);for(const[type,item]of found.slice(0,100)){const b=document.createElement("button");b.className="match";b.innerHTML=`<span class="type">${type}</span><span class="name">${esc(item.name)}</span><span>${esc(type==="route"?item.pip_count:item.kind)}</span>`;b.onclick=()=>{selectItem(type,item,item.node);focusItem(type,item)};matches.append(b)}}
function focusItem(type,item){let x,y;if(type==="cell"){x=item.x*U+U/2;y=item.y*U+U/2}else if(item.segments.length){const s=item.segments[0];x=s[0]*U+U/2;y=s[1]*U+U/2}else return;view.w=Math.min(full.w,Math.max(90,view.w/3));view.h=view.w*svg.clientHeight/svg.clientWidth;view.x=x-view.w/2;view.y=y-view.h/2;setView()}
function initMeta(){document.title=`Texo · ${DATA.design}`;document.querySelector("#design").textContent=DATA.design;document.querySelector("#target").textContent=[DATA.target.device,DATA.target.package,DATA.target.speed_grade&&`speed ${DATA.target.speed_grade}`].filter(Boolean).join(" · ");const values=[["cells",DATA.cells.length],["nets",DATA.routes.length],["PIPs",DATA.metrics.total_pips??DATA.routes.reduce((n,r)=>n+r.pip_count,0)],["WNS",slack(DATA.timing.worst_slack_ps)],["WHS",slack(DATA.timing.worst_hold_slack_ps)],["unmapped PIPs",DATA.metrics.unmapped_pips]];document.querySelector("#stats").innerHTML=values.map(([k,v])=>`<div class="stat"><span class="muted">${k}</span><b>${v}</b></div>`).join("")}
document.querySelector("#fit").onclick=()=>{view={...full};setView()};document.querySelector("#routes-toggle").onchange=e=>routeLayer.style.display=fixedLayer.style.display=e.target.checked?"":"none";document.querySelector("#cells-toggle").onchange=e=>cellLayer.style.display=e.target.checked?"":"none";document.querySelector("#grid-toggle").onchange=e=>gridLayer.style.display=e.target.checked?"":"none";document.querySelector("#critical-toggle").onchange=updateFilter;document.querySelector("#threshold").oninput=updateFilter;document.querySelector("#search").oninput=search;document.addEventListener("keydown",e=>{if(e.key==="/"&&!/input/i.test(document.activeElement.tagName)){e.preventDefault();document.querySelector("#search").focus()}if(e.key==="Escape")clearSelection()});
svg.addEventListener("wheel",e=>{e.preventDefault();const r=svg.getBoundingClientRect(),px=view.x+(e.clientX-r.left)/r.width*view.w,py=view.y+(e.clientY-r.top)/r.height*view.h,f=Math.exp(e.deltaY*.001);view.w=Math.max(35,Math.min(full.w*8,view.w*f));view.h=Math.max(35,Math.min(full.h*8,view.h*f));view.x=px-(e.clientX-r.left)/r.width*view.w;view.y=py-(e.clientY-r.top)/r.height*view.h;setView()},{passive:false});svg.addEventListener("pointerdown",e=>{drag={x:e.clientX,y:e.clientY,vx:view.x,vy:view.y};svg.setPointerCapture(e.pointerId);svg.classList.add("dragging")});svg.addEventListener("pointermove",e=>{if(!drag)return;view.x=drag.vx-(e.clientX-drag.x)/svg.clientWidth*view.w;view.y=drag.vy-(e.clientY-drag.y)/svg.clientHeight*view.h;setView()});svg.addEventListener("pointerup",e=>{if(drag&&Math.hypot(e.clientX-drag.x,e.clientY-drag.y)<3)clearSelection();drag=null;svg.classList.remove("dragging")});
initMeta();build();setView();
</script>
</body>
</html>
"##;

/// Writes an interactive, dependency-free HTML view of a Texo checkpoint.
pub(crate) fn write_checkpoint_visualizer(
    checkpoint_path: &str,
    output_path: &str,
) -> Result<(), Box<dyn Error>> {
    let checkpoint: Value = serde_json::from_reader(BufReader::new(File::open(checkpoint_path)?))?;
    let html = checkpoint_visualizer(&checkpoint)?;
    let mut output = BufWriter::new(File::create(output_path)?);
    output.write_all(html.as_bytes())?;
    output.flush()?;
    Ok(())
}

fn checkpoint_visualizer(checkpoint: &Value) -> Result<String, Box<dyn Error>> {
    let data = visualization_data(checkpoint)?;
    let embedded = serde_json::to_string(&data)?
        .replace('<', "\\u003c")
        .replace('>', "\\u003e")
        .replace('\u{2028}', "\\u2028")
        .replace('\u{2029}', "\\u2029");
    Ok(HTML.replacen("__TEXO_DATA__", &embedded, 1))
}

fn visualization_data(checkpoint: &Value) -> Result<Value, Box<dyn Error>> {
    let placement = required_array(checkpoint, "placement")?;
    let routes = required_array(checkpoint, "routes")?;
    let kinds = checkpoint_cell_kinds(checkpoint);

    let timing = checkpoint.get("timing").unwrap_or(&Value::Null);
    let setup_nets = minimum_slacks(timing.get("net_setup_slacks"), "net_id");
    let setup_cells = minimum_slacks(timing.get("setup_checks"), "cell_id");
    let hold_cells = minimum_slacks(timing.get("hold_checks"), "cell_id");
    let hold_nets = hold_net_slacks(timing);

    let mut max_x = 0_u64;
    let mut max_y = 0_u64;
    let cells = placement
        .iter()
        .map(|cell| {
            let id = cell.get("cell_id").and_then(Value::as_u64).unwrap_or(0);
            let x = cell.get("x").and_then(Value::as_u64).unwrap_or(0);
            let y = cell.get("y").and_then(Value::as_u64).unwrap_or(0);
            max_x = max_x.max(x);
            max_y = max_y.max(y);
            json!({
                "id": id,
                "name": cell.get("cell").and_then(Value::as_str).unwrap_or("?"),
                "bel": cell.get("bel").and_then(Value::as_str).unwrap_or("?"),
                "kind": kinds
                    .get(&id)
                    .copied()
                    .or_else(|| cell.get("kind").and_then(Value::as_str))
                    .or_else(|| {
                        cell.get("bel")
                            .and_then(Value::as_str)
                            .filter(|bel| bel.contains("DCC"))
                            .map(|_| "global_clock")
                    })
                    .unwrap_or("cell"),
                "x": x,
                "y": y,
                "setup_slack_ps": setup_cells.get(&id),
                "hold_slack_ps": hold_cells.get(&id),
            })
        })
        .collect::<Vec<_>>();

    let mut unmapped_pips = 0_u64;
    let route_data = routes
        .iter()
        .map(|route| {
            let id = route.get("net_id").and_then(Value::as_u64).unwrap_or(0);
            let pips = route
                .get("pips")
                .and_then(Value::as_array)
                .map_or(&[][..], Vec::as_slice);
            let mut segments = Vec::with_capacity(pips.len());
            for pip in pips {
                let Some(from) = pip.get("from").and_then(Value::as_str).and_then(grid_point)
                else {
                    unmapped_pips += 1;
                    continue;
                };
                let Some(to) = pip.get("to").and_then(Value::as_str).and_then(grid_point) else {
                    unmapped_pips += 1;
                    continue;
                };
                max_x = max_x.max(from.0).max(to.0);
                max_y = max_y.max(from.1).max(to.1);
                segments.push(json!([
                    from.0,
                    from.1,
                    to.0,
                    to.1,
                    pip.get("fixed").and_then(Value::as_bool).unwrap_or(false)
                ]));
            }
            json!({
                "id": id,
                "name": route.get("net").and_then(Value::as_str).unwrap_or("?"),
                "pip_count": pips.len(),
                "setup_slack_ps": setup_nets.get(&id),
                "hold_slack_ps": hold_nets.get(&id),
                "segments": segments,
            })
        })
        .collect::<Vec<_>>();

    Ok(json!({
        "design": checkpoint.get("design").and_then(Value::as_str).unwrap_or("checkpoint"),
        "target": checkpoint.get("target").cloned().unwrap_or_else(|| json!({})),
        "metrics": {
            "total_pips": checkpoint.pointer("/metrics/total_pips"),
            "unmapped_pips": unmapped_pips,
        },
        "timing": {
            "worst_slack_ps": timing.get("worst_slack_ps"),
            "worst_hold_slack_ps": timing.get("worst_hold_slack_ps"),
        },
        "extent": {"x": max_x, "y": max_y},
        "cells": cells,
        "routes": route_data,
    }))
}

fn checkpoint_cell_kinds(checkpoint: &Value) -> BTreeMap<u64, &str> {
    let mut kinds = BTreeMap::new();
    for entry in checkpoint
        .get("primitive_metadata")
        .and_then(Value::as_array)
        .map_or(&[][..], Vec::as_slice)
    {
        if let (Some(id), Some(kind)) = (
            entry.get("cell_id").and_then(Value::as_u64),
            entry.pointer("/configuration/kind").and_then(Value::as_str),
        ) {
            kinds.insert(id, kind);
        }
    }
    for clock in checkpoint
        .pointer("/packing/global_clocks")
        .and_then(Value::as_array)
        .map_or(&[][..], Vec::as_slice)
    {
        if let Some(id) = clock.get("buffer").and_then(Value::as_u64) {
            kinds.insert(id, "global_clock");
        }
    }
    kinds
}

fn required_array<'a>(value: &'a Value, name: &str) -> Result<&'a Vec<Value>, Box<dyn Error>> {
    value.get(name).and_then(Value::as_array).ok_or_else(|| {
        io::Error::new(
            io::ErrorKind::InvalidData,
            format!("checkpoint has no `{name}` array"),
        )
        .into()
    })
}

fn minimum_slacks(entries: Option<&Value>, id_field: &str) -> BTreeMap<u64, i64> {
    let mut result = BTreeMap::<u64, i64>::new();
    for entry in entries
        .and_then(Value::as_array)
        .map_or(&[][..], Vec::as_slice)
    {
        let Some(id) = entry.get(id_field).and_then(Value::as_u64) else {
            continue;
        };
        let Some(slack) = entry.get("slack_ps").and_then(Value::as_i64) else {
            continue;
        };
        result
            .entry(id)
            .and_modify(|current| *current = (*current).min(slack))
            .or_insert(slack);
    }
    result
}

fn hold_net_slacks(timing: &Value) -> BTreeMap<u64, i64> {
    let mut pin_nets = BTreeMap::new();
    for delay in timing
        .get("net_delays")
        .and_then(Value::as_array)
        .map_or(&[][..], Vec::as_slice)
    {
        if let (Some(pin), Some(net)) = (
            delay.get("sink_pin_id").and_then(Value::as_u64),
            delay.get("net_id").and_then(Value::as_u64),
        ) {
            pin_nets.insert(pin, net);
        }
    }
    let mut result = BTreeMap::<u64, i64>::new();
    for check in timing
        .get("hold_checks")
        .and_then(Value::as_array)
        .map_or(&[][..], Vec::as_slice)
    {
        let Some(net) = check
            .get("data_pin_id")
            .and_then(Value::as_u64)
            .and_then(|pin| pin_nets.get(&pin).copied())
        else {
            continue;
        };
        let Some(slack) = check.get("slack_ps").and_then(Value::as_i64) else {
            continue;
        };
        result
            .entry(net)
            .and_modify(|current| *current = (*current).min(slack))
            .or_insert(slack);
    }
    result
}

fn grid_point(wire: &str) -> Option<(u64, u64)> {
    let rest = wire.strip_prefix('R')?;
    let (row, rest) = rest.split_once('C')?;
    let column_end = rest.find(|character: char| !character.is_ascii_digit())?;
    let column = &rest[..column_end];
    Some((column.parse().ok()?, row.parse().ok()?))
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::{checkpoint_visualizer, grid_point, visualization_data};

    #[test]
    fn extracts_grid_coordinates() {
        assert_eq!(grid_point("R31C66/H02E0201"), Some((66, 31)));
        assert_eq!(grid_point("not-a-wire"), None);
    }

    #[test]
    fn emits_a_self_contained_visualizer() {
        let checkpoint = json!({
            "design": "demo</script>",
            "target": {"device": "LFE5U-25F", "package": "CABGA381", "speed_grade": "6"},
            "metrics": {"total_pips": 1},
            "primitive_metadata": [{"cell_id": 0, "configuration": {"kind": "lut4"}}],
            "placement": [
                {"cell_id": 0, "cell": "lut", "bel": "R2C3/SLICEA", "x": 3, "y": 2},
                {"cell_id": 1, "cell": "$gbuf$n", "bel": "R0C4/TDCC0", "x": 4, "y": 0}
            ],
            "packing": {"global_clocks": [{"buffer": 1}]},
            "routes": [{"net_id": 7, "net": "n", "pips": [{"from": "R2C3/A", "to": "R4C5/B", "fixed": false}]}],
            "timing": {
                "worst_slack_ps": -8,
                "worst_hold_slack_ps": 12,
                "net_setup_slacks": [{"net_id": 7, "slack_ps": -8}]
            }
        });
        let data = visualization_data(&checkpoint).unwrap();
        let html = checkpoint_visualizer(&checkpoint).unwrap();

        assert_eq!(data["cells"][1]["kind"], "global_clock");
        assert!(html.starts_with("<!doctype html>"));
        assert!(html.contains("demo\\u003c/script\\u003e"));
        assert!(html.contains("\"segments\":[[3,2,5,4,false]]"));
        assert!(html.contains("net identity"));
        assert!(html.contains("violation (&lt;0 ps)"));
        assert!(!html.contains("__TEXO_DATA__"));
        assert!(!html.contains("https://"));
    }
}
