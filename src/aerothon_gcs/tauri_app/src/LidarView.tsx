import { useEffect, useRef, useState } from 'react';
import { gridBounds, scanPoints, type Grid, type Scan } from './lidarGeometry';
export type { Grid } from './lidarGeometry';

export function MetricPlot({scan, grid, fresh=true}: {scan?:Scan; grid?:Grid|null; fresh?:boolean}) {
  const canvas = useRef<HTMLCanvasElement>(null);
  const [size,setSize] = useState([700,500]);
  const [view,setView] = useState({x:0,y:0,span:12});
  const drag = useRef<{x:number;y:number;vx:number;vy:number}|null>(null);
  const fitted = useRef(false);
  const fit = () => {
    if(grid) {
      const b=gridBounds(grid);
      setView({x:(b.minX+b.maxX)/2,y:(b.minY+b.maxY)/2,
        span:Math.max(2,b.maxY-b.minY,(b.maxX-b.minX)*size[1]/size[0])*1.15});
    } else setView({x:0,y:0,span:12});
  };
  useEffect(()=>{ fitted.current=false; },[Boolean(grid)]);
  useEffect(()=>{ if(grid&&!fitted.current){ fit();fitted.current=true; } },[grid]);
  useEffect(()=>{
    if(!canvas.current)return;
    const observer=new ResizeObserver(entries=>{const r=entries[0].contentRect;setSize([r.width,r.height]);});
    observer.observe(canvas.current);return()=>observer.disconnect();
  },[]);
  useEffect(()=>{
    const node=canvas.current;if(!node)return;
    const wheel=(e:WheelEvent)=>{e.preventDefault();setView(v=>({...v,span:Math.max(.5,Math.min(200,v.span*Math.exp(e.deltaY*.001)))}));};
    node.addEventListener('wheel',wheel,{passive:false});return()=>node.removeEventListener('wheel',wheel);
  },[]);
  useEffect(()=>{
    const node=canvas.current, c=node?.getContext('2d');if(!node||!c)return;
    const [w,h]=size,dpr=window.devicePixelRatio||1,k=h/view.span;
    node.width=Math.round(w*dpr);node.height=Math.round(h*dpr);c.scale(dpr,dpr);
    const css=getComputedStyle(node),text=css.getPropertyValue('--text').trim()||'#d8e2ee';
    c.fillStyle=css.getPropertyValue('--bg').trim()||'#101824';c.fillRect(0,0,w,h);
    const xy=(x:number,y:number)=>[w/2+(x-view.x)*k,h/2-(y-view.y)*k];
    const step=10**Math.floor(Math.log10(view.span/7));
    const spacing=view.span/step>15?step*2:step;
    c.lineWidth=1;c.strokeStyle='#61758b44';c.font='11px sans-serif';c.fillStyle=text;
    for(let x=Math.floor((view.x-w/k/2)/spacing)*spacing;x<view.x+w/k/2;x+=spacing){
      const [sx]=xy(x,0);c.beginPath();c.moveTo(sx,0);c.lineTo(sx,h);c.stroke();
      if(grid)c.fillText(`${Number(x.toFixed(2))}`,sx+3,h-10);
    }
    for(let y=Math.floor((view.y-view.span/2)/spacing)*spacing;y<view.y+view.span/2;y+=spacing){
      const [,sy]=xy(0,y);c.beginPath();c.moveTo(0,sy);c.lineTo(w,sy);c.stroke();
      if(grid)c.fillText(`${Number(y.toFixed(2))}`,5,sy-4);
    }
    if(grid&&grid.data.length===grid.w*grid.h){
      const off=document.createElement('canvas');off.width=grid.w;off.height=grid.h;
      const oc=off.getContext('2d')!;const pixels=oc.createImageData(grid.w,grid.h);
      grid.data.forEach((v,i)=>{
        const j=((grid.h-1-Math.floor(i/grid.w))*grid.w+i%grid.w)*4;
        const rgb=v<0?[104,117,133]:v>=65?[28,35,43]:v<=25?[235,239,243]:[160,170,181];
        pixels.data.set([...rgb,255],j);
      });oc.putImageData(pixels,0,0);
      c.save();c.globalAlpha=fresh?1:.5;c.translate(w/2,h/2);c.scale(k,-k);c.translate(-view.x,-view.y);
      c.translate(grid.ox,grid.oy);c.rotate(grid.yaw||0);c.scale(grid.res,-grid.res);c.imageSmoothingEnabled=false;
      c.drawImage(off,0,-grid.h);c.restore();
    } else {
      const [ox,oy]=xy(0,0);
      c.strokeStyle='#8296ae66';
      for(let r=spacing;r<view.span;r+=spacing){
        c.beginPath();c.arc(ox,oy,r*k,0,Math.PI*2);c.stroke();
        if(r*k<h/2-15)c.fillText(`${Number(r.toFixed(2))} m`,ox+5,oy-r*k+12);
      }
      c.fillStyle=fresh?'#45c9e8':'#8995a2';
      scanPoints(scan).forEach(([forward,left])=>{const [x,y]=xy(-left,forward);c.fillRect(x-1.25,y-1.25,2.5,2.5);});
      c.strokeStyle='#f6b652';c.lineWidth=2;c.beginPath();c.moveTo(ox,oy+10);c.lineTo(ox,oy-18);c.lineTo(ox-5,oy-10);c.moveTo(ox,oy-18);c.lineTo(ox+5,oy-10);c.stroke();
      c.fillStyle=text;c.fillText('Nose 0°',ox+8,oy-18);c.fillText('Left +90°',ox-85,oy+20);
    }
    c.fillStyle=text;c.fillRect(20,h-32,k,2);c.fillText('1 m',20,h-40);
  },[scan,grid,fresh,size,view]);
  return <div className="metric-plot">
    <div className="plot-tools"><button onClick={fit}>Fit view</button>
      <button aria-label="Zoom in" onClick={()=>setView(v=>({...v,span:Math.max(.5,v.span/1.5)}))}>+</button>
      <button aria-label="Zoom out" onClick={()=>setView(v=>({...v,span:Math.min(200,v.span*1.5)}))}>−</button>
      <span>Drag to pan · Scroll to zoom · {scan&&!grid?`${scanPoints(scan).length} valid / ${scan.ranges.length} beams`:'Metric scale'}</span></div>
    <canvas ref={canvas} aria-label={grid?'SLAM occupancy grid with metric scale':'Live lidar scan, nose up, full resolution'}
      onPointerDown={e=>{e.currentTarget.setPointerCapture(e.pointerId);drag.current={x:e.clientX,y:e.clientY,vx:view.x,vy:view.y};}}
      onPointerMove={e=>{const d=drag.current;if(d)setView(v=>({...v,x:d.vx-(e.clientX-d.x)*v.span/size[1],y:d.vy+(e.clientY-d.y)*v.span/size[1]}));}}
      onPointerUp={()=>{drag.current=null;}} onPointerCancel={()=>{drag.current=null;}} />
  </div>;
}

export default function Slam({grid,current,scan,lidarFresh,slamEnabled,status}: {
  grid:Grid|null;current:boolean;scan?:Scan;lidarFresh:boolean;poseFresh?:boolean;
  slamEnabled:boolean;status?:{state?:string;reason?:string;source?:string};
}) {
  const [mode,setMode]=useState<'map'|'scan'>('map');
  const showMap=mode==='map'&&!!grid;
  return <div className="slam-view">
    <div className="actions"><button aria-pressed={mode==='map'} onClick={()=>setMode('map')}>SLAM map</button>
      <button aria-pressed={mode==='scan'} onClick={()=>setMode('scan')}>Live lidar</button>
      <span>{showMap?(current?'Live map':'Map stale'):(lidarFresh?'Live scan':'Scan stale')}</span></div>
    <MetricPlot scan={scan} grid={showMap?grid:null} fresh={showMap?current:lidarFresh}/>
    {showMap&&grid?<p>Frame {grid.frame} · {grid.res} m/cell · {grid.w} × {grid.h} cells · origin {grid.ox.toFixed(2)}, {grid.oy.toFixed(2)} m · local map, no GPS required</p>:
      <p>{!lidarFresh?'Waiting for live lidar.':!slamEnabled?'SLAM is disabled in this profile. Live scan needs no GPS.':!grid?'Waiting for a map. Full lidar scan shown.':'Full scan in the aircraft frame. No GPS required.'}</p>}
    {status&&<p role="status">{status.source||'LiDAR odometry'} · {status.state} {status.reason}</p>}
    <p>Light = free · dark = occupied · grey = unknown. Local scan matching does not establish GPS position.</p>
  </div>;
}
