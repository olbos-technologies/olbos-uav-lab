import React,{useEffect,useMemo,useRef,useState} from 'react';

export type MapSample={east?:number;north?:number;altitude?:number;time_s?:number};
type Origin={lat:number;lon:number};
type Point={x:number;y:number};
type Props={samples:MapSample[]};

const DEFAULT_ORIGIN:Origin={lat:36.154,lon:-95.9928};
const WORLD_METERS=40075016.686;
const TILE_SIZE=256;
const DEFAULT_ZOOM=18;
const TILE_TEMPLATE=(import.meta as ImportMeta & {env?:{VITE_MAP_TILE_URL?:string}}).env?.VITE_MAP_TILE_URL
  || 'https://tile.openstreetmap.org/{z}/{x}/{y}.png';

function worldPixel(lat:number,lon:number,zoom:number):Point{
  const size=TILE_SIZE*2**zoom;
  const safeLat=Math.max(-85.05112878,Math.min(85.05112878,lat));
  const sin=Math.sin(safeLat*Math.PI/180);
  return {x:(lon+180)/360*size,y:(0.5-Math.log((1+sin)/(1-sin))/(4*Math.PI))*size};
}

function localToGeo(east:number,north:number,origin:Origin):Origin{
  return {
    lat:origin.lat+north/111320,
    lon:origin.lon+east/(111320*Math.cos(origin.lat*Math.PI/180))
  };
}

function tileX(x:number,zoom:number):number{
  const count=2**zoom;
  return ((x%count)+count)%count;
}

function MapTile({x,y,zoom,left,top}:{x:number;y:number;zoom:number;left:number;top:number}){
  const url=TILE_TEMPLATE.replace('{z}',String(zoom)).replace('{x}',String(x)).replace('{y}',String(y));
  return <img className="geo-map-tile" src={url} alt="" draggable={false} style={{left,top}} loading="eager" decoding="async"/>;
}

export default function GeoMap({samples}:Props){
  const [origin,setOrigin]=useState<Origin>(DEFAULT_ORIGIN);
  const [zoom,setZoom]=useState(DEFAULT_ZOOM);
  const [pan,setPan]=useState<Point>({x:0,y:0});
  const [size,setSize]=useState({width:0,height:0});
  const viewport=useRef<HTMLDivElement|null>(null);
  const drag=useRef<{pointerX:number;pointerY:number;panX:number;panY:number}|null>(null);

  useEffect(()=>{
    const element=viewport.current;
    if(!element)return;
    const observer=new ResizeObserver(entries=>{
      const box=entries[0]?.contentRect;
      if(box)setSize({width:box.width,height:box.height});
    });
    observer.observe(element);
    return()=>observer.disconnect();
  },[]);

  const center=worldPixel(origin.lat,origin.lon,zoom);
  const visibleTiles=useMemo(()=>{
    if(!size.width||!size.height)return [];
    const minX=Math.floor((center.x-size.width/2-pan.x)/TILE_SIZE);
    const maxX=Math.floor((center.x+size.width/2-pan.x-1)/TILE_SIZE);
    const minY=Math.max(0,Math.floor((center.y-size.height/2-pan.y)/TILE_SIZE));
    const maxY=Math.min(2**zoom-1,Math.floor((center.y+size.height/2-pan.y-1)/TILE_SIZE));
    const tiles:{x:number;y:number;left:number;top:number}[]=[];
    for(let y=minY;y<=maxY;y++)for(let x=minX;x<=maxX;x++){
      tiles.push({x:tileX(x,zoom),y,left:x*TILE_SIZE-center.x+size.width/2+pan.x,top:y*TILE_SIZE-center.y+size.height/2+pan.y});
    }
    return tiles;
  },[center.x,center.y,pan.x,pan.y,size.width,size.height,zoom]);

  const toScreen=(east:number,north:number):Point=>{
    const geo=localToGeo(east,north,origin),point=worldPixel(geo.lat,geo.lon,zoom);
    return {x:point.x-center.x+size.width/2+pan.x,y:point.y-center.y+size.height/2+pan.y};
  };
  const route=samples.filter(s=>Number.isFinite(s.east)&&Number.isFinite(s.north));
  const routePoints=route.map(s=>toScreen(s.east!,s.north!));
  const metersPerPixel=WORLD_METERS*Math.cos(origin.lat*Math.PI/180)/(TILE_SIZE*2**zoom);
  const radiusPx=40/metersPerPixel;
  const updateOrigin=(key:keyof Origin,value:string)=>{
    const numeric=Number(value);
    if(Number.isFinite(numeric)){
      const bounded=key==='lat'?Math.max(-85,Math.min(85,numeric)):Math.max(-180,Math.min(180,numeric));
      setOrigin(old=>({...old,[key]:bounded}));
    }
  };
  const startDrag=(event:React.PointerEvent<HTMLDivElement>)=>{
    if(event.button!==0)return;
    event.currentTarget.setPointerCapture(event.pointerId);
    drag.current={pointerX:event.clientX,pointerY:event.clientY,panX:pan.x,panY:pan.y};
  };
  const moveDrag=(event:React.PointerEvent<HTMLDivElement>)=>{
    if(!drag.current)return;
    setPan({x:drag.current.panX+event.clientX-drag.current.pointerX,y:drag.current.panY+event.clientY-drag.current.pointerY});
  };
  const endDrag=()=>{drag.current=null;};

  return <article className="panel geo-map-panel">
    <div className="geo-map-heading">
      <div><h3>Geographic route view</h3><span className="geo-map-subtitle">Open map tiles · local telemetry overlay</span></div>
      <span className="geo-map-badge">SIMULATION VIEW</span>
    </div>
    <div className="geo-map-stage" ref={viewport}
      onPointerDown={startDrag} onPointerMove={moveDrag} onPointerUp={endDrag} onPointerCancel={endDrag}
      role="region" aria-label="Interactive map with a local telemetry route drawn around the selected display origin">
      <div className="geo-map-tiles">{visibleTiles.map((tile,i)=><MapTile key={`${zoom}/${tile.x}/${tile.y}/${i}`} {...tile} zoom={zoom}/>)}</div>
      <svg className="geo-map-overlay" aria-hidden="true">
        <circle className="geo-map-fence" cx={size.width/2+pan.x} cy={size.height/2+pan.y} r={radiusPx}/>
        <line className="geo-map-origin-cross" x1={size.width/2+pan.x-7} y1={size.height/2+pan.y} x2={size.width/2+pan.x+7} y2={size.height/2+pan.y}/>
        <line className="geo-map-origin-cross" x1={size.width/2+pan.x} y1={size.height/2+pan.y-7} x2={size.width/2+pan.x} y2={size.height/2+pan.y+7}/>
        {routePoints.length>1&&<polyline className="geo-map-route" points={routePoints.map(p=>`${p.x},${p.y}`).join(' ')}/>}
        {routePoints.length>0&&<circle className="geo-map-aircraft" cx={routePoints.at(-1)!.x} cy={routePoints.at(-1)!.y} r="6"/>}
      </svg>
      {!samples.length&&<div className="geo-map-empty">Waiting for telemetry replay<br/><small>Run a demo and start the replay bridge to draw a route.</small></div>}
      <div className="geo-map-controls">
        <button type="button" aria-label="Zoom in" onPointerDown={e=>e.stopPropagation()} onClick={()=>setZoom(z=>Math.min(20,z+1))}>+</button>
        <button type="button" aria-label="Zoom out" onPointerDown={e=>e.stopPropagation()} onClick={()=>setZoom(z=>Math.max(13,z-1))}>−</button>
        <button type="button" onPointerDown={e=>e.stopPropagation()} onClick={()=>{setPan({x:0,y:0});setZoom(DEFAULT_ZOOM);}}>Center</button>
      </div>
      <a className="geo-map-attribution" href="https://www.openstreetmap.org/copyright" target="_blank" rel="noreferrer">© OpenStreetMap contributors</a>
    </div>
    <div className="geo-map-footer">
      <div className="geo-map-origin">
        <label>DISPLAY ORIGIN LAT<input aria-label="Display origin latitude" type="number" min="-85" max="85" step="0.000001" value={origin.lat.toFixed(6)} onChange={e=>updateOrigin('lat',e.target.value)}/></label>
        <label>LON<input aria-label="Display origin longitude" type="number" min="-180" max="180" step="0.000001" value={origin.lon.toFixed(6)} onChange={e=>updateOrigin('lon',e.target.value)}/></label>
      </div>
      <small>{route.length} plotted samples · 40 m reference ring · zoom {zoom}</small>
    </div>
    <p className="geo-map-disclaimer">LOCAL ENU offsets are projected around this display origin. This is not GPS and must not be used for navigation.</p>
  </article>;
}
