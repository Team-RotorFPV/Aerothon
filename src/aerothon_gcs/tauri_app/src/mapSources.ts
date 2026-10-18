// Satellite sources for the map tab. Offline, each coverage region (see
// scripts/map_regions.json) is its own raster source bounded to where tiles
// exist, so MapLibre neither requests missing tiles nor blanks the view: above
// a region's maxzoom it overscales that region's deepest tiles.
export interface Region { name: string; bounds: number[]; minzoom: number; maxzoom: number }
export interface Coverage { initial: { center: number[]; zoom: number }; regions: Region[] }

const ATTRIBUTION = '© Esri, Maxar, Earthstar Geographics, GIS User Community';
const ONLINE = 'https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}';

export function satelliteStyle(coverage: Coverage, online: boolean, origin: string) {
  const sources: Record<string, any> = {};
  if (online) {
    sources['satellite'] = { type: 'raster', tiles: [ONLINE], tileSize: 256, minzoom: 0, maxzoom: 19,
                             attribution: ATTRIBUTION };
  } else {
    [...coverage.regions].sort((a, b) => a.minzoom - b.minzoom).forEach((r, i) => {
      sources[`satellite-${i}`] = { type: 'raster', tiles: [`${origin}/satellite/{z}/{x}/{y}.jpg`],
                                    tileSize: 256, bounds: r.bounds, minzoom: r.minzoom,
                                    maxzoom: r.maxzoom, attribution: ATTRIBUTION };
    });
  }
  const layers = Object.keys(sources).map(id => ({ id, type: 'raster', source: id }));
  return { sources, layers };
}
