import test from 'node:test';
import assert from 'node:assert/strict';
import {satelliteStyle} from '../src/mapSources.ts';
import coverage from '../src/satelliteCoverage.json' with {type: 'json'};

const origin = 'http://192.168.0.101:8899';

test('offline VIT Vellore is drawn from zoom 0, not only the simulator box', () => {
  const {sources, layers} = satelliteStyle(coverage, false, origin);
  const vit = Object.values(sources).find(s => s.bounds[0] <= 79.1559 && s.bounds[2] >= 79.1559
                                              && s.bounds[1] <= 12.9692 && s.bounds[3] >= 12.9692);
  assert.ok(vit, 'a source covers VIT Vellore');
  assert.equal(vit.minzoom, 0);
  assert.equal(vit.tiles[0], `${origin}/satellite/{z}/{x}/{y}.jpg`);
  assert.deepEqual(coverage.initial.center, [79.1559, 12.9692]);
  assert.deepEqual(layers.map(l => l.source), Object.keys(sources));
});

test('each offline region only requests tiles inside its own bounds', () => {
  const {sources} = satelliteStyle(coverage, false, origin);
  for (const region of coverage.regions) {
    const source = Object.values(sources).find(s => s.bounds.join() === region.bounds.join());
    assert.ok(source, region.name);
    assert.equal(source.minzoom, region.minzoom);
    assert.equal(source.maxzoom, region.maxzoom);
  }
});

test('coarse regions draw under detail regions', () => {
  const {sources, layers} = satelliteStyle(coverage, false, origin);
  const minzooms = layers.map(l => sources[l.source].minzoom);
  assert.deepEqual(minzooms, [...minzooms].sort((a, b) => a - b));
});

test('online imagery is one unbounded source', () => {
  const {sources, layers} = satelliteStyle(coverage, true, origin);
  assert.equal(layers.length, 1);
  const [source] = Object.values(sources);
  assert.equal(source.bounds, undefined);
  assert.match(source.tiles[0], /^https:\/\/services\.arcgisonline\.com\//);
});
