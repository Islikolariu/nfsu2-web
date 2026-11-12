import { test } from 'node:test';
import assert from 'node:assert/strict';
import { fileURLToPath } from 'node:url';
import { build } from 'esbuild';

const compiled = await build({ entryPoints: [fileURLToPath(new URL('../src/app/fpv.ts', import.meta.url))],
  bundle: true, format: 'esm', write: false });
const moduleUrl = 'data:text/javascript;base64,' + Buffer.from(compiled.outputFiles[0].text).toString('base64');
const CLEAR = 0x80808080 | 0;
const channels = value => [0, 1, 2, 3].map(i => value << (24 - 8 * i) >> 24);

async function setup(url) {
  let pads = [];
  const old = Object.fromEntries(['location', 'navigator', 'document'].map(k => [k, Object.getOwnPropertyDescriptor(globalThis, k)]));
  const mock = { location: new URL(url), navigator: { getGamepads: () => pads }, document: { createElement: () => ({}) } };
  for (const [k, value] of Object.entries(mock)) Object.defineProperty(globalThis, k, { configurable: true, value });
  try {
    const { FpvControls } = await import(moduleUrl + '#' + encodeURIComponent(url));
    const sent = [], tag = [];
    const controls = new FpvControls({ append: node => tag.push(node) }, (cmd, v) => { sent.push([cmd, v]); return true; }, () => {});
    return { controls, sent, tag: tag[0], setPads: value => { pads = value; }, restore };
  } catch (e) {
    restore();
    throw e;
  }
  function restore() {
    for (const [k, descriptor] of Object.entries(old)) {
      if (descriptor) Object.defineProperty(globalThis, k, descriptor);
      else delete globalThis[k];
    }
  }
}

test('the web client only opts in on a local URL with fpv=1', async () => {
  for (const url of ['http://localhost:8611/', 'https://example.test/?fpv=1']) {
    const { controls, sent, tag, restore } = await setup(url);
    try {
      controls.tick(false, 0);
      controls.status(1024 | 2048);
      assert.equal(controls.toggle(), false);
      assert.equal(tag.hidden, true);
      assert.deepEqual(sent, []);
    } finally { restore(); }
  }
});

test('keyboard zero throttle, takeoff, and focus loss send distinct inputs', async () => {
  const { controls, sent, tag, restore } = await setup('http://localhost:8611/?fpv=1');
  try {
    controls.tick(false, 0);
    controls.status(1024);
    assert.equal(controls.toggle(), true);
    assert.deepEqual(sent, [[31, 2], [31, 1]]);
    controls.status(1024 | 2048);
    assert.equal(tag.hidden, false);
    controls.tick(false, 40);
    controls.tick(false, 80);
    assert.deepEqual(channels(sent.at(-1)[1]), [0, 0, 0, -127]);
    controls.key({ code: 'KeyW' }, true, false);
    controls.tick(false, 120);
    assert.ok(channels(sent.at(-1)[1])[3] > 0);
    controls.tick(true, 160);
    assert.deepEqual(sent.at(-1), [32, CLEAR]);
    controls.key({ code: 'Escape' }, true, false);
    assert.deepEqual(sent.at(-1), [31, 0]);
  } finally { restore(); }
});

test('DJI raw axes use their configured signs and a disconnect clears the flight inputs', async () => {
  const { controls, sent, setPads, restore } = await setup('http://127.0.0.1:8611/?fpv=1&fpv_axes=-2,1,3,-4');
  try {
    controls.tick(false, 0);
    controls.status(2048);
    setPads([{ connected: true, id: 'HID Interface (Vendor: 2ca3 Product: 1021)', index: 2, axes: [0.5, -0.5, 0.25, 1] }]);
    controls.tick(false, 40);
    controls.tick(false, 80);
    assert.deepEqual(channels(sent.at(-1)[1]), [64, 64, 32, -127]);
    setPads([]);
    controls.tick(false, 120);
    assert.deepEqual(sent.at(-1), [32, CLEAR]);
  } finally { restore(); }
});
