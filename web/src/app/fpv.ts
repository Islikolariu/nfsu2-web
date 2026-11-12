import type { PadSnapshot } from '../input/gamepad';
import { PAD_AXIS_LX, PAD_AXIS_LY, PAD_AXIS_RX, PAD_AXIS_RY } from '../input/proto';
import { t } from './i18n';

// Matches the photo input commands and status bits in runtime/photo.c.
const PH_FPV = 31, PH_FPV_STICKS = 32;
const CAN_OPEN = 1024, ACTIVE = 2048;
const CLEAR = 0x80808080 | 0;
const allowedKeys = ['KeyI', 'KeyK', 'KeyJ', 'KeyL', 'KeyA', 'KeyD', 'KeyW', 'KeyS'];
const axisDefault = [1, 2, 4, -3];     // signed one-based indexes: roll, pitch, yaw, throttle
const params = new URLSearchParams(location.search);
const axisOption = (params.get('fpv_axes') ?? '').split(',').map(Number);
const axes = axisOption.length === 4 && axisOption.every(v => Number.isInteger(v) && Math.abs(v) >= 1 && Math.abs(v) <= 32)
  && new Set(axisOption.map(Math.abs)).size === 4 ? axisOption : axisDefault;
const padName = params.get('fpv_pad')?.trim().toLowerCase();
const enabled = params.get('fpv') === '1' && ['localhost', '127.0.0.1', '[::1]'].includes(location.hostname);

function packed(sticks: number[]) {
  return sticks.reduce((out, axis, i) => {
    const n = Number.isFinite(axis) && Math.abs(axis) >= 0.06 ? Math.round(Math.max(-1, Math.min(1, axis)) * 127) : 0;
    return out | (n & 255) << (8 * i);
  }, 0);
}

/** Browser-side controls for the client-only FPV camera. */
export class FpvControls {
  can = false;
  active = false;
  private keys = new Set<string>();
  private pad: PadSnapshot | null = null;
  private last = -Infinity;
  private source: string | null = null;
  private ready = false;
  private tag: HTMLElement;

  constructor(stage: HTMLElement, private send: (cmd: number, value: number) => boolean,
    private release: () => void) {
    this.tag = document.createElement('div');
    this.tag.className = 'fpv-tag';
    this.tag.textContent = `FPV · ${t('Back')}: F8 / Esc`;
    this.tag.hidden = true;
    stage.append(this.tag);
  }

  status(bits: number) {
    this.can = enabled && (bits & CAN_OPEN) !== 0;
    const active = enabled && (bits & ACTIVE) !== 0;
    if (active === this.active) return;
    this.active = active;
    this.tag.hidden = !active;
    this.clear();
    this.release();
  }

  toggle() {
    if (!this.active && !this.can) return false;
    this.release();
    return this.send(PH_FPV, this.active ? 0 : 1);
  }

  reset() {
    this.status(0);
    this.ready = false;
  }

  key(e: KeyboardEvent, down: boolean, dialog: boolean) {
    if (!this.active) return false;
    if (!down) this.keys.delete(e.code);
    if (dialog || e.ctrlKey || e.metaKey || e.altKey) return false;
    if (down && !e.repeat && e.code === 'Escape') this.send(PH_FPV, 0);
    if (down && allowedKeys.includes(e.code)) this.keys.add(e.code);
    return true;
  }

  capture(pad: PadSnapshot) {
    if (!this.active) return false;
    this.pad = pad.connected ? pad : null;
    return true;
  }

  clear() {
    this.keys.clear();
    this.pad = null;
    this.source = null;
    this.last = -Infinity;
    if (this.active) this.send(PH_FPV_STICKS, CLEAR);
  }

  tick(blocked: boolean, now: number) {
    if (!enabled) return;
    if (!this.ready) this.ready = this.send(PH_FPV, 2);
    if (!this.active || now - this.last < 40) return;
    if (blocked) {
      this.keys.clear();
      this.pad = null;
      this.source = null;
      if (this.send(PH_FPV_STICKS, CLEAR)) this.last = now;
      return;
    }
    const key = (positive: string, negative: string) => Number(this.keys.has(positive)) - Number(this.keys.has(negative));
    const keyboard = [key('KeyL', 'KeyJ'), key('KeyK', 'KeyI'), key('KeyD', 'KeyA'), key('KeyW', 'KeyS')];
    const radio = Array.from(navigator.getGamepads?.() ?? []).find(p =>
      p?.connected && axes.every(v => Number.isFinite(p.axes[Math.abs(v) - 1])) &&
      (padName ? p.id.toLowerCase().includes(padName) : /2ca3.*1021|dji|remote controller 3/i.test(p.id)));
    const a = this.pad && !this.pad.wheel ? this.pad.axes : null;
    const useKeys = !radio && !a;
    const source = useKeys ? 'keyboard' : radio ? `radio:${radio.index}:${radio.id}` : `pad:${this.pad?.index}:${this.pad?.name}`;
    if (source !== this.source) {
      if (this.send(PH_FPV_STICKS, CLEAR)) {
        this.source = source;
        this.last = now;
      }
      return;
    }
    // Spring-centred controllers and the keyboard have zero throttle at rest.
    const sticks = useKeys ? [keyboard[0] * 0.55, keyboard[1] * 0.55, keyboard[2] * 0.55, keyboard[3] > 0 ? 0.45 : -1]
      : radio ? axes.map(v => radio.axes[Math.abs(v) - 1] * Math.sign(v))
      : a ? [a[PAD_AXIS_RX], a[PAD_AXIS_RY], a[PAD_AXIS_LX], 2 * Math.max(0, -a[PAD_AXIS_LY]) - 1]
      : [0, 0, 0, -1];
    if (this.send(PH_FPV_STICKS, packed(sticks))) this.last = now;
  }
}
