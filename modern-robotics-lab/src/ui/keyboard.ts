/** Keyboard driving (WASD / arrows / space) scoped to a focusable element, with cleanup. */
import { el } from './dom';

export type DriveAction = 'up' | 'down' | 'left' | 'right' | 'brake' | 'a' | 'b';

const DEFAULT_KEYS: Record<string, DriveAction> = {
  KeyW: 'up', ArrowUp: 'up', KeyS: 'down', ArrowDown: 'down', KeyA: 'left', ArrowLeft: 'left', KeyD: 'right', ArrowRight: 'right',
  Space: 'brake', KeyQ: 'a', KeyE: 'b',
};

export interface KeyboardDriveHandle {
  /** Currently held actions. */
  pressed: Set<DriveAction>;
  /** −1, 0 or +1 from a pair of actions. */
  axis(neg: DriveAction, pos: DriveAction): number;
  has(a: DriveAction): boolean;
  destroy(): void;
  readonly focused: boolean;
}

/**
 * Makes `target` focusable and listens for keys while it has focus. Shows a hint overlay
 * ("點一下畫面以啟用鍵盤") until focused. Does not steal keys from inputs.
 */
export function keyboardDrive(target: HTMLElement, opts: { keys?: Record<string, DriveAction>; hint?: string; onChange?: (pressed: Set<DriveAction>) => void } = {}): KeyboardDriveHandle {
  const keys = opts.keys ?? DEFAULT_KEYS;
  const pressed = new Set<DriveAction>();
  target.tabIndex = 0;
  target.style.outline = 'none';
  const hint = el('div', { class: 'mr-key-hint' }, opts.hint ?? '點一下畫面以啟用鍵盤（WASD / 方向鍵）');
  if (getComputedStyle(target).position === 'static') target.style.position = 'relative';
  target.appendChild(hint);
  let focused = false;
  const onFocus = () => { focused = true; hint.style.display = 'none'; };
  const onBlur = () => { focused = false; hint.style.display = ''; pressed.clear(); opts.onChange?.(pressed); };
  const onDown = (ev: KeyboardEvent) => {
    const a = keys[ev.code];
    if (!a) return;
    ev.preventDefault();
    if (!pressed.has(a)) { pressed.add(a); opts.onChange?.(pressed); }
  };
  const onUp = (ev: KeyboardEvent) => {
    const a = keys[ev.code];
    if (!a) return;
    ev.preventDefault();
    if (pressed.delete(a)) opts.onChange?.(pressed);
  };
  const onClick = () => target.focus({ preventScroll: true });
  target.addEventListener('focus', onFocus);
  target.addEventListener('blur', onBlur);
  target.addEventListener('keydown', onDown);
  target.addEventListener('keyup', onUp);
  target.addEventListener('pointerdown', onClick);
  return {
    pressed,
    axis: (neg, pos) => (pressed.has(pos) ? 1 : 0) - (pressed.has(neg) ? 1 : 0),
    has: (a) => pressed.has(a),
    get focused() { return focused; },
    destroy() {
      target.removeEventListener('focus', onFocus); target.removeEventListener('blur', onBlur);
      target.removeEventListener('keydown', onDown); target.removeEventListener('keyup', onUp);
      target.removeEventListener('pointerdown', onClick);
      hint.remove();
    },
  };
}
