/** A row of preset "chips"; the active one is highlighted. Can be driven from onStep. */
import { el } from './dom';

export interface Preset { label: string; apply: () => void; title?: string }

export interface PresetChipsHandle {
  el: HTMLElement;
  /** Apply preset i and highlight it. */
  activate(i: number): void;
  /** Highlight preset i without running its apply callback (e.g. to reflect state restored elsewhere). */
  highlight(i: number): void;
  /** Remove the highlight (e.g. after the learner changes something manually). */
  clear(): void;
  /** Replace the whole set of presets (e.g. a different set per step). Clears the highlight. */
  setPresets(presets: Preset[]): void;
  readonly active: number;
}

export function presetChips(parent: HTMLElement, presets: Preset[], opts: { label?: string } = {}): PresetChipsHandle {
  let active = -1;
  let list = presets;
  let buttons: HTMLButtonElement[] = [];
  const wrap = el('div', { class: 'mr-chips' });
  const labelEl = opts.label ? el('span', { class: 'mr-chips-label' }, opts.label) : null;
  function paint() { buttons.forEach((b, i) => b.classList.toggle('active', i === active)); }
  function activate(i: number) { active = i; paint(); list[i]?.apply(); }
  function render() {
    buttons = list.map((p, i) => el('button', { class: 'mr-chip', type: 'button', title: p.title, onClick: () => activate(i) }, p.label));
    wrap.replaceChildren(...(labelEl ? [labelEl] : []), ...buttons);
    paint();
  }
  render();
  parent.appendChild(wrap);
  return {
    el: wrap,
    activate,
    highlight(i) { active = i; paint(); },
    clear() { active = -1; paint(); },
    setPresets(p) { list = p; active = -1; render(); },
    get active() { return active; },
  };
}
