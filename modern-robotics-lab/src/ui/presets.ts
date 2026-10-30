/** A row of preset "chips"; the active one is highlighted. Can be driven from onStep. */
import { el } from './dom';

export interface Preset { label: string; apply: () => void; title?: string }

export interface PresetChipsHandle {
  el: HTMLElement;
  /** Apply preset i and highlight it. */
  activate(i: number): void;
  /** Remove the highlight (e.g. after the learner changes something manually). */
  clear(): void;
  readonly active: number;
}

export function presetChips(parent: HTMLElement, presets: Preset[], opts: { label?: string } = {}): PresetChipsHandle {
  let active = -1;
  const buttons: HTMLButtonElement[] = [];
  const wrap = el('div', { class: 'mr-chips' }, opts.label ? el('span', { class: 'mr-chips-label' }, opts.label) : null);
  presets.forEach((p, i) => {
    const b = el('button', { class: 'mr-chip', type: 'button', title: p.title, onClick: () => activate(i) }, p.label);
    buttons.push(b); wrap.appendChild(b);
  });
  parent.appendChild(wrap);
  function paint() { buttons.forEach((b, i) => b.classList.toggle('active', i === active)); }
  function activate(i: number) { active = i; paint(); presets[i]?.apply(); }
  return { el: wrap, activate, clear() { active = -1; paint(); }, get active() { return active; } };
}
