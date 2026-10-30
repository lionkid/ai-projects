#!/usr/bin/env node
/**
 * Smoke test: boots the Vite dev server, opens every lesson (or the ones given with
 * --lesson id1,id2) in headless Chromium, walks through all steps, wiggles every slider,
 * clicks every button once, answers the quiz, and reports console/page errors.
 *
 *   node scripts/smoke.mjs                      # all lessons
 *   node scripts/smoke.mjs --lesson ch03-01-x   # one lesson
 *   node scripts/smoke.mjs --screens out/dir    # also save screenshots
 *   node scripts/smoke.mjs --json               # machine-readable summary on stdout
 *
 * Requires Playwright + Chromium to be resolvable from this directory (pre-installed in
 * the cloud environment; locally run `npm i -D playwright && npx playwright install chromium`).
 */
import { createServer } from 'vite';
import { createRequire } from 'node:module';
import path from 'node:path';
import fs from 'node:fs';

const require = createRequire(import.meta.url);
const { chromium } = require('playwright');

const argv = process.argv.slice(2);
const opt = (name, def) => { const i = argv.indexOf(`--${name}`); return i >= 0 ? (argv[i + 1] ?? true) : def; };
const only = opt('lesson', null);
const screens = opt('screens', null);
const asJson = argv.includes('--json');
const settle = Number(opt('settle', 500));

const root = path.resolve(new URL('..', import.meta.url).pathname);
const port = 5300 + Math.floor(Math.random() * 600);
const server = await createServer({ root, configFile: path.join(root, 'vite.config.ts'), logLevel: 'error', server: { port, strictPort: false, host: '127.0.0.1' } });
await server.listen();
const base = server.resolvedUrls.local[0];

const launchOpts = { args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist', '--no-sandbox'] };
if (process.env.PW_CHROMIUM_PATH) launchOpts.executablePath = process.env.PW_CHROMIUM_PATH;
const browser = await chromium.launch(launchOpts);
const page = await browser.newPage({ viewport: { width: 1400, height: 950 } });

const errors = [];
const benign = /favicon|ERR_BLOCKED_BY_CLIENT|Download the React DevTools/i;
page.on('pageerror', (e) => errors.push(`pageerror: ${e.message}`));
page.on('console', (m) => { if (m.type() === 'error' && !benign.test(m.text())) errors.push(`console: ${m.text()}`); });

await page.goto(base, { waitUntil: 'load' });
await page.waitForFunction(() => !!window.__MRLAB__, null, { timeout: 30000 });
const all = await page.evaluate(() => window.__MRLAB__.lessons.map((l) => l.id));
const ids = only ? String(only).split(',').map((s) => s.trim()).filter(Boolean) : all;
const unknown = ids.filter((id) => !all.includes(id));
if (unknown.length) { console.error(`Unknown lesson id(s): ${unknown.join(', ')}\nKnown: ${all.join(', ')}`); await browser.close(); await server.close(); process.exit(2); }
if (screens) fs.mkdirSync(screens, { recursive: true });

const results = [];
for (const id of ids) {
  errors.length = 0;
  const r = { id, ok: false, steps: 0, canvases: 0, sliders: 0, buttons: 0, quiz: 0, errors: [], ms: 0 };
  const t0 = Date.now();
  try {
    await page.goto(`${base}#/lesson/${id}`, { waitUntil: 'load' });
    await page.waitForSelector(`[data-lesson-ready="${id}"]`, { timeout: 30000 });
    await page.waitForTimeout(settle);
    const info = await page.evaluate(() => {
      const host = document.querySelector('.sim-host');
      return { canvases: host.querySelectorAll('canvas, svg').length, hasError: host.classList.contains('error'), text: host.textContent.slice(0, 300) };
    });
    r.canvases = info.canvases;
    if (info.hasError) errors.push(`sim-host error: ${info.text}`);
    // Walk through every step via the primary button.
    r.steps = await page.locator('.stepper-bar .dot').count();
    for (let k = 1; k < r.steps; k++) {
      await page.locator('.stepper-nav .btn.primary').click();
      await page.waitForTimeout(150);
    }
    // Wiggle sliders: max → min → original.
    r.sliders = await page.evaluate(async () => {
      const inputs = [...document.querySelectorAll('.sim-host input[type="range"]')];
      const fire = (inp, v) => { inp.value = String(v); inp.dispatchEvent(new Event('input', { bubbles: true })); inp.dispatchEvent(new Event('change', { bubbles: true })); };
      for (const inp of inputs) {
        const orig = inp.value;
        fire(inp, inp.max); await new Promise((res) => setTimeout(res, 30));
        fire(inp, inp.min); await new Promise((res) => setTimeout(res, 30));
        fire(inp, orig); await new Promise((res) => setTimeout(res, 30));
      }
      return inputs.length;
    });
    // Toggle selects/checkboxes once and back.
    await page.evaluate(async () => {
      for (const sel of document.querySelectorAll('.sim-host select')) {
        const orig = sel.value;
        for (const o of sel.options) { sel.value = o.value; sel.dispatchEvent(new Event('change', { bubbles: true })); await new Promise((res) => setTimeout(res, 40)); }
        sel.value = orig; sel.dispatchEvent(new Event('change', { bubbles: true }));
      }
      for (const cb of document.querySelectorAll('.sim-host input[type="checkbox"]')) {
        cb.click(); await new Promise((res) => setTimeout(res, 40)); cb.click();
      }
    });
    // Click every button in the sim once (play/pause/reset/etc.).
    const buttons = page.locator('.sim-host button');
    r.buttons = await buttons.count();
    for (let k = 0; k < r.buttons; k++) {
      const b = buttons.nth(k);
      if (await b.isVisible().catch(() => false)) { await b.click({ timeout: 2000 }).catch(() => {}); await page.waitForTimeout(120); }
    }
    await page.waitForTimeout(400);
    // Drag on the first canvas to exercise pointer handlers.
    const canvas = page.locator('.sim-host canvas').first();
    if (await canvas.count()) {
      const box = await canvas.boundingBox();
      if (box) {
        const cx = box.x + box.width / 2, cy = box.y + box.height / 2;
        await page.mouse.move(cx, cy); await page.mouse.down(); await page.mouse.move(cx + 40, cy - 30, { steps: 5 }); await page.mouse.up();
      }
    }
    // Quiz: answer every question with its first option.
    const qs = page.locator('.quiz-q');
    r.quiz = await qs.count();
    for (let k = 0; k < r.quiz; k++) await qs.nth(k).locator('.quiz-opt').first().click();
    await page.waitForTimeout(300);
    if (screens) await page.screenshot({ path: path.join(screens, `${id}.png`), fullPage: true });
  } catch (e) {
    errors.push(`exception: ${e.message}`);
  }
  r.errors = [...new Set(errors)];
  r.ok = r.errors.length === 0 && r.canvases > 0;
  r.ms = Date.now() - t0;
  results.push(r);
  if (!asJson) console.log(`${r.ok ? 'PASS' : 'FAIL'}  ${id}  steps=${r.steps} canvases=${r.canvases} sliders=${r.sliders} buttons=${r.buttons} quiz=${r.quiz} (${r.ms} ms)`);
  if (!asJson) for (const e of r.errors) console.log(`      ↳ ${e}`);
}

await browser.close();
await server.close();
const failed = results.filter((r) => !r.ok);
if (asJson) console.log(JSON.stringify({ total: results.length, failed: failed.length, results }, null, 2));
else console.log(`\n${results.length - failed.length}/${results.length} lessons passed.`);
process.exit(failed.length ? 1 : 0);
