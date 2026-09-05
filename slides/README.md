# G1 MuJoCo Entrance-Test — Slidev Deck

A comprehensive slide deck (built with [Slidev](https://sli.dev), using the
[`slidev-theme-academic`](https://github.com/alexanderdavide/slidev-theme-academic)
theme) introducing the Unitree G1 MuJoCo entrance-test project to a
professor/academic audience: environment constraints, the missing-gripper
feasibility audit, the two failed control architectures before the one that
worked, Task 1's full pipeline and honestly-reported limitations, the VLA
demonstration-data pipeline's three replay-fidelity iterations, and Task 2's
language-conditioned two-object selection evaluation.

All content, figures, and numbers are sourced directly from
`~/Documents/Robotics/submission/entrance_test_report.md`,
`HANDOFF.md`, and `docs/work_log.md`; images in `public/` are copies of
real evidence stills from `~/Documents/Robotics/artifacts/`.

## Run locally (live-reload editor)

```bash
npm install
npm run dev       # opens http://localhost:3030
```

## Build a static site

```bash
npm run build      # outputs to dist/
```

## Export to PDF

`npm run export` (Slidev's built-in exporter) requires `playwright-chromium`,
whose bundled Chromium build does not support macOS 12 (Monterey) — it fails
to install on this host. Slidev's browser-based `/export` route also doesn't
work headlessly here: its continuous-scroll preview lazy-mounts each slide
(mermaid diagrams included) via an intersection observer, so a scriptless
headless print only captures whichever slides happened to scroll into view.

Instead, use the included Puppeteer script, which drives the **already-running
Brave browser binary** (no bundled-browser download) to print each slide route
individually — waiting for every mermaid diagram to actually finish
rendering — then merges the 23 single-page PDFs into one:

```bash
npm run dev -- --port 3035   # start the dev server first, in another terminal
npm run export:pdf           # writes slide.pdf (23 pages, 16:9)
```

`export-pdf.mjs` forces `prefers-color-scheme: dark` before printing each
page — a fresh headless Chromium profile otherwise defaults to light mode,
which made the cover slide's white-on-dark title render as invisible
white-on-white in early attempts. If you change the deck's theme away from a
dark default, revisit that line.

## Present with speaker notes

```bash
npm run dev
# then open http://localhost:3030/presenter
```
