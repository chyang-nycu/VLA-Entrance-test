import puppeteer from 'puppeteer-core';
import { PDFDocument } from 'pdf-lib';
import fs from 'fs';

const N = 23;
const BASE = 'http://localhost:3035';
const WIDTH_PX = 980, HEIGHT_PX = 552; // slidev default canvas
const WIDTH_IN = WIDTH_PX / 96, HEIGHT_IN = HEIGHT_PX / 96;

const browser = await puppeteer.launch({
  executablePath: '/Applications/Brave Browser.app/Contents/MacOS/Brave Browser',
  headless: true,
  args: ['--no-sandbox', '--disable-gpu'],
});

const merged = await PDFDocument.create();

for (let i = 1; i <= N; i++) {
  const page = await browser.newPage();
  await page.emulateMediaFeatures([{ name: 'prefers-color-scheme', value: 'dark' }]);
  await page.setViewport({ width: WIDTH_PX, height: HEIGHT_PX, deviceScaleFactor: 2 });
  await page.goto(`${BASE}/${i}`, { waitUntil: 'networkidle0', timeout: 30000 });
  await page.waitForFunction(() => {
    const mmds = document.querySelectorAll('.mermaid, .slidev-mermaid-container');
    if (mmds.length === 0) return true;
    return Array.from(mmds).every(el => el.querySelector('svg'));
  }, { timeout: 8000 }).catch(() => {});
  await new Promise(r => setTimeout(r, 500));
  const pdfBytes = await page.pdf({
    width: `${WIDTH_IN}in`,
    height: `${HEIGHT_IN}in`,
    printBackground: true,
    margin: { top: 0, bottom: 0, left: 0, right: 0 },
  });
  const src = await PDFDocument.load(pdfBytes);
  const [copied] = await merged.copyPages(src, [0]);
  merged.addPage(copied);
  await page.close();
  console.log(`slide ${i}/${N} done`);
}

await browser.close();
const outBytes = await merged.save();
fs.writeFileSync('slide.pdf', outBytes);
console.log('wrote slide.pdf, pages:', merged.getPageCount());
