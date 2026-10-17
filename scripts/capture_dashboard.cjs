// Optional screenshot/visual QA helper. Requires Playwright + installed Chromium.
const {chromium}=require('playwright');
(async()=>{
 const browser=await chromium.launch({headless:true});
 const page=await browser.newPage({viewport:{width:1440,height:1180}});
 const errors=[];page.on('pageerror',e=>errors.push(String(e)));
 await page.goto('http://127.0.0.1:5173');
 await page.waitForFunction(()=>document.body.innerText.includes('KINEMATIC_HARNESS'));
 await page.waitForTimeout(3000);
 await page.screenshot({path:'docs/dashboard.png',fullPage:true});
 if(errors.length)throw Error(errors.join('\n'));
 await browser.close();
})();
