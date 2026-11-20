#pragma once
#include <Arduino.h>

/*
  Twenty2 / ESP32-S3 Wi-Fi dashboard add-on for professor's ECE 5 V4.0 sketch.
  Save this file as Twenty2_WiFi_Dashboard.h in the SAME Arduino sketch folder.
  Add #include "Twenty2_WiFi_Dashboard.h" near the top of your main .ino.

  Main sketch needs:
    setup(): DashboardBegin();   // after Calibrate() / ReadPotentiometers()
    loop():  DashboardUpdate();  // call every iteration, no delay()

  This header only READS the variables already defined in the original sketch.
  It does NOT drive the motors, change PID parameters, or sample the ADC.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

// CHANGE PASSWORD before using in a public/shared area (at least 8 characters).
static const char *TWENTY2_AP_NAME = "Twenty2-Dashboard";
static const char *TWENTY2_AP_PASSWORD = "Twenty2-2026";
static WebServer twenty2WebServer(80);

// These variables are declared by professor's V4.0 starter sketch.
extern int rawPResistorData[20];
extern int LDR[20];
extern float Mn[20], Mx[20];
extern float error, kP, kI, kD;
extern int SpRead, kPRead, kIRead, kDRead;
extern int Turn, M1SpeedtoMotor, M2SpeedtoMotor;

// No CDN, no internet access required. The HTML/JS is served by the ESP32.
static const char twenty2WebPage[] PROGMEM = R"TWENTY2(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta name="theme-color" content="#101827">
<title>Twenty2 Robot | Live Telemetry</title>
<style>
:root{font-family:system-ui,-apple-system,Segoe UI,sans-serif;color:#edf3ff;background:#101827}
*{box-sizing:border-box}
body{margin:0;padding:20px 12px 40px}
main{max-width:980px;margin:auto}
header{display:flex;justify-content:space-between;align-items:center;gap:12px;flex-wrap:wrap;margin-bottom:16px}
h1{margin:0;font-size:clamp(24px,4vw,32px);letter-spacing:-.03em}
sub,.muted{color:#9aaecb} .subtitle{font-size:13px;color:#9aaecb;margin:5px 0 0}
.status{border:1px solid #516178;border-radius:999px;padding:6px 12px;font-size:13px}
.status.ok{border-color:#1daa91;color:#6ce7c5}
.grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:10px;margin:14px 0}
.card{border:1px solid #27354c;background:#19243a;border-radius:14px;padding:15px}
.label{color:#aab9d3;font-size:12px;letter-spacing:.03em}
.value{font-size:28px;font-weight:730;line-height:1.4;font-variant-numeric:tabular-nums}
.panel{border:1px solid #27354c;background:#19243a;border-radius:14px;padding:16px;margin-top:12px}
h2{font-size:17px;margin:0 0 12px}
small{color:#93a8c7}
.sensors{display:grid;grid-template-columns:repeat(7,minmax(0,1fr));gap:8px}
.sensor{text-align:center;min-width:0}.sensor b{display:block;font-size:13px}
.track{margin:8px auto;height:116px;max-width:62px;border-radius:9px;background:#111b2d;position:relative;overflow:hidden;border:1px solid #3b4b62}
.bar{position:absolute;bottom:0;width:100%;height:0;background:linear-gradient(0deg,#2abf9e,#75e7d6);transition:height .2s ease}
.raw{font-size:12px;color:#dbe8fb;font-variant-numeric:tabular-nums}
.metric{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:12px}
.metric .value{font-size:23px}
#trend{display:block;width:100%;height:135px;border-radius:9px;background:#111b2d}
footer{color:#8da0bf;font-size:12px;line-height:1.65;padding:13px 3px}
@media(max-width:680px){.grid,.metric{grid-template-columns:repeat(2,minmax(0,1fr))}.card{padding:12px}.value{font-size:25px}.sensors{gap:4px}.panel{padding:12px}.sensor b{font-size:11px}.raw{font-size:10px}}
</style>
</head>
<body><main>
<header><div><h1>Twenty2 <span style="color:#70e1c5">Live Telemetry</span></h1>
<p class="subtitle">ESP32-S3 hotspot · read-only · refresh every 350 ms</p></div>
<span class="status" id="status">Connecting…</span></header>
<div class="grid">
  <div class="card"><div class="label">Line Error</div><div class="value" id="error">—</div></div>
  <div class="card"><div class="label">PID Turn</div><div class="value" id="turn">—</div></div>
  <div class="card"><div class="label">Speed Pot (0–100)</div><div class="value" id="speed">—</div></div>
  <div class="card"><div class="label">Uptime (s)</div><div class="value" id="uptime">—</div></div>
</div>
<section class="panel"><h2>Seven photoresistors <small>left → right, viewed from behind</small></h2>
<div class="sensors" id="sensors"></div>
<p class="subtitle">Bar = calibrated reading (approximately 0 white, 100 black). Number below = raw ADC (0–4095). Values outside 0–100 may occur because the original mapping is not clamped.</p></section>
<section class="panel"><h2>Error history <small>most recent ~21 seconds</small></h2><canvas id="trend"></canvas></section>
<section class="panel"><h2>PID & motor commands</h2><div class="metric">
  <div><div class="label">Kp</div><div class="value" id="kp">—</div></div>
  <div><div class="label">Ki</div><div class="value" id="ki">—</div></div>
  <div><div class="label">Kd</div><div class="value" id="kd">—</div></div>
  <div><div class="label">Max normalized LDR</div><div class="value" id="maxldr">—</div></div>
  <div><div class="label">M1 PWM variable</div><div class="value" id="m1">—</div></div>
  <div><div class="label">M2 PWM variable</div><div class="value" id="m2">—</div></div>
  <div><div class="label">P potentiometer</div><div class="value" id="pread">—</div></div>
  <div><div class="label">D potentiometer</div><div class="value" id="dread">—</div></div>
</div></section>
<footer>
PWM values are software commands, NOT measured motor RPM. In the original professor V4.0 code,
RunMotors() sends M2 to LEFT and M1 to RIGHT; verify the actual motor wiring.
If RunMotors() is commented out, these PWM variables may stay at zero. For tuning only;
Wi-Fi HTTP requests may introduce jitter in the control loop.</footer>
</main>
<script>
'use strict';
const el=id=>document.getElementById(id);
const fmt=(x,d=2)=>(typeof x==='number'&&Number.isFinite(x))?x.toFixed(d):'—';
let history=[];let busy=false;
el('sensors').innerHTML=Array.from({length:7},(_,i)=>
  '<div class="sensor"><b>LDR '+i+'</b><div class="track"><div class="bar" id="bar'+i+'"></div></div><b id="norm'+i+'">—</b><div class="raw" id="adc'+i+'">ADC —</div></div>'
).join('');
function trendDraw(){
 const c=el('trend');const dpr=window.devicePixelRatio||1;
 const w=Math.max(200,Math.round(c.clientWidth)),h=135;
 if(c.width!==Math.round(w*dpr)||c.height!==Math.round(h*dpr)){
   c.width=Math.round(w*dpr);c.height=Math.round(h*dpr);
 }
 const ctx=c.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);
 ctx.clearRect(0,0,w,h);
 for(let v=-3;v<=3;v+=1){let y=h/2-v*(h-26)/6;
   ctx.beginPath();ctx.strokeStyle=v===0?'#6982a4':'#263952';ctx.lineWidth=v===0?1.5:1;
   ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke();
 }
 ctx.strokeStyle='#70e1c5';ctx.lineWidth=2;ctx.beginPath();
 let started=false;
 history.forEach((v,i)=>{if(typeof v!=='number'){started=false;return;}
   const x=(i/59)*w,y=h/2-Math.max(-3,Math.min(3,v))*(h-26)/6;
   if(!started){ctx.moveTo(x,y);started=true;}else{ctx.lineTo(x,y);}
 });ctx.stroke();
}
async function refresh(){
 if(busy)return;busy=true;
 try{
  const r=await fetch('/data',{cache:'no-store'});if(!r.ok)throw new Error('HTTP '+r.status);
  const d=await r.json();
  el('status').textContent='● LIVE';el('status').className='status ok';
  el('error').textContent=fmt(d.error,3);
  el('turn').textContent=String(d.turn);
  el('speed').textContent=String(d.speed);
  el('uptime').textContent=fmt(d.uptime/1000,0);
  el('kp').textContent=fmt(d.kp,3);el('ki').textContent=fmt(d.ki,4);
  el('kd').textContent=fmt(d.kd,3);el('maxldr').textContent=String(Math.max(...d.ldr));
  el('m1').textContent=String(d.m1);el('m2').textContent=String(d.m2);
  el('pread').textContent=String(d.pread);el('dread').textContent=String(d.dread);
  d.ldr.forEach((v,i)=>{
    const percent=Math.max(0,Math.min(100,v));
    el('bar'+i).style.height=percent+'%';
    el('norm'+i).textContent=String(v);
    el('adc'+i).textContent='ADC '+String(d.adc[i]);
  });
  history.push(d.error);if(history.length>60)history.shift();trendDraw();
 }catch(e){el('status').textContent='● OFFLINE';el('status').className='status';}
 finally{busy=false;}
}
window.addEventListener('resize',trendDraw);
trendDraw();refresh();setInterval(refresh,350);
</script>
</body></html>
)TWENTY2";

static String twenty2JsonNumber(float value, unsigned char places=3) {
  if (!isfinite(value)) return "null";
  return String(value, places);
}

static void twenty2SendData() {
  String json;
  json.reserve(600);
  json += "{\"uptime\":"; json += String(millis());
  json += ",\"adc\":[";
  for (int i=0; i<7; ++i) { if(i)json+=','; json+=String(rawPResistorData[i]); }
  json += "],\"ldr\":[";
  for (int i=0; i<7; ++i) { if(i)json+=','; json+=String(LDR[i]); }
  json += "],\"white\":[";
  for (int i=0; i<7; ++i) { if(i)json+=','; json+=twenty2JsonNumber(Mn[i],0); }
  json += "],\"black\":[";
  for (int i=0; i<7; ++i) { if(i)json+=','; json+=twenty2JsonNumber(Mx[i],0); }
  json += "],\"error\":";json+=twenty2JsonNumber(error,4);
  json += ",\"turn\":";json+=String(Turn);
  json += ",\"speed\":";json+=String(SpRead);
  json += ",\"kp\":";json+=twenty2JsonNumber(kP,4);
  json += ",\"ki\":";json+=twenty2JsonNumber(kI,5);
  json += ",\"kd\":";json+=twenty2JsonNumber(kD,4);
  json += ",\"pread\":";json+=String(kPRead);
  json += ",\"iread\":";json+=String(kIRead);
  json += ",\"dread\":";json+=String(kDRead);
  json += ",\"m1\":";json+=String(M1SpeedtoMotor);
  json += ",\"m2\":";json+=String(M2SpeedtoMotor);
  json += '}';
  twenty2WebServer.sendHeader("Cache-Control", "no-store");
  twenty2WebServer.send(200, "application/json", json);
}

void DashboardBegin() {
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(TWENTY2_AP_NAME, TWENTY2_AP_PASSWORD)) {
    Serial.println("[Dashboard] Failed to create Wi-Fi hotspot!");
    return;
  }
  twenty2WebServer.on("/", HTTP_GET, []() {
    twenty2WebServer.send_P(200,"text/html; charset=utf-8",twenty2WebPage);
  });
  twenty2WebServer.on("/data", HTTP_GET, twenty2SendData);
  twenty2WebServer.onNotFound([]() {
    twenty2WebServer.send(404,"text/plain","Not found; open / or /data");
  });
  twenty2WebServer.begin();
  Serial.print("[Dashboard] Connect Wi-Fi: "); Serial.println(TWENTY2_AP_NAME);
  Serial.print("[Dashboard] Open: http://"); Serial.println(WiFi.softAPIP());
}

void DashboardUpdate() {
  twenty2WebServer.handleClient();
}
