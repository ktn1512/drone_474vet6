#pragma once
static const char INDEX_HTML[] PROGMEM = R"WEB(
<!doctype html><html lang="vi"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Drone PID / LoRa E220</title>
<style>
:root{color-scheme:dark;font-family:system-ui,Segoe UI,Arial,sans-serif;background:#10151c;color:#edf3fc}
*{box-sizing:border-box}body{margin:0 auto;padding:20px 14px 40px;max-width:940px}
h1{font-size:25px;margin:8px 0 4px}h2{font-size:17px;margin:0 0 16px}
p,small{color:#aab7c8}p{line-height:1.5;margin:6px 0 18px}
.panel{background:#1b2531;border:1px solid #2c3b4b;border-radius:14px;padding:18px;margin-top:16px}
.row{display:flex;gap:12px;flex-wrap:wrap;align-items:center;justify-content:space-between}
.pill{border:1px solid #52647a;border-radius:30px;padding:6px 12px;font-size:13px}
button{background:#2b85e4;color:white;border:0;border-radius:8px;padding:11px 16px;font-weight:650;cursor:pointer}
button.alt{background:#334559}button.stop{background:#b52937}button:disabled{opacity:.4;cursor:default}
#throttleRange{width:100%;accent-color:#56b1fa;touch-action:pan-y}
.big{font-size:36px;font-weight:750;font-variant-numeric:tabular-nums}
.pidrow{display:grid;grid-template-columns:1.5fr repeat(3,minmax(0,1fr));gap:9px;align-items:center;margin:10px 0}
.pidrow.head{font-size:12px;color:#9eafc4}
.pidrow input{min-width:0;width:100%;background:#0c131c;color:#fff;border:1px solid #516078;border-radius:6px;padding:10px}
.statusgrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:10px}
.metric{background:#111a25;border-radius:8px;padding:12px}
.metric strong{display:block;font-size:19px;font-variant-numeric:tabular-nums;margin-top:5px}
.metric strong.rawMessage{font:14px ui-monospace,Consolas,monospace;white-space:pre-wrap;overflow-wrap:anywhere;word-break:break-word;line-height:1.5}
.err{color:#ffa9ac}.success{color:#9de8b2}#message{min-height:20px}
@media(max-width:480px){body{padding:12px}.pidrow{gap:5px}.pidrow input{padding:8px 5px}.pidrow div:first-child{font-size:12px}}
</style></head><body>
<h1>Drone Control Dashboard</h1>
<p>ESP32 Wi-Fi (STA) → E220 UART → STM32G474. Thử trên bàn, tháo cánh quạt trước khi tăng xung ESC.</p>
<div class="row"><span class="pill" id="radio">E220: chưa có phản hồi</span><span class="pill" id="operation">Không có yêu cầu PID</span></div>
<section class="panel"><h2>Điều khiển ga trực tiếp</h2>
<div class="row"><div><div class="big"><span id="throttleText">1000</span> <small>µs</small></div><small>1000–1700 µs</small></div>
<button id="minBtn" class="stop">Đưa ga về 1000 µs</button></div>
<input id="throttleRange" aria-label="Đặt mức ga" type="range" min="1000" max="1700" value="1000" step="1">
<div class="row"><small>1000 µs</small><small>1700 µs</small></div>
<p id="throttleMessage">Gửi trực tiếp T và nhận S phản hồi. Mất tín hiệu sẽ tự về 1000 µs.</p>
</section>
<section class="panel"><h2>Cấu hình 5 bộ PID</h2>
<div class="pidrow head"><div>Bộ điều khiển</div><div>Kp</div><div>Ki</div><div>Kd</div></div>
<div id="pidFields"></div><div class="row"><button id="applyBtn">Gửi PID tới drone</button><button id="readBtn" class="alt">Đọc PID từ drone</button></div>
<p id="message"></p><small>Mỗi lần chỉ gửi một P hoặc G rồi đợi phản hồi A/N/R. ESP32 chèn nhịp ga T giữa các lượt PID, không chèn khi đang chờ ACK. Mất liên lạc vẫn đưa ga về 1000 µs. Chỉ thử với cánh quạt đã tháo.</small>
</section>
<section class="panel"><h2>Phản hồi từ STM32</h2><div class="statusgrid">
<div class="metric">Ga STM32 nhận<strong id="rxThr">--</strong></div>
<div class="metric">Roll<strong id="rxRoll">--</strong></div>
<div class="metric">Pitch<strong id="rxPitch">--</strong></div>
<div class="metric">Yaw<strong id="rxYaw">--</strong></div>
<div class="metric">M1–M4 (µs)<strong id="rxMotors" style="font-size:14px">--</strong></div>
</div></section>
<section class="panel"><h2>Gửi nhận E220</h2><div class="statusgrid">
<div class="metric">Gói TX UART<strong id="radioTx">0</strong></div>
<div class="metric">Dòng RX<strong id="radioRx">0</strong></div>
<div class="metric">Lỗi TX / RX<strong id="radioErrors">0 / 0</strong></div>
<div class="metric">Lần RX gần nhất<strong id="radioAge">--</strong></div>
</div><p id="radioReady">Đang đọc trạng thái E220...</p>
<div class="metric">RX gần nhất<strong id="radioLastRx" class="rawMessage">--</strong></div>
<div class="metric" style="margin-top:10px">TX gần nhất<strong id="radioLastTx" class="rawMessage">--</strong></div>
<small>TX qua UART chưa chắc đã tới STM32. Gói S hợp lệ xác nhận đường STM32 → ESP32 hoạt động.</small>
</section>
<script>
const names=['Angle Roll','Angle Pitch','Rate Roll','Rate Pitch','Rate Yaw'];
const el=id=>document.getElementById(id);
const fields=el('pidFields');
names.forEach((name,i)=>{
 const row=document.createElement('div');row.className='pidrow';
 const label=document.createElement('div');label.textContent=name;row.appendChild(label);
 ['kp','ki','kd'].forEach(k=>{
  const input=document.createElement('input');input.id=`pid_${i}_${k}`;input.type='number';input.min='0';input.max='100';input.step='0.001';input.value='0';row.appendChild(input);
 });fields.appendChild(row);
});
const slider=el('throttleRange');
let pidEditing=false,lastPIDKey='',timer=null,lastOp='';
function note(t,error=false){el('message').textContent=t;el('message').className=error?'err':'success';}
async function post(url,values={}){
 const res=await fetch(url,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams(values)});
 const data=await res.json();if(!res.ok||!data.ok)throw new Error(data.error||`HTTP ${res.status}`);return data;
}
async function setThrottle(value){
 try{await post('/api/throttle',{throttle:value});}catch(e){note(e.message,true);}
}
slider.addEventListener('input',()=>{
 el('throttleText').textContent=slider.value;
 clearTimeout(timer);
 timer=setTimeout(()=>setThrottle(slider.value),100);
});
slider.addEventListener('change',()=>{clearTimeout(timer);setThrottle(slider.value);});
el('minBtn').onclick=()=>{
 clearTimeout(timer);slider.value='1000';el('throttleText').textContent='1000';setThrottle(1000);
};
el('applyBtn').onclick=async()=>{
 const values={};for(let i=0;i<5;i++)for(const key of ['kp','ki','kd'])values[`${i}_${key}`]=el(`pid_${i}_${key}`).value;
 try{await post('/api/pids',values);note('Đang gửi PID và chờ phản hồi A/N...');}catch(e){note(e.message,true);}
};
el('readBtn').onclick=async()=>{
 try{await post('/api/readpid');note('Đang đọc 5 bộ PID (G → R)...');}catch(e){note(e.message,true);}
};
async function refresh(){
 try{
  const r=await fetch('/api/state',{cache:'no-store'});if(!r.ok)throw new Error(`HTTP ${r.status}`);
  const d=await r.json(),radio=d.radio||{};
  const recentRx=radio.rxAgeMs!=null&&radio.rxAgeMs<=4500;
  el('radio').textContent=d.link?'E220: có status STM32':recentRx?'E220: có RX, chờ status':'E220: chưa có status';
  el('radioTx').textContent=radio.txCount??0;el('radioRx').textContent=radio.rxCount??0;
  el('radioErrors').textContent=(radio.txErrors??0)+' / '+(radio.rxErrors??0);
  el('radioAge').textContent=radio.rxAgeMs==null?'--':(radio.rxAgeMs/1000).toFixed(1)+' giây trước';
  el('radioLastRx').textContent=radio.lastRx||'--';el('radioLastTx').textContent=radio.lastTx||'--';
  el('radioReady').textContent=radio.auxConnected?(radio.auxReady?'AUX HIGH; UART có thể gửi.':'E220 đang bận (AUX LOW).'):'AUX không nối; truyền theo khoảng nghỉ cố định.';
  el('operation').textContent=d.operation;
  if(d.operation!==lastOp){
    if(d.operation.startsWith('Đã đọc PID')||d.operation.startsWith('Đã gửi PID')){
      pidEditing=false;note(d.operation);
    }else if(d.operation.startsWith('Lỗi')||d.operation.startsWith('STM32 từ chối'))note(d.operation,true);
    lastOp=d.operation;
  }
  el('throttleMessage').textContent=d.link?'Đã có phản hồi status S hợp lệ.':`Chưa có status S mới. ${d.statusDiagnostic||''}`;
  el('throttleMessage').className=d.link?'success':'err';
  if(document.activeElement!==slider){slider.value=d.throttle;el('throttleText').textContent=d.throttle;}
  if(d.status){const s=d.status;
   el('rxThr').textContent=s.throttle+' µs';
   el('rxRoll').textContent=s.roll.toFixed(1)+'°';el('rxPitch').textContent=s.pitch.toFixed(1)+'°';
   el('rxYaw').textContent=s.yaw.toFixed(1)+'°';el('rxMotors').textContent=s.motors.join(' / ');
  }
  const pidKey=JSON.stringify(d.pids);
  if(!pidEditing&&lastPIDKey!==pidKey){
    d.pids.forEach((p,i)=>['kp','ki','kd'].forEach(k=>el(`pid_${i}_${k}`).value=p[k]));
    lastPIDKey=pidKey;
  }
 }catch(e){el('radio').textContent='ESP32: lỗi kết nối';el('throttleMessage').textContent='Không tải được trạng thái: '+e.message;el('throttleMessage').className='err';}
}
fields.addEventListener('input',()=>{pidEditing=true;});
refresh();setInterval(refresh,500);
</script></body></html>
)WEB";
