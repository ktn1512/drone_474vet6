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
.err{color:#ffa9ac}.success{color:#9de8b2}#message,#radioMessage{min-height:20px}
@media(max-width:480px){body{padding:12px}.pidrow{gap:5px}.pidrow input{padding:8px 5px}.pidrow div:first-child{font-size:12px}}
</style></head><body>
<h1>Drone Control Dashboard</h1>
<p>ESP32 Wi-Fi router (STA) → E220 UART → STM32G474. Thử nghiệm trên bàn, <b>tháo cánh quạt trước khi kích hoạt ga.</b></p>
<div class="row"><span class="pill" id="radio">E220: chưa có phản hồi</span><span class="pill" id="armState">DISARMED</span><span class="pill" id="operation">Không có yêu cầu PID</span></div>
<section class="panel"><h2>Điều khiển ga ESC</h2>
<div class="row"><div><div class="big"><span id="throttleText">1000</span> <small>µs</small></div><small>Đặt trong khoảng 1000–1700 µs</small></div>
<div class="row"><button id="armBtn" disabled>ARM (1000 µs)</button><button id="stopBtn" class="stop">DISARM / STOP</button></div></div>
<input id="throttleRange" aria-label="Đặt mức ga" type="range" min="1000" max="1700" value="1000" step="1" disabled>
<div class="row"><small>1000 µs</small><small>1700 µs</small></div>
<small>Muốn ARM phải đặt ga bằng 1000 µs và có phản hồi mới từ drone. Đóng trang / mất kết nối web sẽ tự DISARM trên ESP32.</small></section>
<section class="panel"><h2>Cấu hình 5 bộ PID</h2>
<div class="pidrow head"><div>Bộ điều khiển</div><div>Kp</div><div>Ki</div><div>Kd</div></div>
<div id="pidFields"></div><div class="row"><button id="applyBtn">Gửi PID tới drone</button><button id="readBtn" class="alt">Đọc PID từ drone</button></div>
<p id="message"></p><small>PID chỉ được cập nhật khi drone đang DISARM. Mỗi bộ PID được gửi riêng và chờ ACK, tránh ghi nhầm khi truyền lỗi.</small>
</section>
<section class="panel"><h2>Phản hồi từ STM32</h2><div class="statusgrid">
<div class="metric">Ga thực nhận<strong id="rxThr">--</strong></div>
<div class="metric">ARM trên drone<strong id="rxArm">--</strong></div>
<div class="metric">Roll<strong id="rxRoll">--</strong></div>
<div class="metric">Pitch<strong id="rxPitch">--</strong></div>
<div class="metric">Yaw<strong id="rxYaw">--</strong></div>
<div class="metric">M1–M4 (µs)<strong id="rxMotors" style="font-size:14px">--</strong></div>
</div></section>
<section class="panel"><h2>Gửi nhận E220</h2><div class="statusgrid">
<div class="metric">Gói gửi qua UART<strong id="radioTx">0</strong></div>
<div class="metric">Dòng đã nhận<strong id="radioRx">0</strong></div>
<div class="metric">Lỗi TX / RX<strong id="radioErrors">0 / 0</strong></div>
<div class="metric">Lần nhận gần nhất<strong id="radioAge">--</strong></div>
</div>
<p id="radioReady">Đang đọc trạng thái E220...</p>
<div class="metric">RX gần nhất<strong id="radioLastRx" class="rawMessage">--</strong></div>
<div class="metric" style="margin-top:10px">TX gần nhất<strong id="radioLastTx" class="rawMessage">--</strong></div>
<div class="row" style="margin-top:14px"><button id="radioTestBtn" class="alt" disabled>Gửi gói test E220</button></div>
<p id="radioMessage" role="status"></p>
<small>Gói gửi qua UART chưa xác nhận STM32 đã nhận. Các dòng test như “STM32 → ESP32” cho biết E220 có dữ liệu; cần gói status drone hợp lệ để hiển thị góc, ga và cho phép ARM.</small>
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
let isArmed=false, editing=false, pidEditing=false, lastPIDKey='';
const slider=el('throttleRange');
function note(t,error=false){el('message').textContent=t;el('message').className=error?'err':'success';}
function radioNote(t,error=false){el('radioMessage').textContent=t;el('radioMessage').className=error?'err':'success';}
function disableRemoteControls(){el('armBtn').disabled=true;slider.disabled=true;el('radioTestBtn').disabled=true;}
async function post(url,values={}){
 const res=await fetch(url,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams(values)});
 const data=await res.json();if(!res.ok||!data.ok)throw new Error(data.error||`HTTP ${res.status}`);return data;
}
let timer=null;
slider.addEventListener('input',()=>{
 el('throttleText').textContent=slider.value;
 clearTimeout(timer);
 timer=setTimeout(async()=>{try{await post('/api/throttle',{throttle:slider.value});}catch(e){note(e.message,true);}},100);
});
slider.addEventListener('change',async()=>{clearTimeout(timer);try{await post('/api/throttle',{throttle:slider.value})}catch(e){note(e.message,true)}});
el('armBtn').onclick=async()=>{try{await post('/api/arm',{arm:1});note('Đã yêu cầu ARM tại 1000 µs')}catch(e){note(e.message,true)}};
el('stopBtn').onclick=async()=>{clearTimeout(timer);slider.value=1000;el('throttleText').textContent='1000';try{await post('/api/arm',{arm:0});note('Đã gửi DISARM')}catch(e){note(e.message,true)}};
el('applyBtn').onclick=async()=>{
 const values={};for(let i=0;i<5;i++)for(const key of ['kp','ki','kd'])values[`${i}_${key}`]=el(`pid_${i}_${key}`).value;
 try{await post('/api/pids',values);note('Đang gửi từng bộ PID, chờ drone xác nhận...')}catch(e){note(e.message,true)}
};
el('readBtn').onclick=async()=>{try{await post('/api/readpid');note('Đang đọc 5 bộ PID từ drone...')}catch(e){note(e.message,true)}};
el('radioTestBtn').onclick=async()=>{
 try{await post('/api/radio/test');radioNote('Đã xếp hàng gói test E220. Xem TX để kiểm tra gói đã gửi qua UART. Chưa có xác nhận nhận gói từ STM32.');await refresh()}
 catch(e){radioNote(e.message,true)}
};
async function refresh(){
 try{
  const r=await fetch('/api/state',{cache:'no-store'});if(!r.ok)throw new Error(`HTTP ${r.status}`);
  const d=await r.json();isArmed=d.armed;
  el('armState').textContent=d.armed?'ARMED':'DISARMED';
  const radio=d.radio||{};
  const recentRx=radio.rxAgeMs!=null&&radio.rxAgeMs<=4500;
  el('radio').textContent=d.link?'E220: có status drone':recentRx?'E220: có dữ liệu, chờ status drone':'E220: chưa có phản hồi';
  el('radioTx').textContent=radio.txCount??0;el('radioRx').textContent=radio.rxCount??0;
  el('radioErrors').textContent=(radio.txErrors??0)+' / '+(radio.rxErrors??0);
  el('radioAge').textContent=radio.rxAgeMs==null?'--':(radio.rxAgeMs/1000).toFixed(1)+' giây trước';
  el('radioLastRx').textContent=radio.lastRx||'--';el('radioLastTx').textContent=radio.lastTx||'--';
  el('radioReady').textContent=radio.testPending?'Đang chờ gửi gói test E220...':radio.auxConnected?(radio.auxReady?(radio.ready?'Sẵn sàng gửi UART E220.':'Chờ lượt gửi tiếp theo.'):'E220 đang bận (AUX LOW).'):(radio.ready?'Sẵn sàng gửi UART E220; AUX không nối.':'Chờ lượt gửi tiếp theo; AUX không nối.');
  el('radioTestBtn').disabled=!!d.armed||!!d.pidBusy||!!radio.testPending;
  el('operation').textContent=d.operation;
  slider.disabled=!d.armed||!d.link||!d.status||!d.status.armed;el('armBtn').disabled=d.armed||!d.link||!!d.pidBusy;
  if(!d.armed){slider.value=1000;el('throttleText').textContent='1000'}
  if(d.armed&&document.activeElement!==slider){slider.value=d.throttle;el('throttleText').textContent=d.throttle}
  if(d.status){const s=d.status;
   el('rxThr').textContent=s.throttle+' µs';el('rxArm').textContent=s.armed?'ARMED':'DISARMED';
   el('rxRoll').textContent=s.roll.toFixed(1)+'°';el('rxPitch').textContent=s.pitch.toFixed(1)+'°';
   el('rxYaw').textContent=s.yaw.toFixed(1)+'°';el('rxMotors').textContent=s.motors.join(' / ');
  }
  // Don't overwrite local edits when reading live state. Sync only when read command finishes or first page load.
  const pidKey=JSON.stringify(d.pids);
  if((!pidEditing || d.operation.startsWith('Đã đọc PID')) && lastPIDKey!==pidKey){
    if(!pidEditing||d.operation.startsWith('Đã đọc PID')){
      d.pids.forEach((p,i)=>['kp','ki','kd'].forEach(k=>el(`pid_${i}_${k}`).value=p[k]));
      lastPIDKey=pidKey;
    }
  }
  if(d.operation.startsWith('Đã đọc PID'))pidEditing=false;
 }catch(e){el('radio').textContent='ESP32: lỗi kết nối';disableRemoteControls()}
}
fields.addEventListener('input',()=>{pidEditing=true});
refresh();setInterval(refresh,500);
</script></body></html>
)WEB";
