/* Existing dashboard tab; no motion on navigation, selection or camera polls. */
"use strict";
const RT = {rows:[],selected:null,detail:null,seq:0,busy:false,frame:null,bound:false,live:null,alignment:{running:false},stopRequested:false,beatBusy:false,starting:false,range:null,rangeDirty:false,rangeSaving:false,rangePreview:false,rangeFrame:null};
function rtLabel(status) { return status === "verified" ? "검증 통과" : status === "not_computed" ? "계산 대기" : "검토용 · 검증 미통과"; }
async function rtFetch(path,options) {
  const r=await fetch("/kuavo/retargeting"+path,options);
  const d=await r.json();
  if(!r.ok) throw new Error(d.detail || "요청 실패");
  return d;
}
function rtRenderList() {
  const q=$("rt-search").value.toLowerCase();
  const rows=RT.rows.filter(r=>(r.label+" "+r.id).toLowerCase().includes(q));
  $("rt-count").textContent=String(RT.rows.length);
  $("rt-list").innerHTML=rows.map(r=>`<button type="button" class="rt-episode" data-rt-id="${esc(r.id)}" aria-pressed="${RT.selected===r.id}"><strong>${esc(r.label)}</strong><small>${esc(rtLabel(r.status))} · ${r.has_video?"4-window 있음":"영상 대기"}</small></button>`).join("");
  $("rt-list-msg").textContent=rows.length?"":"표시할 episode가 없습니다.";
  $("rt-list").querySelectorAll("[data-rt-id]").forEach(b=>b.addEventListener("click",()=>rtSelect(b.dataset.rtId)));
}
async function rtRefresh() {
  try { const d=await rtFetch("");RT.rows=d.episodes;rtRenderList(); }
  catch(e) { $("rt-list-msg").textContent=e.message; }
}
function rtClearCamera() {
  RT.frame=null;RT.live=null;$("rt-head").removeAttribute("src");$("rt-projection").innerHTML="";
  $("rt-camera-empty").hidden=false;$("rt-camera-empty").textContent="head 카메라 연결 중…";
  $("rt-check-robot").checked=false;$("rt-check-boxes").checked=false;$("rt-validation-override").checked=false;
  $("rt-check-robot").disabled=true;$("rt-check-boxes").disabled=true;
  $("rt-align-start").disabled=true;
  $("rt-base-error").textContent="로봇 기준 목표까지: -";$("rt-replay").disabled=true;
}
async function rtSelect(id) {
  await rtLeave();
  RT.selected=id;RT.detail=null;const seq=++RT.seq;
  RT.range=null;RT.rangePreview=false;RT.rangeDirty=false;RT.rangeSaving=false;$("rt-range").disabled=true;
  $("rt-empty").hidden=true;$("rt-work").hidden=false;$("rt-title").textContent="불러오는 중…";
  $("rt-command-msg").textContent="";$("rt-video-msg").textContent="";rtClearCamera();rtRenderList();
  const video=$("rt-video");video.pause();video.removeAttribute("src");video.load();video.hidden=true;
  try {
    const d=await rtFetch("/episode/"+encodeURIComponent(id)+"?sid="+encodeURIComponent(SID));
    if(seq!==RT.seq) return;
    RT.detail=d;$("rt-title").textContent=d.label+" · 4-window";
    rtRangeSetup(d);
    $("rt-status").textContent=rtLabel(d.status);$("rt-status").className="rt-badge "+(d.status==="verified"?"ok":"warn");
    if(d.has_video) { video.src="/kuavo/retargeting/episode/"+encodeURIComponent(id)+"/video";video.hidden=false; }
    $("rt-video-msg").textContent=d.has_video?(d.review_duration_s?`검토 영상 ${d.review_duration_s}초 · 실행 궤적은 별도 검증`:
      "standard 4-window 영상"):"이 episode의 standard 4-window 영상은 아직 생성되지 않았습니다.";
    rtInitialLabel(d);
    $("rt-blockers").textContent=d.driver_reason;
    if(seq!==RT.seq) return;
    await rtRangeRestore();
    if(seq!==RT.seq) return;
    await pullRetargeting();
  } catch(e) { if(seq===RT.seq) {$("rt-title").textContent="episode를 불러오지 못했습니다";$("rt-video-msg").textContent=e.message;} }
}
function rtDraw(live) {
  const ov=$("rt-projection");ov.setAttribute("viewBox",`0 0 ${live.width||640} ${live.height||480}`);
  if(!(live.projection_ok ?? live.ok) || !RT.camera?.ok || !$("rt-overlay").checked) {ov.innerHTML="";return;}
  ov.innerHTML=(live.boxes||[]).map((b,i)=>{
    if(!b.visible) return "";
    const color=i===0?"#ffd54a":"#39d9ee";
    const edges=b.edges.map(([a,c])=>`<line x1="${b.corners[a][0]}" y1="${b.corners[a][1]}" x2="${b.corners[c][0]}" y2="${b.corners[c][1]}" stroke="${color}"/>`).join("");
    const c=b.corners[0];return edges+`<text x="${c[0]+5}" y="${c[1]-8}">${esc(b.id.includes("110")?"목표 110·111":"목표 100·101")}</text>`;
  }).join("");
}
async function pullRetargeting() {
  if(S.view!=="retargeting" || !RT.detail || RT.busy) return;
  RT.busy=true;const id=RT.selected,seq=RT.seq;
  try {
    const d=await rtFetch("/episode/"+encodeURIComponent(id)+"/state?sid="+encodeURIComponent(SID));
    if(seq!==RT.seq || RT.rangeSaving || RT.rangeDirty) return;
    if(d.fingerprint!==RT.detail.fingerprint) { rtSelect(id);return; }
    const l=d.live;RT.live=l;
    const camera=d.camera||{};RT.camera=camera;
    if(camera.url && RT.frame!==camera.frame_id) {
      const img=$("rt-head");
      RT.frame=camera.frame_id;
      img.onload=()=>{if(seq===RT.seq) rtDraw(RT.live);};
      img.src=camera.url+"?seq="+encodeURIComponent(camera.seq);
      $("rt-camera-empty").hidden=true;
    } else rtDraw(l);
    if(!(l.projection_ok ?? l.ok) || !camera.ok) $("rt-projection").innerHTML="";
    $("rt-live-msg").textContent=(camera.ok?"대시보드 최신 head 영상 · ":"head 영상 수신 대기 · ")+(l.message||"태그 관측 대기")+(l.ok&&Number.isFinite(l.age_s)?` · 태그 관측 ${l.age_s.toFixed(2)}초 전`:"");
    $("rt-validation-override").checked=!!d.checks.validation_override;
    $("rt-validation-override").disabled=!!d.alignment?.running||d.validation_status==="verified";
    $("rt-validation-approval-msg").textContent=d.validation_status==="verified"?"standard 검증 통과 · 별도 승인 불필요":(d.checks.validation_override?"현재 구간의 검증 미통과 실행 승인됨":"승인은 현재 episode·선택 구간에만 적용됩니다.");
    const stationary=d.stationary;
    $("rt-stationary-status").textContent=`자동 정지 기준: 최대 ${stationary?.threshold_deg_s||3}°/s 이하 · 실측 ${stationary?.available?stationary.maximum_deg_s.toFixed(2)+"°/s ("+stationary.joint+")":"대기"}`;
    $("rt-stationary-confirm").disabled=!!d.alignment?.running;
    $("rt-robot-approval-label").textContent=d.checks.robot_manual?"첫 state 도착 완료 (수동 승인)":"첫 state 도착 완료";
    $("rt-check-robot").checked=!!d.checks.robot;$("rt-check-boxes").checked=!!d.checks.boxes;
    $("rt-check-robot").disabled=!!d.alignment?.running||!l.ok;$("rt-check-boxes").disabled=!l.ok||!d.checks.robot;
    RT.alignment=d.alignment||{running:false};
    const a=RT.alignment;
    $("rt-align-start").disabled=RT.starting||a.running||!d.align_available||RT.rangeDirty||RT.rangeSaving;
    $("rt-range").disabled=!RT.range||a.running||RT.starting||RT.rangeSaving;
    $("rt-align-start").textContent=a.running?(a.kind==="replay"?"replay 중…":"정렬 중…"):"▶ 시작";
    $("rt-align-status").textContent=a.running?((a.mine?"":"다른 페이지 · ")+a.stage):
      (a.error||(a.kind==="replay"&&a.id===id?a.stage:((a.success&&a.id===id&&d.checks.robot)?"첫 state 도착·정지 확인됨 · head 제외":(d.align_reason||"시작을 누르면 베이스 정렬과 첫 state 이동을 함께 진행합니다."))));
    const err=a.running&&a.kind==="align"&&a.vision_locked&&a.base_error?{forward_m:a.base_error[0],left_m:a.base_error[1],yaw_deg:a.base_error[2]*180/Math.PI}:l.robot_error;
    $("rt-base-error").textContent=err?`로봇 기준 목표까지: ${err.forward_m>=0?"앞":"뒤"} ${Math.abs(err.forward_m*100).toFixed(1)}cm · ${err.left_m>=0?"왼쪽":"오른쪽"} ${Math.abs(err.left_m*100).toFixed(1)}cm · ${err.yaw_deg>=0?"좌회전":"우회전"} ${Math.abs(err.yaw_deg).toFixed(1)}°`:
      "로봇 기준 목표까지: -";
    $("rt-prep-base-error").textContent=$("rt-base-error").textContent;
    $("rt-blockers").textContent=d.blockers.join("\n");$("rt-blockers").style.whiteSpace="pre-line";
    $("rt-ready").textContent=a.running&&a.kind==="replay"?a.stage:(d.ready?"replay 준비 완료":(d.checks.robot&&d.checks.boxes?"첫 state·배치 확인 완료":"대기"));
    $("rt-replay").disabled=RT.starting||!d.ready;
  } catch(e) { if(seq===RT.seq) {rtClearCamera();$("rt-live-msg").textContent="연결 오류: "+e.message;} }
  finally {RT.busy=false;}
}
async function rtCommand(command) {
  if(!RT.detail) return;
  const id=RT.selected,seq=RT.seq;
  if(command.op==="align_start"||command.op==="replay")command={...command,stationary_confirmed:$("rt-stationary-confirm").checked};
  try {
    const result=await rtFetch("/episode/"+encodeURIComponent(id)+"/command",{method:"POST",headers:{"Content-Type":"application/json"},
      body:JSON.stringify({...command,token:TOKEN,sid:SID,fingerprint:RT.detail.fingerprint})});
    if(result.alignment) {

      if(seq===RT.seq) {RT.alignment=result.alignment;RT.stopRequested=false;}
      if(seq!==RT.seq||S.view!=="retargeting"||document.hidden) {
        await fetch("/kuavo/retargeting/episode/"+encodeURIComponent(id)+"/command",{method:"POST",keepalive:true,
          headers:{"Content-Type":"application/json"},body:JSON.stringify({op:"align_stop"})});
        RT.stopRequested=true;
      }
    }
    if(seq===RT.seq) $("rt-command-msg").textContent=command.op==="align_start"?"베이스 정렬과 몸통·양팔·그리퍼 첫 state 준비를 함께 시작합니다. head 관절 제외.":(result.text||"확인 상태를 반영했습니다.");
  } catch(e) {if(seq===RT.seq) $("rt-command-msg").textContent=e.message;}
  if(seq===RT.seq) await pullRetargeting();
}
function rtLeave() {
  const a=RT.alignment;
  if(!RT.detail||(!RT.starting&&(!a.running||!a.mine))) return Promise.resolve();
  RT.stopRequested=true;
  return fetch("/kuavo/retargeting/episode/"+encodeURIComponent(RT.selected)+"/command",{
    method:"POST",keepalive:true,headers:{"Content-Type":"application/json"},
    body:JSON.stringify({op:"align_stop",token:TOKEN,sid:SID})}).catch(()=>{});
}
async function rtAlignBeat() {
  if(RT.beatBusy||RT.stopRequested||!RT.detail||!RT.alignment.running||!RT.alignment.mine) return;
  if(S.view!=="retargeting"||document.hidden) {rtLeave();return;}
  RT.beatBusy=true;
  try {await rtFetch("/episode/"+encodeURIComponent(RT.selected)+"/command",{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify({op:"align_beat",token:TOKEN,sid:SID,fingerprint:RT.detail.fingerprint})});}
  catch(e) {rtLeave();}
  finally {RT.beatBusy=false;}
}
function bindRetargeting() {
  if(RT.bound) return;RT.bound=true;
  $("rt-refresh").addEventListener("click",rtRefresh);$("rt-search").addEventListener("input",rtRenderList);
  $("rt-overlay").addEventListener("change",()=>RT.live&&rtDraw(RT.live));
  $("rt-validation-override").addEventListener("change",e=>rtCommand({op:"confirm",step:"validation_override",checked:e.target.checked}));
  const stationaryToggle=$("rt-stationary-confirm");
  try {stationaryToggle.checked=localStorage.getItem("kuavo-rt-stationary-confirm")==="true";} catch(e) {}
  stationaryToggle.addEventListener("change",()=>{
    try {localStorage.setItem("kuavo-rt-stationary-confirm",String(stationaryToggle.checked));} catch(e) {}
  });
  $("rt-check-robot").addEventListener("change",e=>rtCommand({op:"confirm",step:"robot",checked:e.target.checked}));
  $("rt-check-boxes").addEventListener("change",e=>rtCommand({op:"confirm",step:"boxes",checked:e.target.checked}));
  $("rt-align-start").addEventListener("click",async()=>{
    if(RT.starting) return;RT.starting=true;$("rt-align-start").disabled=true;
    try {await rtCommand({op:"align_start"});} finally {RT.starting=false;}
  });
  $("rt-align-stop").addEventListener("click",()=>{RT.stopRequested=true;rtCommand({op:"align_stop"});});
  window.addEventListener("pagehide",rtLeave);
  document.addEventListener("visibilitychange",()=>{if(document.hidden)rtLeave();});
  setInterval(rtAlignBeat,400);
  $("rt-reset").addEventListener("click",()=>rtCommand({op:"confirm",step:"robot",checked:false}));
  $("rt-replay").addEventListener("click",async()=>{
    if(RT.starting)return;RT.starting=true;$("rt-replay").disabled=true;
    try{await rtCommand({op:"replay"});}finally{RT.starting=false;}
  });
  rtBindRange();
  $("rt-video").addEventListener("error",()=>{$("rt-video-msg").textContent="영상을 불러오지 못했습니다. 목록을 새로고침하세요.";});
}

function rtInitialLabel(d) {
  const s=d.selection;
  $("rt-initial").textContent=`${s?`선택 시작: 영상 ${s.start.video_s.toFixed(3)}초 · 실행 ${s.start.execution_s.toFixed(3)}초 · `:""}원본 ${d.initial.source_s.toFixed(3)}초의 자세 · 베이스·몸통 4·양팔 14·그리퍼 · head 제외`;
}
function rtRangeIndex(points,t) {
  let best=0;
  for(let i=1;i<points.length;i++) if(Math.abs(points[i].video_s-t)<Math.abs(points[best].video_s-t)) best=i;
  return best;
}
function rtRangeSetup(d) {
  const t=d.timeline,s=d.selection;
  RT.rangeDirty=false;RT.rangePreview=false;
  $("rt-range-preview").textContent="▶ 선택 구간 미리보기";
  if(!t?.available||!s) {
    RT.range=null;$("rt-range").disabled=true;
    $("rt-range-summary").textContent="구간 선택 대기";
    $("rt-range-msg").textContent=t?.reason||"영상과 궤적의 시각 매핑이 필요합니다.";return;
  }
  const points=t.points;
  RT.range={points,start:rtRangeIndex(points,s.start.video_s),end:rtRangeIndex(points,s.end.video_s)};
  for(const id of ["rt-range-start","rt-range-end"]) $(id).max=String(points.length-1);
  for(const id of ["rt-range-start-time","rt-range-end-time"]) $(id).max=String(points.at(-1).video_s);
  $("rt-range").disabled=!!RT.alignment.running;
  $("rt-range-msg").textContent="핸들을 놓으면 구간이 저장됩니다. 선택 시작점으로 정렬·박스 목표가 갱신되며 출발·정지 궤적은 별도 검증합니다.";
  rtRangePaint();rtRangePosition();
}
function rtRangePaint() {
  const r=RT.range;if(!r)return;
  const a=r.points[r.start],b=r.points[r.end],duration=r.points.at(-1).video_s;
  $("rt-range-start").value=String(r.start);$("rt-range-end").value=String(r.end);
  $("rt-range-start-time").value=a.video_s.toFixed(3);$("rt-range-end-time").value=b.video_s.toFixed(3);
  $("rt-range-start").setAttribute("aria-valuetext",a.video_s.toFixed(3)+"초");
  $("rt-range-end").setAttribute("aria-valuetext",b.video_s.toFixed(3)+"초");
  $("rt-range-summary").textContent=`${a.video_s.toFixed(3)} – ${b.video_s.toFixed(3)}초 · ${(b.execution_s-a.execution_s).toFixed(3)}초 선택`;
  $("rt-range-fill").style.left=(a.video_s/duration*100)+"%";
  $("rt-range-fill").style.right=(100-b.video_s/duration*100)+"%";
}
function rtRangePosition() {
  const v=$("rt-video"),r=RT.range;
  $("rt-range-position").textContent=`현재 ${(v.currentTime||0).toFixed(3)}초`;
  if(r) $("rt-range-playhead").style.left=(Math.min(1,Math.max(0,v.currentTime/r.points.at(-1).video_s))*100)+"%";
}
function rtRangeSet(which,index,seek=true) {
  const r=RT.range;if(!r||RT.rangeSaving||RT.alignment.running||RT.starting)return;
  index=Math.round(Number(index));if(!Number.isFinite(index))return;
  if(which==="start")r.start=Math.max(0,Math.min(index,r.end-1));
  else r.end=Math.min(r.points.length-1,Math.max(index,r.start+1));
  RT.rangeDirty=true;RT.rangePreview=false;
  const v=$("rt-video");v.pause();
  if(seek) v.currentTime=r.points[which==="start"?r.start:Math.min(r.end,r.points.length-2)].video_s;
  $("rt-replay").disabled=true;$("rt-align-start").disabled=true;
  $("rt-check-robot").checked=false;$("rt-check-boxes").checked=false;$("rt-validation-override").checked=false;$("rt-check-boxes").disabled=true;
  $("rt-ready").textContent="구간 변경 중";$("rt-range-msg").textContent="선택 구간 저장 대기…";
  // Never show placement for the previously selected start while a new target is pending.
  $("rt-projection").innerHTML="";RT.live=null;RT.frame=null;
  rtRangePaint();rtRangePosition();
}
async function rtRangeCommit() {
  if(!RT.range||!RT.rangeDirty||RT.rangeSaving)return;
  const id=RT.selected,seq=RT.seq,r=RT.range;
  RT.rangeSaving=true;$("rt-range").disabled=true;
  try {
    const result=await rtFetch("/episode/"+encodeURIComponent(id)+"/command",{method:"POST",headers:{"Content-Type":"application/json"},
      body:JSON.stringify({op:"select_range",token:TOKEN,sid:SID,fingerprint:RT.detail.fingerprint,
        start_video_s:r.points[r.start].video_s,end_video_s:r.points[r.end].video_s})});
    if(seq!==RT.seq)return;
    RT.detail=result.detail;rtRangeSetup(RT.detail);rtInitialLabel(RT.detail);
    $("rt-command-msg").textContent=result.text;$("rt-ready").textContent="대기";
    try {localStorage.setItem("kuavo-rt-range:"+id,JSON.stringify({fingerprint:RT.detail.episode_fingerprint,
      start:RT.detail.selection.start.video_s,end:RT.detail.selection.end.video_s}));}catch(e){}
  } catch(e) {
    if(seq===RT.seq) {
      const d=await rtFetch("/episode/"+encodeURIComponent(id)+"?sid="+encodeURIComponent(SID)).catch(()=>null);
      if(seq!==RT.seq)return;
      if(d){RT.detail=d;rtRangeSetup(d);rtInitialLabel(d);}
      $("rt-range-msg").textContent="구간 저장 실패: "+e.message;
    }
  } finally {
    if(seq===RT.seq){RT.rangeSaving=false;$("rt-range").disabled=!RT.range||RT.alignment.running;await pullRetargeting();}
  }
}
async function rtRangeRestore() {
  if(!RT.range)return;
  let saved;try{saved=JSON.parse(localStorage.getItem("kuavo-rt-range:"+RT.selected));}catch(e){return;}
  if(!saved||saved.fingerprint!==RT.detail.episode_fingerprint||!Number.isFinite(saved.start)||!Number.isFinite(saved.end))return;
  const r=RT.range,a=rtRangeIndex(r.points,saved.start),b=rtRangeIndex(r.points,saved.end);
  if(a>=b||(a===r.start&&b===r.end))return;
  r.start=a;r.end=b;RT.rangeDirty=true;rtRangePaint();await rtRangeCommit();
}
function rtRangeTick() {
  const v=$("rt-video"),r=RT.range;
  if(!RT.rangePreview||!r||v.paused)return;
  const a=r.points[r.start].video_s,b=r.points[r.end].video_s;
  if(v.currentTime>=b-.002||v.ended) {
    if($("rt-range-loop").checked){v.currentTime=a;v.play().catch(()=>{});}
    else{v.pause();v.currentTime=b;RT.rangePreview=false;$("rt-range-preview").textContent="▶ 선택 구간 미리보기";}
  }
  rtRangePosition();
}
function rtBindRange() {
  for(const which of ["start","end"]) {
    $("rt-range-"+which).addEventListener("input",e=>rtRangeSet(which,e.target.value));
    $("rt-range-"+which).addEventListener("change",rtRangeCommit);
    $("rt-range-"+which+"-time").addEventListener("change",e=>{
      if(!RT.range)return;const t=Number(e.target.value);if(!Number.isFinite(t))return;
      rtRangeSet(which,rtRangeIndex(RT.range.points,t));rtRangeCommit();
    });
    $("rt-range-mark-"+which).addEventListener("click",()=>{
      if(!RT.range)return;rtRangeSet(which,rtRangeIndex(RT.range.points,$("rt-video").currentTime));rtRangeCommit();
    });
  }
  $("rt-range-full").addEventListener("click",()=>{
    if(!RT.range)return;RT.range.end=RT.range.points.length-1;rtRangeSet("start",0);rtRangeCommit();
  });
  $("rt-range-preview").addEventListener("click",async()=>{
    const v=$("rt-video");if(!RT.range)return;
    if(RT.rangePreview&&!v.paused){v.pause();RT.rangePreview=false;$("rt-range-preview").textContent="▶ 선택 구간 미리보기";return;}
    RT.rangePreview=true;v.currentTime=RT.range.points[RT.range.start].video_s;
    try{await v.play();$("rt-range-preview").textContent="Ⅱ 미리보기 정지";}
    catch(e){RT.rangePreview=false;$("rt-range-msg").textContent="미리보기 재생 실패: "+e.message;}
  });
  const v=$("rt-video");
  v.addEventListener("timeupdate",()=>{rtRangePosition();rtRangeTick();});
  v.addEventListener("play",()=>{
    if(RT.rangeFrame!==null)cancelAnimationFrame(RT.rangeFrame);
    const tick=()=>{rtRangeTick();if(!v.paused)RT.rangeFrame=requestAnimationFrame(tick);else RT.rangeFrame=null;};tick();
  });
  v.addEventListener("ended",()=>{if(RT.rangePreview&&RT.range&&$("rt-range-loop").checked){v.currentTime=RT.range.points[RT.range.start].video_s;v.play().catch(()=>{});}});
}
