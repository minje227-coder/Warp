/* Replay output history is separate from the input episode library. */
"use strict";
const RTR={rows:[],selected:null,detailId:null,busy:false,at:0,seq:0};
function rtrStatus(row) {
  const names={starting:"준비 중",recording:"녹화 중",running:"replay 중",saving:"저장 중",saved:"저장 완료",
    completed:"완료",stopped:"중단",aborted:"중단",failed:"실패",error:"오류",unknown:"상태 미기록"};
  const task=(row.success===true?" · 작업 성공":row.success===false?" · 작업 실패":" · 작업 미분류")+
    (row.collection_valid===true?" · 데이터 검사 통과":row.collection_valid===false?" · 데이터 검사 실패":"");
  return (names[row.status]||row.status)+task;
}
function rtrTime(t) {
  return t===null?"시작 시각 미기록":new Date(t*1000).toLocaleString("ko-KR",{timeZone:"Asia/Seoul",hour12:false});
}
function rtrPaint() {
  const list=$("rt-recordings-list");
  $("rt-recordings-count").textContent=String(RTR.rows.length);
  list.innerHTML=RTR.rows.map(row=>`<div class="rtr-record-row"><button type="button" class="rt-episode" data-rtr="${esc(row.id)}" aria-pressed="${RTR.selected===row.id}"><strong>${esc(row.name)}</strong><small>${esc(rtrTime(row.started_at))}</small><small>${esc(rtrStatus(row))}${row.duration_s===null?"":` · ${row.duration_s.toFixed(1)}초`}</small><small>원본 ${esc(row.source_episode.split("_")[0])}</small></button><button type="button" class="btn sm" data-rtr-rename="${esc(row.id)}" aria-label="${esc(row.name)} 녹화 이름 변경" ${["starting","recording","running","saving"].includes(row.status)?"disabled":""}>이름 변경</button><button type="button" class="btn sm" data-rtr-delete="${esc(row.id)}" aria-label="${esc(row.name)} 녹화 기록 삭제" ${["starting","recording","running","saving"].includes(row.status)?"disabled":""}>삭제</button></div>`).join("");
  list.querySelectorAll("[data-rtr]").forEach(b=>b.addEventListener("click",()=>rtrOpen(b.dataset.rtr)));
  list.querySelectorAll("[data-rtr-rename]").forEach(b=>b.addEventListener("click",()=>rtrRename(b.dataset.rtrRename,b)));
  list.querySelectorAll("[data-rtr-delete]").forEach(b=>b.addEventListener("click",()=>rtrDelete(b.dataset.rtrDelete,b)));
}
async function rtrRename(id,button) {
  const row=RTR.rows.find(r=>r.id===id);if(!row)return;
  const name=window.prompt("녹화 이름 (1~180자)",row.name);if(name===null)return;
  button.disabled=true;
  try {
    await rtFetch(`/recordings/${encodeURIComponent(id)}/rename`,{method:"POST",headers:{"Content-Type":"application/json"},
      body:JSON.stringify({token:TOKEN,name})});
    await rtrPull(true);
    if(RTR.detailId===id&&$("rt-recording-detail").open)await rtrOpen(id);
    if(S.view==="rec")await pullRec();
    $("rt-recordings-msg").textContent="녹화 이름을 변경했습니다.";
  } catch(e) {$("rt-recordings-msg").textContent="이름을 변경하지 못했습니다: "+e.message;}
  finally {if(button.isConnected)button.disabled=false;}
}
async function rtrDelete(id,button) {
  button.disabled=true;
  const message=()=>S.view==='rec'?$("msg2"):$("rt-recordings-msg");
  const post=(suffix,extra={})=>rtFetch(`/recordings/${encodeURIComponent(id)}/${suffix}`,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({token:TOKEN,...extra})});
  try {
    const preview=await post("delete-preview");
    if(!window.confirm(`${preview.name}\n녹화 기록 ${(preview.bytes/1e6).toFixed(2)} MB를 삭제할까요?\n원본 에피소드·IK 결과는 보존합니다.\n복구 기록은 최근 3개만 보관하며, 초과 시 가장 오래된 기록부터 영구 삭제됩니다.`))return;
    await post("delete",{revision:preview.revision});
    if(RTR.selected===id) {
      RTR.selected=RTR.detailId=null;++RTR.seq;
      $("rt-recording-videos").querySelectorAll("video").forEach(v=>v.pause());
      if($("rt-recording-detail").open)$("rt-recording-detail").close();
    }
    await rtrPull(true);
    if(S.view==='rec')await pullRec();
    message().textContent="녹화 기록을 삭제했습니다. 복구용 폴더에는 최근 3개만 보관합니다.";
  } catch(e) {message().textContent="삭제하지 못했습니다: "+e.message;}
  finally {if(button.isConnected)button.disabled=false;}
}
async function rtrPull(force=false) {
  if(RTR.busy || !$("rt-recordings-list") || (!force && (S.view!=="retargeting"||Date.now()-RTR.at<5000))) return;
  RTR.busy=true;RTR.at=Date.now();
  try {
    const d=await rtFetch("/recordings");RTR.rows=d.recordings;rtrPaint();
    $("rt-recordings-msg").textContent=RTR.rows.length?(d.unreadable?`읽지 못한 기록 ${d.unreadable}개`:""):
      "아직 Replay 녹화 기록이 없습니다. 실행·녹화한 결과가 여기에 따로 표시됩니다.";
  } catch(e) {$("rt-recordings-msg").textContent="기록을 불러오지 못했습니다: "+e.message;}
  finally {RTR.busy=false;}
}
async function rtrOpen(id) {
  RTR.selected=id;RTR.detailId=null;const seq=++RTR.seq;rtrPaint();
  $("rt-task-save").disabled=true;$("rt-task-result").disabled=true;$("rt-task-note").disabled=true;
  const dialog=$("rt-recording-detail");if(dialog.parentElement!==document.body)document.body.append(dialog);if(!dialog.open)dialog.showModal();$("rt-recording-title").textContent="녹화 기록 불러오는 중…";
  $("rt-recording-info").textContent="";$("rt-recording-files").textContent="";$("rt-recording-videos").replaceChildren();
  $("rt-recording-qc").textContent="";
  try {
    const d=await rtFetch("/recordings/"+encodeURIComponent(id));if(seq!==RTR.seq)return;
    RTR.detailId=id;
    $("rt-recording-title").textContent=d.name;
    $("rt-recording-info").textContent=[rtrStatus(d),rtrTime(d.started_at),
      "원본 episode: "+d.source_episode,d.duration_s===null?"":`길이: ${d.duration_s.toFixed(1)}초`,
      "저장 위치: "+d.recording_path,d.note].filter(Boolean).join("\n");
    $("rt-task-result").value=d.success===true?"success":d.success===false?"failure":"unlabeled";
    $("rt-task-note").value=d.task_note||"";$("rt-task-save").disabled=!d.can_label;
    $("rt-task-result").disabled=!d.can_label;$("rt-task-note").disabled=!d.can_label;
    if(d.collection)$("rt-recording-info").textContent+="\n로봇 bag: "+(d.collection.path||"준비 중")+
      "\n"+(d.collection.errors||[]).join("\n");
    $("rt-recording-qc").textContent=d.collection?Object.entries(d.collection.topics||{}).map(([topic,row])=>
      `${topic}: ${row.count}개 · ${row.hz.toFixed(1)}Hz · 최대 간격 ${row.max_gap_s.toFixed(3)}초`).join("\n"):"이전 기록에는 수집 품질 정보가 없습니다.";
    if(d.tracking&&d.tracking.samples)$("rt-recording-qc").textContent+=
      `\n관절 추종 오차: 최대 ${d.tracking.maximum_joint_error_deg.toFixed(2)}° · RMS ${d.tracking.rms_joint_error_deg.toFixed(2)}°`;
    $("rt-recording-files").textContent=d.files.length?d.files.map(f=>`${f.name} · ${(f.bytes/1e6).toFixed(2)} MB`).join("\n"):"저장 파일 없음";
    for(const media of d.videos) {
      const label=document.createElement("p");label.textContent=media.label;
      const video=document.createElement("video");video.controls=true;video.playsInline=true;video.preload="metadata";
      video.setAttribute("aria-label",d.name+" · "+media.label);video.src=`/kuavo/retargeting/recordings/${encodeURIComponent(id)}/video/${media.index}`;
      $("rt-recording-videos").append(label,video);
    }
  } catch(e) {if(seq===RTR.seq)$("rt-recording-info").textContent=e.message;}
}
document.addEventListener("DOMContentLoaded",()=>{
  $("rt-task-save").addEventListener("click",async()=>{
    const id=RTR.detailId,seq=RTR.seq;if(!id||id!==RTR.selected)return;
    $("rt-task-save").disabled=true;
    try {
      const value=$("rt-task-result").value;
      await rtFetch("/recordings/"+encodeURIComponent(id)+"/label",{method:"POST",headers:{"Content-Type":"application/json"},
        body:JSON.stringify({token:TOKEN,success:value==="success"?true:value==="failure"?false:null,note:$("rt-task-note").value})});
      await rtrPull(true);if(seq===RTR.seq)await rtrOpen(id);
    } catch(e) {$("rt-recording-info").textContent=e.message;$("rt-task-save").disabled=false;}
  });
  $("rt-recordings-refresh").addEventListener("click",()=>rtrPull(true));
  $("rt-recording-close").addEventListener("click",()=>$("rt-recording-detail").close());
  $("rt-recording-detail").addEventListener("close",()=>{
    RTR.seq++;$("rt-recording-videos").querySelectorAll("video").forEach(v=>v.pause());
  });
  // Existing refresh/tab selection also refreshes the independent output list.
  $("rt-refresh").addEventListener("click",()=>rtrPull(true));
  document.querySelector('[data-screen="retargeting"]').addEventListener("click",()=>rtrPull(true));
  setInterval(()=>rtrPull(),1000);rtrPull();
});
