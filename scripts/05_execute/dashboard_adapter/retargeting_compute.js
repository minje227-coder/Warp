/* Offline standard queue controls. Calculation never sends robot commands. */
(() => {
  "use strict";
  const C={jobs:{},busy:false,signature:"",bound:false};
  const active=j=>j && ["queued","running"].includes(j.status);
  const originalList=rtRenderList,originalRefresh=rtRefresh,originalSelect=rtSelect,originalBind=bindRetargeting;
  const statusText=j=>j?({queued:"대기",running:"계산 중",done:"검증 통과",rejected:"검증 미통과",failed:"계산 실패",cancelled:"취소됨"}[j.status]||j.status):"";
  function summary() {
    const jobs=Object.values(C.jobs),running=jobs.filter(j=>j.status==="running").length,queued=jobs.filter(j=>j.status==="queued").length;
    if($("rt-compute-summary")) $("rt-compute-summary").textContent=`최대 2개 동시 계산 · 진행 ${running} · 대기 ${queued}`;

  }
  rtRenderList=function() {
    if(!C.bound)return originalList();
    const query=$("rt-search").value.toLowerCase(),rows=RT.rows.filter(r=>(r.label+" "+r.id).toLowerCase().includes(query));
    $("rt-count").textContent=String(RT.rows.length);
    $("rt-list").innerHTML=rows.map(r=>{
      const j=C.jobs[r.id],working=active(j),id=esc(r.id);
      return `<div class="rt-calc-row"><button type="button" class="rt-episode" data-rt-id="${id}" aria-pressed="${RT.selected===r.id}"><strong>${esc(r.label)}</strong><small>${esc(working?statusText(j):rtLabel(r.status))} · ${r.has_video?"4-window 있음":"미계산 · 영상 없음"}</small>${r.has_video?`<small>${r.coordinate_frame==="fixed_rack_height"?"실측 높이 고정":r.coordinate_frame==="observed_base_link"?"이전 로봇 관측 기준 · 재계산 필요":"녹화 좌표계 · 재계산 필요"}</small>`:""}${j?`<small class="rt-job-stage">${esc(j.stage||statusText(j))}${j.tcp_lowering_cm!=null?` · TCP ↓${j.tcp_lowering_cm}cm`:""}</small>`:""}</button><button type="button" class="btn sm rt-calc-one" data-rt-compute="${id}" data-rt-op="${working?"cancel":"enqueue"}">${working?"중단":r.status==="not_computed"?"계산":"재계산"}</button><button type="button" class="btn sm rt-delete-one" data-rt-delete="${id}" ${working?"disabled":""} aria-label="${esc(r.label)} episode 삭제">삭제</button></div>`;
    }).join("");
    $("rt-list-msg").textContent=rows.length?"":"표시할 episode가 없습니다.";
    $("rt-list").querySelectorAll("[data-rt-id]").forEach(b=>b.onclick=()=>rtSelect(b.dataset.rtId));
    $("rt-list").querySelectorAll("[data-rt-compute]").forEach(b=>b.onclick=()=>submit([b.dataset.rtCompute],b.dataset.rtOp));
    $("rt-list").querySelectorAll("[data-rt-delete]").forEach(b=>b.onclick=()=>previewDelete(b.dataset.rtDelete));
    const selected=C.jobs[RT.selected];
    if($("rt-compute-log")) $("rt-compute-log").textContent=selected?(selected.log_tail||selected.error||selected.stage):"episode를 선택하면 계산 로그를 표시합니다.";
    summary();
  };
  async function refresh() {
    const [catalog,queue]=await Promise.all([rtFetch(""),rtFetch("/compute")]);
    const signature=JSON.stringify([catalog.episodes,queue.jobs]);
    if(signature!==C.signature) {
      const previous=RT.rows.find(r=>r.id===RT.selected),previousJob=C.jobs[RT.selected];
      RT.rows=catalog.episodes;C.jobs=Object.fromEntries(queue.jobs.map(j=>[j.episode,j]));C.signature=signature;
      rtRenderList();
      const current=RT.rows.find(r=>r.id===RT.selected);
      if(current && current.status==="not_computed" && !RT.detail)renderEmpty(current);
      const job=C.jobs[RT.selected];
      const published=job && ["done","rejected"].includes(job.status) &&
        (previousJob?.id!==job.id || previousJob?.status!==job.status || previousJob?.result!==job.result);
      if(previous && current && current.has_video && (!previous.has_video || published))await rtSelect(current.id);
      if(RT.selected)showDiagnostics(RT.selected);
    }
  }
  rtRefresh=async function() {
    if(!C.bound)return originalRefresh();
    try {await refresh();}catch(e){$("rt-list-msg").textContent=e.message;}
  };
  function renderEmpty(r) {
    const j=C.jobs[r.id],working=active(j);
    $("rt-empty").innerHTML=`<h2>${esc(r.label)} · ${working?statusText(j):"미계산"}</h2><p class="dim">${working?esc(j.stage||"계산 준비 중")+" · 완료되면 영상이 여기에 자동으로 표시됩니다.":"목록의 계산 버튼을 누르세요.<br>각 episode의 계산 버튼을 누르면 최대 2개까지 동시에 처리하고, 나머지는 대기합니다."}</p><button type="button" class="btn go" id="rt-empty-compute" ${working?"disabled":""}>${working?"계산 중":"계산"}</button>`;
    $("rt-empty-compute").onclick=()=>submit([r.id]);
  }
  rtSelect=async function(id) {
    const r=RT.rows.find(row=>row.id===id);
    if(r && r.status==="not_computed") {
      if(typeof rtLeave==="function")rtLeave();
      RT.selected=id;RT.detail=null;++RT.seq;rtClearCamera();
      const v=$("rt-video");v.pause();v.removeAttribute("src");v.load();
      $("rt-work").hidden=true;$("rt-empty").hidden=false;
      renderEmpty(r);rtRenderList();showDiagnostics(id);return;
    }
    await originalSelect(id);rtRenderList();showDiagnostics(id);
  };
  const subjectName=s=>s.replace(/^zarm_([lr])(\d+)_joint$/,(_,side,n)=>(side==="l"?"왼팔":"오른팔")+n+"번 관절");
  const number=v=>Number.isFinite(v)?v.toFixed(3):"미기록";
  async function showDiagnostics(id) {
    const seq=RT.seq;
    try {
      const d=await rtFetch("/episode/"+encodeURIComponent(id)+"/validation");
      if(RT.selected!==id || RT.seq!==seq)return;
      let panel=$("rt-validation");
      if(d.status==="not_computed"&&!d.compute_error) {if(panel)panel.remove();return;}
      if(!panel){panel=document.createElement("section");panel.id="rt-validation";panel.setAttribute("aria-label","선택 episode의 미통과 이유");}
      const target=RT.detail?$("rt-video").closest(".pb"):$("rt-empty");target.append(panel);
      const reasons=d.reasons||[],checks=d.checks||{};
      const rows=reasons.map(r=>{
        const unit=r.unit||"",metric=Number.isFinite(r.excess)?`최대 초과 <b>${number(r.excess)}${esc(unit)}</b> · 측정 ${number(r.measured)}${esc(unit)} / 기준 ${esc(r.operator||"≤")} ${number(r.limit)}${esc(unit)}`:"";
        const range=Number.isFinite(r.first_execution_s)?`발생 범위 ${number(r.first_execution_s)}–${number(r.last_execution_s)}초`:"";
        const v=$("rt-video"),outside=Number.isFinite(v.duration)&&r.worst_execution_s>v.duration;
        const seek=Number.isFinite(r.worst_execution_s)?`<button type="button" class="btn sm" data-rt-failure-time="${r.worst_execution_s}" ${outside?"disabled":""}>${number(r.worst_execution_s)}초 보기${outside?" (영상 범위 밖)":""}</button>`:"";
        return `<li><b>${esc(r.label)}</b>${r.samples!=null?` · ${r.samples}개 샘플`:""}${r.subjects?.length?`<small>${esc(r.subjects.map(subjectName).join(" · "))}</small>`:""}${metric?`<p>${metric}</p>`:""}${r.code==="joint_guard"?"<small>기계적 한계보다 안쪽에 둔 안전 여유 기준입니다.</small>":""}<p>${range} ${seek}</p></li>`;
      }).join("");
      const ok=(label,n)=>n===0?`${label} 통과`:n==null?`${label} 미기록`:`${label} ${n}건 미통과`;
      const dynamics=(label,r)=>r==null?`${label} 미기록`:`${label} ${r<=1.00001?"통과":"미통과"} (상한의 ${(r*100).toFixed(1)}%)`;
      panel.innerHTML=`<h3>${d.status==="verified"?"검증 통과":"미통과 이유"} <small>전체 계산 결과</small></h3>${d.invalid_samples!=null?`<p>중간 자세 검증: ${d.sampled_states}개 중 <b>${d.invalid_samples}개 미통과</b></p>`:""}${rows?`<ul>${rows}</ul>`:""}${d.checks?`<p class="dim">${esc([ok("충돌 안전거리",checks.collision_failures),ok("MoveIt",checks.moveit_invalid==null?null:checks.moveit_invalid+(checks.moveit_interframe_invalid||0)),dynamics("속도",checks.velocity_ratio),dynamics("가속도",checks.acceleration_ratio)].join(" · "))}</p>`:""}${reasons.length>1?'<p class="dim">같은 샘플이 여러 미통과 항목에 포함될 수 있습니다.</p>':""}${d.compute_error?`<details open><summary>최근 계산 오류</summary><pre>${esc(d.compute_error)}</pre></details>`:""}`;
      panel.querySelectorAll("[data-rt-failure-time]").forEach(b=>b.onclick=()=>{const v=$("rt-video");v.pause();v.currentTime=Number(b.dataset.rtFailureTime);});
    }catch(e){if(RT.selected===id)$("rt-compute-message").textContent="검증 상세를 불러오지 못했습니다: "+e.message;}
  }
  async function previewDelete(id) {
    if(C.busy)return;
    C.busy=true;
    try {
      const plan=await rtFetch("/episode/"+encodeURIComponent(id)+"/delete-preview",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({token:TOKEN})});
      const dialog=$("rt-delete-dialog");
      $("rt-delete-title").textContent=plan.label+" episode를 영구 삭제할까요?";
      $("rt-delete-info").textContent=plan.scope+" · "+(plan.bytes/1024**3).toFixed(2)+" GiB · "+plan.entries+"개 항목";
      $("rt-delete-paths").textContent=plan.paths.join("\n");
      $("rt-delete-note").textContent=plan.device_note+" 삭제 후 복구할 수 없습니다.";
      $("rt-delete-error").textContent="";
      const button=$("rt-delete-confirm");button.disabled=false;
      button.onclick=async()=>{
        button.disabled=true;
        try {
          const d=await rtFetch("/episode/"+encodeURIComponent(id)+"/delete",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({token:TOKEN,ticket:plan.ticket,confirmed:true})});
          dialog.close();
          if(RT.selected===id){
            RT.selected=null;RT.detail=null;++RT.seq;RT.range=null;RT.rangePreview=false;
            const v=$("rt-video");v.pause();v.removeAttribute("src");v.load();rtClearCamera();
            $("rt-work").hidden=true;$("rt-empty").hidden=false;$("rt-empty").textContent="episode를 선택하세요.";
            $("rt-validation")?.remove();
          }
          await refresh();
          $("rt-compute-message").textContent=d.cleanup_pending?.length?"목록에서 제거했으나 일부 파일 삭제에 실패했습니다. 관리자 로그를 확인하세요.":"episode 원본과 계산 결과를 삭제했습니다.";
        }catch(e){$("rt-delete-error").textContent=e.message;}
      };
      dialog.showModal();$("rt-delete-cancel").focus();
    }catch(e){$("rt-compute-message").textContent=e.message;}
    finally{C.busy=false;}
  }
  async function submit(keys,op="enqueue") {
    const input=$("rt-tcp-lowering-cm");
    const tcpLoweringCm=input?.value.trim()===""?NaN:Number(input?.value??0);
    if(op==="enqueue" && (!Number.isFinite(tcpLoweringCm)||tcpLoweringCm<0||tcpLoweringCm>30)) {
      $("rt-compute-message").textContent="TCP 하향 보정은 0–30 cm 사이 숫자로 입력하세요.";input?.focus();return;
    }
    C.busy=true;summary();
    try {
      await rtFetch("/compute",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({op,episodes:keys,token:TOKEN,tcp_lowering_cm:op==="enqueue"?tcpLoweringCm:undefined})});
      $("rt-compute-message").textContent=op==="cancel"?"계산을 중단합니다.":`${keys.length}개 실측 고정 높이 계산 · 양손 TCP ${tcpLoweringCm}cm 하향을 요청했습니다. 완료 후 영상이 표시됩니다.`;
      await refresh();
    } catch(e){$("rt-compute-message").textContent=e.message;}
    finally {C.busy=false;summary();}
  }
  bindRetargeting=function() {
    originalBind();if(C.bound)return;C.bound=true;
    const style=document.createElement("style");style.textContent=`.rt-calc-row{display:grid;grid-template-columns:minmax(0,1fr) auto auto;align-items:center;gap:5px}.rt-calc-row .rt-episode{padding:9px}.rt-job-stage{color:#2563eb!important}.rt-calc-one{padding:7px!important}#rt-compute-summary,#rt-compute-message{font-size:12px;line-height:1.6;margin:5px 0}#rt-compute-log{white-space:pre-wrap;overflow-wrap:anywhere;font-size:11px;max-height:200px;overflow:auto}.rt-compute-help{font-size:11px;color:var(--muted);margin:6px 0}.rt-sidebar .rt-library .pb{max-height:64vh}.rt-calc-row button:disabled{opacity:.5;cursor:not-allowed}#rt-validation{margin-top:16px;padding:14px;border:1px solid #fed7aa;border-radius:8px;background:#fffaf5;font-size:13px;width:100%;box-sizing:border-box}#rt-validation h3{margin:0 0 8px;font-size:14px}#rt-validation small{font-size:12px;font-weight:400;display:block;color:#64748b}#rt-validation ul{padding-left:20px;margin:10px 0}#rt-validation li{margin:10px 0}#rt-validation p{margin:5px 0;line-height:1.7}#rt-validation pre{white-space:pre-wrap;overflow-wrap:anywhere;max-height:180px;overflow:auto}`;document.head.append(style);
    const controls=document.createElement("div");controls.innerHTML=`<p id="rt-compute-summary" role="status"></p><label for="rt-tcp-lowering-cm">양손 TCP 박스 태그 아래축 보정 (cm)</label><input id="rt-tcp-lowering-cm" type="number" min="0" max="30" step="0.1" value="3" style="width:80px;margin:6px" aria-describedby="rt-tcp-lowering-help"><p id="rt-tcp-lowering-help" class="rt-compute-help">박스 태그 아래축(-Y) 방향 · 전체 궤적에 동일 적용<br>0 = 보정 없음 · 다음 계산부터 적용</p><p class="rt-compute-help">태그 높이 고정: 0·4·1 = 1.615m / 2·5·3 = 1.000m<br>녹화 XY·방향 유지 · 로봇 카메라·TF 불필요<br>TCP 보정은 박스 태그 -Y만 사용 · 축 확인 불가 시 계산 거부<br>팔 초기값 수정 적용 · 녹화 끝 2초 제외<br>계산·검증 결과는 episode마다 최신 1개 보관</p><p id="rt-compute-message" role="status"></p><details><summary>선택 episode 계산 로그</summary><pre id="rt-compute-log"></pre></details>`;
    $("rt-list").before(controls);
    const dialog=document.createElement("dialog");dialog.id="rt-delete-dialog";
    dialog.setAttribute("aria-labelledby","rt-delete-title");
    dialog.style.cssText="max-width:640px;width:90vw;border:1px solid #fca5a5;border-radius:12px;padding:24px";
    dialog.innerHTML=`<h3 id="rt-delete-title"></h3><p id="rt-delete-info"></p><details><summary>삭제할 경로 확인</summary><pre id="rt-delete-paths" style="white-space:pre-wrap;overflow-wrap:anywhere;max-height:220px;overflow:auto"></pre></details><p id="rt-delete-note"></p><p id="rt-delete-error" role="alert" style="color:#b91c1c"></p><div style="display:flex;justify-content:flex-end;gap:10px"><button type="button" class="btn" id="rt-delete-cancel">취소</button><button type="button" class="btn" id="rt-delete-confirm" style="background:#b91c1c;color:white">원본·결과 영구 삭제</button></div>`;
    document.body.append(dialog);$("rt-delete-cancel").onclick=()=>dialog.close();
    rtRenderList();
    setInterval(()=>{if(S.view==="retargeting")rtRefresh();},3000);
  };
})();
