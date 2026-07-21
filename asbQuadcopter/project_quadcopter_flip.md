---
name: quadcopter-flip
description: "Parrot Mambo 백플립(외란 복구형). 고정수평목표 SO(3) 기하제어 + momentum 플립, ω_crit 판정. 3DOF 검증완료, 6DOF 이식 중."
metadata: 
  node_type: memory
  type: project
  originSessionId: d3d09c01-17ab-44f0-9050-8c69083c0e66
---

# 쿼드콥터 플립 프로젝트 (외란 복구형 백플립)

> 이 파일은 2026-07-09에 대화 기준으로 전면 재작성됨 (이전 누적 로그 폐기).

## 목표 & 컨셉
Parrot Mambo 백플립. **외란으로 90°↑ 뒤집히면 되돌리지 말고, 그 방향으로 플립을 완주시킨 뒤 정상 자세로 안정화.** 외란 "보상/거부(rejection)"가 아니라 외란 후 "**플립 복구(recovery)**".
- 연구 빈칸: 표준 복구는 전부 최단경로로 되돌림. momentum 타고 일부러 플립 완주는 아무도 안 함(서베이로 확인). Lee2010은 upside-down 되돌림만 다룸(플립 완주 영역 빈칸).
- ⚠️ ADRC/ESO/GP/DOB는 이 프로젝트 방법 **아님** (외란제어 프로젝트 [[project-quadcopter-disturbance]]서 잘못 끌어온 오염, 삭제됨).

## 확정 설계 (핵심)
1. **자세 제어 = 고정 수평목표 SO(3) 기하 PD.** R_des = 항상 수평(단위행렬). 플립 궤적 추종 안 함.
   - `tau = -kR·eR - kW·eW + ω×Jω`, `eR = vee(0.5(R_des'R − R'R_des))`. kR=0.06, kW=0.006.
   - 되돌림이냐 플립이냐는 **제어기가 결정 안 함 — 물리(momentum vs ω_crit)가 자동으로 가름.** 제어기는 항상 최대제동(고ω서 토크 포화).
2. **90° = 추력 freeze 모드 스위치.** 기울기≥90°(cos≤0)면 고도제어 OFF·추력 호버값 고정(T=mg) → 토크에 모터 예산 양보. (자세 불안정점 아님. 순수 고도 지지력 상실점)
3. **외란 = 토크 임펄스.** 짧게 툭 쳐서 각속도 킥 → 90° 통과.
4. **단일플립만** 다룸 (2·3바퀴는 스코프 제외, 사용자 결정 2026-07-08).

## 핵심 물리 (정답지)
- **a_max = τ_max/J = 394 rad/s²** (하드웨어 상수, 측정X). τ_max = 2L(fmax−fmin) = 0.0282 N·m.
- **판정선 ω_crit(θ) = √(2·a_max·(π−θ))** — 90°서 최대반토크로 제동해 180°에 딱 멈추는 임계 각속도.
  - θ=0°→49.8 / **90°→35.2** / 135°→24.9 / 180°→0.
  - 90°서 ω>35.2 → 플립완주, ω<35.2 → 되돌림. (n번째 언덕: √(2a_max((2n−1)π−θ)))
- **단일플립 밴드 (90° 기준): ω 35 ~ 78 rad/s.** 하한=첫언덕(180°) 넘기, 상한 78.7=둘째언덕(540°) 못넘기(√(2a_max·2.5π)). 위로 가면 2바퀴+.
- 각도 두 개의 물리: **cos(θ)=0 → 90°**(고도), **sin(θ)=0 → 180°**(자세 불안정평형).
- Euler 못 씀: pitch 90°서 짐벌락. **자세는 SO(3)/DCM 필수.**

## 3DOF 검증 완료 ✅
폴더 `/home/phm/Documents/MATLAB/Examples/R2025b/aeroblks_quad/Flip3DOF/`
- `DroneDynamics.m` 평면 강체동역학(플랜트, NED, 상태[x z θ vx vz w]) / `motorSat.m` 액추에이터한계(앞2·뒤2 덩어리, T·τ 커플링) / `flipController.m` 제어기(고도PD+자세 기하PD, `tau=J·thdd−kR·sin(θ−θd)−kW·(w−wd)`) / `flipTrajCheck.m` 궤적데모 / `flipRecoverySweep.m` IC밴드 / `flipPipeline.m` 풀파이프라인(신규).
- **flipRecoverySweep**: θ0=135° 밴드 ω0[26,76], θ0=90° 밴드 [36,79]. ω_crit 공식과 <1% 일치.
- **flipPipeline** (호버→임펄스외란→cos(θ)로 90° freeze→물리 자동 플립/되돌림→재안정화):
  - 90° 판정선(35.2)이 전 외란세기서 플립/되돌림 **100% 정확 예측**. 미세스윕: ω@90°=33.6 되돌림 / 34.96 플립 (해석 35.17과 <1%).
  - **단일플립 안전 외란범위** (τ_ext, 임펄스 0.05s): <0.084 되돌림 / **0.084~0.135 딱 한바퀴** / 0.14+ 두바퀴(제외). 1↔2바퀴 경계 0.135~0.140.
  - 단일플립 품질: 고도손실 0.5~1.6m, 정착 ~0.5s. (다중플립대 32m 이상치는 스코프 밖=무시)

## 6DOF 이식 (진행 중)
폴더 `.../QuadcopterProjectExample2/asbQuadcopter/`. 백업 `asbQuadcopter/_flip_backup_20260708/`.
- **왜 6DOF:** 3DOF=알고리즘 ground-truth(평면·1축). 6DOF=진짜 드론(3D커플링 roll/**yaw**·ω×Jω, 센서/추정기) 검증 + AI 데이터 공장. ※모터랙은 없음(실측). 6DOF 고유난점=요 커플링.
- **모델 구조:** `asbQuadcopter`(top) → FCS=`flightControlSystem` → controller=`flightController`. 자세제어 = `flightController/Flight Controller/Attitude`의 MATLAB Function `GeometricController(R,W,t)`. 추력freeze = `.../gravity feedforward//equilibrium thrust` 안 `flipWindow` + `Switch`(w0 vs SaturationThrust).
- **옛날 방식 왜 실패했나(교훈):** 6DOF가 R_des=Ry(θ_d(t)) 0→2π **궤적 추종** → 못 따라가 레퍼런스가 실제보다 180°↑ 앞섬 → sin오차 부호반전 → 역토크 제동 → **62°만 돌고 발산**(roll 42°). 실제 몸체토크 ±0.028서 포화. ⇒ 궤적추종 버리고 고정수평목표(방식B)로.

### 이식 3종 세트 — 3개 다 완료, 근데 결과 비강인 (2026-07-09)
1. **자세목표 수평고정 ✅** 사용자 직접 수정. `GeometricController`의 R_des `Ry(θd(t))`→`eye(3)`, feedforward `J·thdd` 삭제. 호버 확인.
2. **추력 freeze 기울기 기준 ✅** 사용자 직접 배선. `flipWindow(t)`→`flipWindow(R): inflip=double(R(3,3)<=0)`. R_meas(DCM)를 equilibrium thrust 서브시스템에 inport 추가+상위층 가지치기로 배선. (오일러 pitch는 짐벌락이라 못 씀, DCM(3,3)=cos기울기 사용.)
3. **외란 주입 ✅** `tau_ext_ts`는 죽은 배선(무반응). **진짜 주입점 = `nonlinearAirframe/Nonlinear`의 `Step`−`Step1`→`Sum1`(부호 +−)→AC model.** 펄스 만들려면 Step.After=Step1.After=`[0 A 0]`(pitch축), Step.Time=t0·Step1.Time=t0+폭. 현재 [0 0.01 0], t=2.0/2.1(0.1s 펄스).

### 캘리브 + 진짜 문제 규명 (2026-07-09) ⭐⭐
- **A≈0.24 → 단일 백플립. 근데 "재안정화"는 반쪽짜리:** 기울기(피치)는 0→180→0 완주+정착(15°)하는데 **요(yaw)는 안 잡히고 발산** → 애니메이션서 팽이처럼 빙빙 돎. (내 초기 "정착" 판정이 tilt/q만 보고 요를 놓침 — 정정.) A=0.24 데이터: 플립중 r(yaw rate) 15.8, 끝나도 r 계속 커짐. 그림 `flip6dof_A024.png`.
- 필요 외란 3DOF(0.09~0.13)의 **약 2배**. 3DOF 단일플립 밴드는 6DOF에 안 넘어옴(A 0.16~0.30서 바퀴수 0/1/2/3 널뛰고 대부분 미정착).
- **❌ "모터랙" 가설 폐기(중요 정정):** 6DOF 모터에 지연 **없음**(실측: nonlinearAirframe 동역학블록=SO3적분 UnitDelay 3개뿐, 모터경로 MotorsToW=순수 Gain 즉각). 내가 "모터 굼떠서"라 한 거 **틀림.**
- **✅ 진짜 원인 = 요 PD가 "스스로 요를 만들어냄" (커플링 아님! 실험으로 정정):** 순수 피치 플립은 요를 **물리적으로 안 여기함**(롤=0이면 자이로커플링 0 — 사용자가 이론적으로 지적, 맞음). **실험 증명(A=0.24): 요제어 ON→최대요속도 14.7 / 요게인0으로 OFF→0.4** (플립은 양쪽 다 완주). ⇒ 뒤집히면 요각도(atan2)가 garbage→요 PD가 엉뚱한 요토크 쏨→**없던 요를 자기가 생성.** "요 못잡음"이 아니라 "요 만들어냄". ⚠️내 앞선 'ω×Jω 커플링이 요 여기' 설명 **틀림**(정정).
- **구조: 요만 기하제어 아님.** GeometricController는 tau=[롤;피치;요] 다 계산하나 **tau(3)(요) 버리고 tau(1),tau(2)만 출력.** 요는 별도 `Yaw` 서브(순수 PD: `tau_yaw←Sum2=P_yaw·오차−D_yaw·요속도`, P=0.004). **`R2yaw`(atan2 요각) 블록은 dangling 확인**(옛 배선버그 잔재, 안 씀). 실제 쓰는 요=추정기 `<yaw>`.
- **⚠️ 위치제어 함정(2026-07-09):** 사용자가 Yaw 서브블록 **출력 통째로 끊음 → 플립 완벽·요 clean, 근데 위치제어 깨짐(수평 드리프트).** 이유: Yaw 서브 출력 2개 = ①**요각도→XY위치제어기**(heading 필요) ②tau_yaw→믹서. 통째 끊으면 garbage tau_yaw뿐 아니라 **요각도도 끊겨** 위치제어기가 방향 몰라 드리프트. ⇒ **요각도는 살리고 tau_yaw만 교체**할 것.
- **⚠️⚠️ 위치제어 깨짐의 진짜 원인 = 요 아님 (2026-07-10 정정):** 사용자가 "토크만 끊고 요각도는 안끊었는데 왜 위치제어 안됨?" 재확인. 정적 점검 결과 **진범은 step① R_des=eye.** 자세제어가 항상 수평만 목표→**refAttitude(위치제어기가 만드는 기울기 명령)를 통째로 무시.** refAttitude는 죽은 PID경로(Demux1/Sum19)로 흘러 아무 효과 없음. tau_pitch는 오직 GeometricController(항상 R_des=eye)서만 나옴 ⇒ 위치 오차가 있어도 드론이 기울질 못해 XY 제자리 못감. **요·0.24 외란값과 무관.** (앞선 '요각도 끊겨서' 설명은 부분 원인일 뿐, 근본은 R_des=eye.)
- 3DOF는 평면(요 축 없음)이라 이 요 문제 원리적으로 못 봄. "3DOF서 됨"=피치만 맞다는 뜻.
- **플립카운트 올바른 법:** DCM `unwrap(atan2(R(1,3),R(3,3)))` 또는 R(3,3) 0-교차수. body q(pitch rate) 적분은 3D회전서 뻥튀기(헛값).
- **도구:** `monitorFlip.m`(수동), `flipGraph.m`+`StopFcn='flipGraph'`(Run 때마다 자동 그래프: 회전각/기울기/회전속도, DCM기반 짐벌락無, 검은배경, 창 재사용). ⚠️ StopFcn·외란값 다 **session-only(모델 disk 저장 안 함)**. 현재 외란 live=[0 0.24 0] t=2.0/2.1.

### 다음 할 일 = tau_yaw만 geometric으로 교체 (요각도는 유지!)
- **GeometricController에 tau_yaw=tau(3) 출력 추가**(함수 2줄: 시그니처에 tau_yaw + `tau_yaw=single(tau(3))`) → Attitude에 출력포트 추가 → **ControlMixer의 tau_yaw 입력을 Yaw 서브 → Attitude(신 tau_yaw)로 재배선.** 피치·롤 이식과 동일.
- **⚠️⚠️ Yaw 서브의 "요각도" 출력(→XY 위치제어기)은 절대 안 끊음** (안 그러면 위치제어 깨짐 — 위 함정). tau_yaw 경로만 교체.
- **대안(간단): tau_yaw만 freeze** — 뒤집힌 동안(cos≤0) tau_yaw=0. 실험이 사실상 이걸 증명(요게인0=clean). ②freeze 신호 재활용.
- 효과: 뒤집혀도 요오차 회전행렬로 제대로 계산→garbage 안 쏨. R2yaw 유령블록도 이때 정리.
- ⚠️쿼드 요=물리적 최약축, 핵심은 "안새게 방어">"세게 교정". 후속(남으면): kW 강화, freeze 히스테리시스.

### ❌❌ 고도+위치 대공사 전체 폐기 (2026-07-11, 사용자 결정)
**아래 대공사(해결1 BusAssignment 참값주입 + 6번입력 + 해결2 조건부R_des + 참yaw)는 오류나고 지저분해져서 사용자가 통째로 버림.** 현재 열려있는 `flightController`는 그 이전 상태로 돌아가 있음(2026-07-11 MCP 정적점검으로 확인):
- Attitude MATLAB Fn = `GeometricController(R,W,t)`, **R_des=eye(3) 무조건**, refAtt 입력 없음(위치제어 죽음).
- **Yaw 서브 `tau_yaw` Outport 끊김 = 사용자가 일부러 끊은 것(버그 아님! 모델 정상 실행됨).** 요 제어기 출력 살리면 플립시 garbage 요토크→뱅글뱅글, 끊으면 요clean+플립안정화 확인됨(기존 결론). **요는 이미 해결 상태, 건드리지 말 것.** 요각도 출력은 살아있어 위치제어 heading용으로 정상. ⚠️2026-07-11 내가 이걸 "unconnected outport 컴파일에러"라 오독→사용자 분노. tau_yaw 재연결/geometric tau(3) 조언은 폐기.
- BusAssignment 0개(3모델 전부), 6번입력 없음 = 참값주입 흔적 전무.

**앞으로 수정은 사용자가 직접, 나(Claude)는 방법만 제시** ([[feedback-collaboration]]). **요는 이미 해결(tau_yaw 끊김 = 정답).** 진짜 남은 문제 = **위치(XY) + 고도 강인성**, 근본원인 = 컨트롤러가 플립후 오염된 estimator(Z,dz,X,Y,dx,dy) 씀(자세R·ω만 참값UD_R/UD_Omega로 내려감). 참yaw는 이미 있는 R2yaw(참R_meas)로 해결가능=배선0.
- **남은 핵심 = 위치제어:** 자세제어(GeometricController)가 R_des=eye 무조건이라 refAttitude(위치가 만드는 기울기명령) 무시 → 안 기울어 XY 제자리. 해법=조건부R_des `(R,W,refAtt)`, `if R(3,3)>0`→refAtt추종 else eye. refAtt 순서·부호 X스텝 검증필요.
- **고도 강인성(원하면):** equilibrium thrust의 Z,dz 추정→참값 교체. 참X_ned/V_ned는 최상위 Bus Selector(현재 DCM_be,Omega_body 뽑음)에 추가→UD_R패턴대로 UnitDelay→FCS신규입력→controller Demux. ⚠️저번 대공사 지저분함=BusAssignment로 estimator버스 재조립. 이번엔 소비지점 직접 꽂기(BusAssignment 안 씀).
- ⚠️⚠️ 다음 대화 시작시 **사용자에게 지금 실제로 뭐가 안 되는지(고도?위치?) 먼저 물을 것.** 정적점검만으로 추측 말 것(2026-07-11 헛손질 교훈).

### 🔬 시뮬 실측 진단 + 확정 수정계획 (2026-07-11) ⭐⭐⭐
**드디어 돌려서 로그로 확인함(추측 아님).** 시나리오: 시작(57,95), 타겟 `pos_ref=[0,0,-3]`(=원점, 고도3m), 외란 `[0 0.24 0]`@5.0/5.1s, 플립@5.36s→179°, StopTime40, ode3. 그래프 `scratchpad/flip_diag.png`.
- ✅ **플립: 완벽.** 179°까지 뒤집혔다 2°(수평) 복귀. 요 안 튐 → **tau_yaw 끊은 게 제대로 작동(재확인, 건드리지 말 것).**
- ❌ **고도: 안 됨.** 타겟3m인데 3↔7.7m로 감쇠없이 출렁, 끝 7.2m. 안정 안 됨(발산까진 아님).
- ❌ **XY: 안 됨.** (57,95)→(74,96), 원점서 오히려 멀어짐.

**근본원인 (로그 증거 기반, 앞선 내 설명 정정):**
- **XY 진범 = R_des=eye (yaw 아님!).** 실측: 플립후 Vx≈2.8→2.2m/s로 날아가는데 **tilt≈0~2°(안 기울임)**. 즉 자세제어가 R_des=eye로 박혀 refAttitude(위치명령) 무시→**위치권한 0→플립잔여속도로 코스팅**. tilt≈0가 "위치제어가 아예 안 움직임"의 증거. ※추정yaw쓰레기는 위치권한 생겨야 나타나는 **2차문제**(방향).
- **yawEst 완전파괴 (PROVEN):** 참yaw 56° vs 추정yaw **−236,000°**(−4119rad, t≈6s서 절벽). 플립이 estimator yaw 무한감김. → 위치권한 생기면 이 쓰레기 yaw로 방향 틀어짐 → 참yaw 필수.
- **고도 오염 = estimator Z/dz 추정(강한 정황, 미직접검증):** yawEst 완파+기존 "dz부호반대" 진단으로 거의 확실하나, **est Z/dz는 직접 안 찍음. 다음 실측때 estimator Z/dz 로깅해 확정할 것.**
- **자세만 멀쩡** = 참 R_meas(UD_R) 쓰기 때문. estimator는 위치·속도·yaw 다 오염.

**확정 수정계획 (사용자 직접, 나는 방법만):**
1. **위치권한 = 조건부 R_des** — `GeometricController(R,W,refAtt)`, `if R(3,3)>0`→refAtt로 R_des `else`→eye. **이거 없으면 XY 영원히 안 됨**(참값 줘도 안 기울어서 소용없음). refAtt 순서(pitch=1,roll=2 추정)·부호 X스텝 검증.
2. **참값 주입** — controller가 고도(Z,dz)·XY(X,Y,dx,dy)를 estimator 대신 참값으로:
   - 참X,Y,Z,dx,dy,dz = 최상위 Airframe `States.X_ned`(위치)·`V_ned`(속도, NED). **UD_R 패턴 복제**: 최상위 `Bus Selector`(현재 DCM_be,Omega_body 뽑음)에 X_ned·V_ned 체크추가→`Mux`(6)→`Unit Delay`(0.005)→FCS 신규입력→controller에서 `Demux`(6). **BusAssignment로 estimator버스 재조립 안 함(저번 지저분 원흉 회피).**
   - 참yaw = **이미 controller에 있는 R_meas로 계산**(유령 `R2yaw` 블록 재활용, 배선0). XY블록의 추정yaw를 이걸로 교체.
3. **단계**: 참값배선(딱1회) → **고도**(equilibrium thrust의 Z,dz 참값교체·테스트) → **조건부R_des+XY**(X,Y,dx,dy,yaw 참값교체·테스트).

### ✅✅ Step1(참값배선)+Step2(고도) 구현 완료 (2026-07-11) ⭐⭐⭐
**사용자 직접 배선, 컴파일+시뮬 통과. 고도 잡힘.**
- **Step1 참값 배선 완료(4층, UD_R 패턴 복제):** 최상위 `Bus Selector1`(Airframe서 X_ned,V_ned) → `Mux`(in1=X_ned,in2=V_ned=[X,Y,Z,dx,dy,dz]) → `Unit Delay`(0.005,IC0) → `FCS` 6번입력. FCS(flightControlSystem)에 Inport `In1`(6) → inner `Flight Control System` 서브 Inport `In1`(6) → `controller`(=flightController) 6번 → flightController 안 `Flight Controller/Demux`(Outputs=6). **타입=double**(estimator세계 double로 풀림, single하면 컴파일 실패—사용자가 double이 맞다고 정정). ⚠️함정2개(사용자가 겪음): ①DTC=single 넣으면 FCS경계서 double기대 에러→DTC 삭제(전부 double). ②Mux 순서 뒤집힘(V_ned먼저)→스왑해 X_ned먼저.
- **Step2 고도 완료:** flightController `Flight Controller/gravity feedforward//equilibrium thrust` 안 `Bus Selector6`(estimator Z,dz) 대신 참값 주입. equilibrium thrust에 Inport `Z_true`,`dz_true`(port5,6, type double) 추가→`Sum3 in2←Z_true`, `D_z←dz_true`로 교체. Demux `out3(Z)→Z_true`, `out6(dz)→dz_true`. ⚠️함정: 처음 `dz_true`에 out4(dx) 잘못꽂아 시뮬 광란→out6(dz)로 수정하니 정상. ADRC(죽은블록)로 가는 Z가지는 방치OK. **freeze로직=`flipWindow(R): inflip=R(3,3)<=0`→Switch(u2>=0.5?w0:SaturationThrust), 즉 뒤집히면 추력 호버w0 고정(확인).**
- **✅고도 결과:** 참 Z·dz 쓰니 **3m 수렴**(전엔 3↔7.7 발산). 안착후(외란 t=12) 플립시 고도 3.0→2.4m 추락후 3m복귀=**물리 정상**.
- **🔬 "플립시 고도추락 없다" 의문 해소:** 외란 t=5.0이면 드론이 아직 3m로 상승중(Vz≈-3m/s 위로)이라 상승관성이 추락 가림→안떨어지고 오히려 뜸. **외란을 안착후(t=12,Vz≈0)로 미루니 추락 0.6m 정상 발현**. 시뮬 맞음(사용자 물리직감 정확, 원인은 외란타이밍). ⚠️외란시점 세션에서 t=12로 바꿔둠(원상복구 t=5 필요).

### ✅✅✅ 요(Yaw) 기하제어 완료 (2026-07-11) — 4모델 전부 disk 저장됨!
**GeometricController에 tau_yaw=tau(3) 추가로 요 완전히 잡힘.** 구현:
- `flightController/Flight Controller/Attitude/MATLAB Function`(=GeometricController) 코드: 시그니처 `[tau_pitch,tau_roll,tau_yaw]=GeometricController(R,W,t)`, 끝에 `tau_yaw=single(tau(3));` 추가(tau는 원래 3벡터 다 계산했고 tau(3)만 버렸던 것).
- Attitude 서브에 Outport `tau_yaw`(3번) 추가→MATLAB Fn 3번출력 연결. Flight Controller층서 `Yaw`블록 out2(tau_yaw)→`ControlMixer` in3 선 삭제, `Attitude` 신 tau_yaw→ControlMixer in3 재배선. **Yaw out1(yaw각도)→XY는 유지**(heading용).
- **구조 발견:** Yaw 끊긴 곳=Yaw 서브 **안쪽** tau_yaw Outport 입력이 빈 것(=0 출력)이었음(Yaw out2→ControlMixer 선 자체는 살아있었음). 초기 참yaw=0°라 R_des=eye 요목표0과 일치=문제없음.
- **✅결과(실측):** 플립 179° 완주+복귀, **요각 최종 0.0°**(플립후 −10.8~1.0° 범위, 전엔 방치되어 흘렀음), 요속도 최종 0.000rad/s(전 ~0.4). 뒤집혀도 회전행렬로 계산해 atan2 garbage 안씀=팽이 안됨.
- **💾 asbQuadcopter/flightControlSystem/flightController/nonlinearAirframe 4개 다 save_system 완료(2026-07-11).** 외란 t=5/5.1·StopTime40 원복 후 저장. 이제 세션 잃어도 Step1+2+요 다 보존됨. (⚠️asbQuadcopter는 parameterized library link 경고 뜨나 무해·기존특성.)

### ✅✅✅✅ XY 위치제어 완성 = 제자리 플립복구 검증 성공 (2026-07-12) ⭐⭐⭐⭐
**핵심 마일스톤: "제자리 호버 중 외란→플립 1번 완주→제자리 복귀" 달성.** 오늘 A+B+C+제자리목표 다 맞물림. ⚠️**전부 세션상태, disk 저장 아직 안 함(다음 대화 시작시 저장됐는지 확인!).**
- **A. 조건부 R_des** (GeometricController): `(R,W,refAtt)`, `R_b2e=double(R'); if R_b2e(3,3)>0 && norm(double(W))<5.0`→refAtt로 R_des(Ry(p)*Rx(r), p=refAtt(1),r=refAtt(2)) `else`→eye(3). tau_pitch/roll/yaw 3출력. in3←Digital Clock 끊고 Attitude refAttitude Inport서 가지. ⚠️if/else R_des 타입 double 통일 필수.
- **⚠️각속도 게이트 `norm(W)<5.0`는 하드코딩(임시)** — 사용자가 정확히 지적. 플립직후 각속도 큰데(20) R33>0 되자마자 위치제어 켜지면 재발산 막으려 넣음. **근본 아님. 제자리선 재플립 안 나니 이 게이트 필요성 재검토(빼고 돌려볼 것). 남기면 ω_crit 물리량 연계or히스테리시스로 승격.**
- **C. 참 yaw** (Yaw/MATLAB Function=R2yaw): `Rb2e=double(R)'; yaw=atan2(Rb2e(2,1),Rb2e(1,1))`. ⚠️**`single`→`double` 필수**(참XY 넣으면 XY연산 double통일→yaw single이면 "Inferred single vs backprop double" 컴파일에러. 고도Step2 "전부double" 함정과 동일). Bus Selector5(estimator오염 yaw)→yaw Outport 선 끊고 R2yaw출력 연결(ADRC·Sum1 가닥은 놔둠).
- **B. 참 X,Y,dx,dy 주입** (진짜 근본, estimator XY 완전오염이 진범—사용자가 "xy추정 이상해서 발산 아니냐" 정확히 지적. estimator X는 t=4 플립전에도 참57 vs 추정0, t=40 추정-142 발산). 배선: XY-to-reference-orientation 서브에 Inport 4개 `real_x,real_y,real_dx,real_dy`(double) 추가 → FC층서 참 Demux `out1(X)→real_x, out2(Y)→real_y, out4(dx)→real_dx, out5(dy)→real_dy`(Step1의 방치된 참값, terminated였음) → XY내부 `Mux.1←real_x, Mux.2←real_y, Mux1.1←real_dx, Mux1.2←real_dy`(기존 Bus Selector1/2=estimator 끊고). Bus Selector1/2·states_estim입력 놀게됨. ⚠️states_estim통째교체는 버스라 BusAssignment필요=폐기방식, 실쓰는4개만 직접이 최소.
- **❌ Saturation(refAtt 포화)은 폐기** — 내가 "P항 0.72 포화=110m라서"라 오진하고 pitch_roll_cmd에 Sat[-0.5,0.5] 제안. 사용자가 "임시방편이잖아, xy추정 이상한거 아니냐" 정확히 반박. 진짜=estimator XY오염(B로 해결). Sat 안 씀.
- **제자리 목표 (원점비행→호버 전환)**: 진짜 마지막 열쇠. 목표가 `posXY=(0,0)`인데 드론 시작 `init.posNED=(57,95)`=110m 미스매치 → 위치제어가 "원점으로 날아가!"를 세게 물어 플립과 겹쳐 **플립 3번+호버실패**(참XY 넣으니 위치제어가 제대로 작동해서 오히려 드러남). ⚠️init.posNED 바꿔도 시작위치 안변함(IC가 posNED 아닌 딴데, 미해결). posref는 출력로그(FromWorkspace 없음). **해결=posXY 목표 소스(FC층 Selector1) 끊고 `Constant HoverTarget=[57;95]`(시작위치=X,Y순서, double) 주입** → 위치오차0 → 진짜 제자리호버.
- **✅결과(실측)**: 플립진입 **1번**(179°,순회전-359°=한바퀴), 제자리 **최대이탈 X0.89m/Y0.05m**, 고도3.00m, tilt 0~1°, |W|=0 완전정착. 그래프 scratchpad/flip_hover_success.png.
- **⚠️HoverTarget[57;95]는 임시 진단상수**(시작위치 하드코딩). 정식화=목표를 시작위치 동적추종 or 제자리미션 확정 필요. 원래 (0,0)미션 복원하려면 Selector1 재연결.
### ✅ disk 저장 완료 + 토크 스윕 + 게이트 발견 (2026-07-12) ⭐⭐
- **💾 저장 완료:** `flightController`(조건부R_des+게이트+R2yaw double+XY참값배선real_x/y/dx/dy+HoverTarget[57;95]+참값Demux연결) / `nonlinearAirframe`(외란) / `asbQuadcopter` 다 save_system·dirty=off. 외란 `[0 0.24 0]` t=5/5.1, StopTime40 복원 후 저장. flightControlSystem·stateEstimator 변경없음. (⚠️asbQuadcopter parameterized library link 경고=무해·기존특성.)
- **🔬 토크 스윕(pitch축 [0 A 0] 펄스, 게이트버전, 제자리 목표):** A≤0.1 플립X(42°되돌림)·안정 / **A=0.2~0.3 단일플립완주+제자리복구✅(이탈1.4~2.2m)=sweet spot** / A=0.4 완주하나 복구실패(최종tilt15°·|W|0.51·이탈10.8m) / A=0.5 이탈28m·|W|0.82. **⇒현 컨트롤러 안정화 한계 ≈ A=0.3~0.4.** A0.4+무너짐 원인=큰각속도로 게이트(|W|<5)오래걸려 위치추종 오래꺼짐→병진 드리프트 누적. B로 개선여지.
- **⚠️⚠️ 각속도게이트 못버림(중요 발견):** 게이트 빼고 제자리 돌리니 제자리·고도·안정은 OK(이탈2m)인데 **플립이 완주(-359°)→146°되돌림(-3°)으로 바뀜.** ⇒게이트가 재발산방지뿐 아니라 **플립 완주도 보조**(각속도 클때=플립회전중 위치추종 꺼서 momentum 유지). 위치추종 조기개입이 완주 방해→되돌림. **그냥 못버림, 복원함.** 각속도조건=반창고 아니라 "플립 진행중 판별" 핵심로직.
- **다음=B 상태머신 (사용자·나 합의, PX4식 게이팅):** 하드코딩5 대신 `persistent inFlip` 상태. **진입=tilt>90°(R33<0), 종료=tilt<30° AND |W|작음 (히스테리시스=진입/종료 임계 다름)**. inFlip이면 위치추종·요 등 "뒤집히면 쓰레기값 되는 것들" 다 off. PX4가 특정각도 넘으면 센서융합 게이팅하는 것과 동일 개념(사용자 지적). 넣고 A0.4~0.5 복구 개선되는지 재스윕. 대안A(물리임계 ω_crit연계)보다 B가 근본적(순간판정→상태관리).
- **손/벽 외란(다음):** 지금 pitch축 [0 A 0] 단일펄스 → 손치기/벽충돌 느낌 = **짧고 강한 랜덤3축 토크임펄스 [Ax Ay Az](방향 무작위)**. 자세붕괴 표현. 병진힘은 자세붕괴가 핵심이면 불필요. 이걸로 재스윕.
- **estimator 재작성 (사용자 MEMORY.md 확정, 별건 대작업):** 참값치트(real_x/y/dx/dy 주입+HoverTarget) 걷어내고 **PX4식 쿼터니언 상보필터**(attitude_estimator_q/Mahony) 차용. 해결: ①좌표계문제(내가 참값을 절대NED로 넣어 목표=estimator상대좌표와 불일치→110m 가짜오차. 원래 estimator 상대좌표라 호버 잘됐던 것을 참값 절대주입이 깨뜨림) ②yawEst완파(오일러 짐벌락→쿼터니언이면 해결) ③가속도 1g게이팅(플립중 accel보정 스킵). geometric 컨트롤러는 유지(PD+ω×Jω라 플립 유리).

### 🔜 다음 대작업 = State Estimator 재작성 (참값주입 걷어내고 쿼터니언 추정기) (2026-07-12 결정) ⭐⭐⭐
**결정: 지금까지 참값주입(UD_R 자세 + Step1/2/B의 위치·속도·yaw 참값)은 "치트". 물리적 정당성(연구 완성도) 위해 진짜 state estimator를 만들어 참값주입을 대체할 계획.** 사용자 선택 (A)=추정기 새로 제작.
- **현재 문제 2가지 (사용자 인지):** ①Mambo 템플릿 기본 추정기가 **오일러각 기반**이라 플립(90°)서 짐벌락+각도 wrap으로 터짐 (yawEst 완파 PROVEN: 참56° vs 추정−236000°). ②플립 격동 중 가속도 오염으로 추정치 쓰레기.
- **PX4 아이디어 차용** (소스: `PX4-Autopilot/src/modules/attitude_estimator_q/attitude_estimator_q_main.cpp`, Mahony식 쿼터니언 상보필터):
  1. **자세 상태를 쿼터니언으로 적분** (오일러 아님): `_q += _q.derivative1(corr)*dt; _q.normalize()` (:557). 오일러각(roll/pitch/yaw)은 **로깅/표시용으로만 맨끝 변환**, 적분 루프엔 절대 안 넣음 → 짐벌락·wrap 원천 제거. **이게 yawEst 완파의 근본 해결.**
  2. **가속도 틸트보정 게이팅** (:534): `|accel|`이 [0.9g,1.1g] 밖이면(=플립 격동중) accel 보정 **스킵**, **순수 자이로 적분만으로 플립 통과**, 끝나고 accel~1g 안정되면 보정 재개. → 기동가속도 오염이 자세추정 망치는 것 방지.
  3. 출력은 R/q로 컨트롤러에 직접(UD_R 패턴 이미 있음), 오일러는 스코프에서만.
- **우리 경로:** Mambo의 오일러 상보필터 → **쿼터니언 상보필터(Mahony/Madgwick)로 교체.** 핵심=아이디어 1+2. 실내라 mag 못 믿음 → heading은 빼거나 vision/optical flow로. PX4 heading 보정도 atan2 쓰지만 그건 heading 스칼라 뽑을 때만이고 자세 상태 자체는 쿼터니언(우리 오일러 방식과 근본 다름).
- **참고: geometric 컨트롤러는 안 엎음** (성능 이유 없음). geometric = PD(Kr·eR=P, Kw·eW=D) + 모델 feedforward(ω×Jω) 구조라 transient(플립)엔 오히려 PID보다 유리. 부족한 건 적분(I)뿐이고 플립 transient엔 안 중요. 컨트롤러 고도화(적분항 등)는 나중.
- ⚠️아직 시작 안 함(계획만). 다음 단계=현재 Simulink에서 estimator 출력이 실제로 어디로 들어가는지 서브시스템 열어 targeting.

**센서 게이팅 (플립 중 위험센서 손절, PX4식) (2026-07-12):**
- **플립 중 센서 거동:** ✅자이로(도는 걸 직접 잼, 자세무관=척추) ✅기압계(압력은 자세무관=유일한 강인 고도원, 느림) / ⚠️가속도계(중력＜기동가속도라 "아래방향" 못찾음—틸트기준용만 위험, PX4 1g게이팅 대상) / ❌소나·옵티컬플로우(하향센서, 기울면 땅 못봄).
- **PX4 rangefinder(1D라이다) 처리 = cos보정 + 45°하드게이트** (소스 `ekf2/EKF/aid_sources/range_finder/sensor_range_finder`):
  1. **보정:** `getDistBottom()=rng*_cos_tilt_rng_to_earth`, `_cos_tilt=R(2,2)`(회전행렬 (2,2)=cos틸트). 즉 수직고도=경사거리×cos틸트.
  2. **게이트:** `isTiltOk()= R(2,2) > _range_cos_max_tilt`, **기본 0.7071=cos(45°)**. 넘으면 `_is_sample_valid=false`→EKF에 융합 안함(quality/tilt/range/stuck/fog 무효화조건 중 틸트가 1급). 버린동안=기압계+관성으로 버팀, 45°밑 복귀하면 재수용(영구비활성 아님=게이팅).
  3. **소나·옵티컬플로우 같은 게이트 공유:** `common.h:427` 주석 명시 "range finder **and flow data**" 같은 cos_max_tilt 하나로 관리. 둘다 하향센서라 틸트에 똑같이 취약.
- **우리 이식:** 고도=소나잰값×R(3,3)보정, `R(3,3)>cos(45°)=0.707`(맘부 소나빔 좁으면 더 보수적 cos30°≈0.87)이면 쓰고 아니면 버림. 게이트신호=추력freeze의 R(3,3) 재활용(틸트게이트 중앙 하나 두고 소나·플로우·accel보정 분기=PX4 구조). 버린동안 기압계+자이로 관성.
- **⚠️옵티컬플로우 "영구 폐기" 미결정:** 사용자가 "지금 옵티컬 안써도 동작 잘됨→버리자" 했으나, **"잘됨"은 참XY주입(치트) 덕분**임에 주의. 실내(GPS없음)선 옵티컬플로우가 **유일한 온보드 수평속도 소스**. 영구폐기하면 참XY 걷어낸 뒤 수평위치가 accel 이중적분으로 무한드리프트→제자리복귀 불가. 선택지: ①플로우 유지+플립중만 게이팅(PX4충실, 실내 위치유지 가능) vs ②플로우 폐기+XY는 외부(OptiTrack/mocap)or참값주입 스코프아웃(자세+고도 복구로 스코프 축소). 미결정, 다음 논의.

### 🔬 Estimator 내부 정찰 완료 (2026-07-12) ⭐⭐⭐ — 재작성 착수 전 지도 (순수 read-only, 모델 미수정)
**stateEstimator.slx(controller/stateEstimator.slx) 구조 다 뜯음(MCP).**
- **구조:** stateEstimator > State Estimator(blk_597) > ①`Complementary Filter`(blk_604)=자세(오일러) ②`EstimatorAltitude`(blk_699)=Z ③`EstimatorXYPosition`(blk_760)=XY ④`SensorPreprocessing`(blk_894). **자세가 ②③으로 흘러들어감**(자세 터지면 고도·XY 도미노).
- **Complementary Filter=오일러 상보필터**(주석 Fabian Riether/pieter-jan). 상태=`Memory`(blk_660)에 오일러**[yaw,pitch,roll]** 저장→`Wbe`(blk_674)로 pqr→오일러변화율 번역→적분→다시 Memory. **Wbe 안 ÷cos(pitch)=90°짐벌락 근본.** 자이로는 Wbe밖 Product(blk_664)서 곱해짐. accel틸트보정=blk_617 **+1g게이트 이미있음**(blk_608/609: 0.9~1.1g). yaw보정=blk_640(yawVIS).
- **✅ Wbe 교과서와 수학적 동일 증명(오차 0.0):** 저장이 [yaw,pitch,roll] 역순 + `Selector[3 2 1]` 뒤집기 2개(blk_670 상태피드백, blk_665 변화율)라 모양만 다름. **팀원 "행렬 이상"·"적분순서 안맞음" 지적=버그아님, 특이한 저장순서.** 근데 맞아도 오일러라 90°폭발=교체대상. (MATLAB로 배선복원 vs 표준 Wbe 수치검증함.)
- **게이팅 현황(중요):** 고도 `OutlierHandling`(blk_718)=**자세입력 無→틸트게이트 없음(추가필요).** 소나튐·기압불일치 outlier만 봄. / XY `DataHandling`(blk_795)=**이미 틸트게이트 있음**(maxp/maxq: |pitch|,|roll|≤`ofPitchRollMax` + 각속도 `ofPQMax`/`ofDPQMax` + `minHeightforOF`). 주석"prevents optical flow with large angles/rates". **XY는 추가 아니라 검증·튜닝만.**
- **비전(posVIS)=확정 꺼짐:** 최상위 블록이름 literally `_DUMMY_FLAG_usePosVIS`/`_DUMMY_posVIS`, `Sensors.NO_VIS_X/NO_VIS_YAW` 표식으로 채움→validVIS 항상 false→yawVIS 보정 원래부터 안걸림. 씬=`Camera (Airport)`=**실외.**
- **⭐mag 상태:** estimator 입력버스 `sensordata_t`에 **mag필드 없음**(ddx/ddy/ddz,p/q/r,altitude_sonar,prs,vbat뿐)→**그래서 템플릿이 yaw를 vision으로 잡은 것.** 근데 **sim은 mag 생성함**: `Environment/Magnetic Field` + `.../HAL_acquisition_creator/HAL_magn_mG_t_creator1`(`HAL_magn_mG_t`={x,y,z}). **"mag 있는데 estimator에 미배선."** 실외라 mag 정당. **⇒yaw=드리프트 감수 아니라 mag배선+mag보정으로 드리프트까지 해결가능.** ⚠️creator가 live신호인지(환경자기장→body회전) 구현시 트레이스 확정 필요.
- **⚠️옵티컬플로우는 yaw(heading) 못 줌**(수평속도만). 비전 버리면 yaw 절대기준=0개(자이로뿐). mag가 유일 해법.
- **PX4 소스(읽음):** `~/PX4-Autopilot/src/modules/attitude_estimator_q/attitude_estimator_q_main.cpp`(1.14도 있음). `update()`=**L463~571**(코어). 쿼터니언적분 `_q+=_q.derivative1(corr)*dt`+normalize=**L551~560**(Wbe대체·나눗셈없음). accel틸트보정+**1g게이트=L535~538**(0.9~1.1g). mag yaw보정=**L501~513**(mag_earth=q.rotate(mag)→atan2→corr). vision heading=L480~498. `init_attitude_q`(accel+mag로 부팅)=L424~461. **구조=corr(회전율보정)에 [heading오차+tilt오차(게이트)+자이로] 다 모아 한방 적분.** rangefinder 틸트게이트: `ekf2/EKF/aid_sources/range_finder/sensor_range_finder` .hpp **L115**(getDistBottom=rng×cos틸트)/**L132**(isTiltOk=cos_tilt>max)/**L164**(max=0.7071=cos45°, 주석"range finder and flow data"), .cpp L60(cos_tilt=R(2,2))/L63~90(updateValidity, L81 게이트적용).
- **확정 스코프:** 1)Complementary Filter 오일러→쿼터니언(Mahony corr구조, Wbe삭제) 🔴대공사 2)출력 q→euler(소비자)+q→R(컨트롤러) 3)고도 틸트게이트 **추가**(R33>cos45°) 4)XY 게이트 **검증** 5)**mag 배선+mag yaw보정 추가**→yaw해결(실외) 6)참값치트(UD_R/Step1/2/B) 제거. **컨트롤러(SO3 geometric) 안 엎음**(q→R 공짜·특이점없음, 쿼터니언·R은 쌍둥이). **사용자 개념이해 완료**(frame vs 표현법, Wbe 유도, 짐벌락=북극경도, Mahony corr).

### 🎛️ 컨트롤러 상태머신 + 게인분석 + 대각선외란 진단 (2026-07-12, 세션2) ⭐⭐⭐
**작업 병렬화 확정:** estimator(1)=팀원, 컨트롤러(5)·외란(4)=사용자. 컨트롤러는 **참R(UD_R치트) 위에서 도라 estimator와 독립** — 지금 튜닝 가능. 파일 안 겹침(estimator=stateEstimator, 외란=nonlinearAirframe, 컨트롤러=flightController).

**① inFlip 상태머신 구현 완료 (norm(W)<5 대체):** `GeometricController`(Attitude/MATLAB Function) 게이트를 순간판정→**래치+히스테리시스**로. `persistent inFlip`, 진입=`|W|>W_enter(5) ‖ R33<0`, 해제=`|W|<W_exit(1) && R33>0.87(30°)`. **순수피치 플립서 잘 됨(사용자 확인).** 히스테리시스=진입/해제 문턱 달라 채터링·조기복귀 방지. **⚠️값 5/1/30°는 아직 하드코딩** — 사용자 정확히 지적. **승격안: 진입 `|W|>ω_crit(θ)=√(2·a_max·(π−θ))`(a_max=394, 제동거리 v²=2as 물리, "90→180 완주 예측"), 해제 tilt=45°(=센서게이트·PX4 rangefinder값과 일치). 진입 R33<0(90°)은 이미 물리적.** 미구현(제안만).

**② 게인 분석 (kR=0.06, kW=0.006 = 야매지만 안전구역):** 소각선형화 자세루프 `J·θ̈+kW·θ̇+kR·θ=0` = 2차. `ωn=√(kR/J)`, `ζ=kW/(2√(kRJ))`. 현재: roll ωn32/ζ1.60, pitch ωn29/ζ1.45, yaw ωn24.5/ζ1.22 — **다 과감쇠(굼뜸), 스칼라라 축마다 제각각.** 튜닝식 `kR=J·ωn²`, `kW=2ζωn·J`(축별 가능). 플립복구엔 ζ≈0.7~1.0(현재 1.5 굼뜸). **⚠️게인은 정착만 지배, 플립자체엔 무관(포화).** 전달함수: 플랜트=`1/(Js²)`(이중적분), PD=`kR+kW·s`, 폐루프 특성방정식 `Js²+kWs+kR=0`의 근=극점. **소각근사라 플립엔 전달함수 자체 무효.**

**③ 대각선 외란 구현 + 진단 (⭐핵심 발견):** `nonlinearAirframe/Nonlinear`의 `Step`−`Step1`(0.1s 사각펄스)의 `After`를 `[0 0.2 0]`(순수피치)→**`[0.15 0.15 0]`(roll+pitch, 벡터=[roll,pitch,yaw]).** **yaw축 0인데 yaw가 깨어남 = ω×Jω 커플링**(Jx≠Jy라 롤·피치 동시회전→`(Jy−Jx)pq` yaw토크 자동발생, 포화라 피드포워드 못지움). 실측: 플립 167°완주, p48/q38/**r−16(yaw깨어남✅)**. **근데 플립후 27초 링잉**(위치 X±3.5/Y±5.1m, tilt 정착 t=32.5s). 진동주기 **3.49s=1.8rad/s=위치루프**(자세루프 29rad/s 아님). **원인=yaw가 위치제어 오염:** 대각선이 yaw 168°스윙시킴→위치제어가 heading으로 "어느쪽" 계산하는데 yaw틀어져 엉뚱한쪽 기울임→위치 뱅글. **순수피치는 yaw 안깨워서 이문제 숨었던것**(=대각선 테스트가 약점 폭로=성공).

**④ ❌yaw게인 2배 실험 실패 (내 처방 틀림, 정정):** 축별 `kR=[.06;.06;.12] kW=[.006;.006;.012]` + `.*`로 바꿔 돌림 → **더 나빠짐**(정착40s vs 32.5, X8.4 vs3.5, yaw279° vs168°). **왜:** ①yaw액추에이터 포화(쿼드 yaw토크≪roll/pitch, 게인↑해도 토크 안나옴) ②4모터 예산 도둑질(yaw가 더뺏어 roll/pitch 복구악화). **⇒ 루트로커스(선형)는 "yaw극점−12.7 빠름 문제없음" 했는데 실제 실패 = 사용자 "비선형기동엔 s-plane 무의미" 실험증명.** 원복함(스칼라).

**⑤ 진짜 원인·해법 (게인 아님):** yaw 물리한계=**약한 액추에이터 authority(하드웨어, 못고침)** + 위치-yaw 커플링. **but 복구는 됨(느릴뿐, 발산아님).** 해법후보: (a)위치제어를 yaw정착중 부드럽게/홀드(증상완화, inFlip확장) (b)외란 스윕으로 깔끔복구 envelope 정량화 (c)외란축소(회피). **"컨트롤러 고장 아니라 드론 yaw물리한계 만난것."**

**⚠️세션 워크스페이스 오염 교훈:** 분석중 `g=@(n)...`(중력g), `m=t>8`(질량m) 덮어써서 `w0=-g*mass` 시뮬 크래시. 복구 `assignin('base','g',9.81)`,`m=0.063`. **모델 파라미터명(g,m,J,kR...)과 분석변수명 겹치지 말것.** 시뮬은 세션변경(script edit via `sfroot`.find EMChart `.Script`)이라 disk 미저장 — 다음세션 확인.

### (구·완료됨) XY Part A (조건부 R_des) 안내 원문 — 참고보존
**사용자 요청: 다음에 XY Part A를 지금과 똑같이 안내.** XY 3부분(A조건부R_des·B참X,Y,dx,dy·C참yaw)인데 **A 먼저 하고 테스트**(제일 중요 레버).
- **XY 아키텍처(확인됨):** `XY-to-reference-orientation`(in1=posXY, in2=states_estim3=오염X,Y,dx,dy, in3=yaw←Yaw블록) → `pitch_roll_cmd` → `Switch_refAtt`(u2>thr, in1=XY명령/in3=Selector2 택1, takeoff스위치) → `Attitude` in1(refAttitude). Attitude 안 refAttitude Inport는 **죽은 Demux1/Sum19**로 감(GeomCtrl가 R_des=eye라 버림=XY안됨 진범).
- **Part A 구현법(다음 대화 그대로):**
  1. GeometricController 코드를 **조건부**로: 시그니처 `(R,W,refAtt)`, `R_b2e=double(R'); if R_b2e(3,3)>0` → `p=double(refAtt(1)); r=double(refAtt(2)); Ry=[cos(p) 0 sin(p);0 1 0;-sin(p) 0 cos(p)]; Rx=[1 0 0;0 cos(r) -sin(r);0 sin(r) cos(r)]; R_des=Ry*Rx;` `else R_des=eye(3); end` 나머지(eR,eW,tau, tau_pitch/roll/**yaw** 3출력) 동일. ⚠️**if/else R_des 타입일치**(double 캐스팅)—전에 여기서 파싱에러남.
  2. 배선: `MATLAB Function` in3 ← 현재 `Digital Clock`(안쓰던 t). 그 선 삭제, Attitude의 `refAttitude` Inport에서 가지쳐 in3 연결(죽은 Demux1/Sum19가지는 놔둠).
  3. 컴파일+40s. **볼것: 플립전(t<외란) 드론이 (57,95)→원점쪽 X,Y 줄며 기울여 이동**. ⚠️반대로가면=refAtt 부호/순서 문제→`p↔r` 스왑or부호반전(돌려야 확정, 미검증).
- **Part B(A 성공 후):** 참 X,Y,dx,dy를 XY블록에 주입. `XY-to-reference-orientation` 안 `Bus Selector1(X,Y)`·`Bus Selector2(dx,dy)`가 states_estim서 뽑는 걸 Demux out1,2,4,5로 교체(Step2 고도와 동형: XY서브에 Inport추가→Demux배선→BusSelector출력 자리 교체).
- **Part C:** 참 yaw를 XY블록 in3에 주입(현재 Yaw블록 out1=오염yaw). 참yaw=R_meas로 계산(R2yaw 유령블록 재활용 or atan2(R(2,1),R(1,1))).

**도구 메모(재현용):** 신호로깅=포트핸들에 `set_param(porthandle,'DataLogging','on')`(라인핸들 아님). `sim`은 SingleSimOutput off라 **tout를 double로 반환**(거대덤프 주의, out=double), logsout은 **base워크스페이스**에 저장. 이미 로깅된 것들: States(X_ned/V_ned/DCM_be/Euler...), yawEst, rEst, DCMlog, Wlog. StopFcn=flipGraph(session-only). 그래프 검은배경 `exportgraphics(f,fp,'BackgroundColor','k')`. ⚠️모델 disk 저장은 안 했음(session만).

### (폐기됨·기록보존) ⭐ 고도+위치 대공사 (2026-07-10)
**모델 아키텍처(중요, 이름 다 헷갈리게 돼있음):** `asbQuadcopter`(top) → `FCS`(ModelRef=**flightControlSystem**) → `Flight Control System` 서브 안에 **`estimator`(=stateEstimator 모델)** + **`controller`(=flightController 모델)**. estimator가 원시센서로 statesEstim_t(X,Y,Z,yaw,pitch,roll,dx,dy,dz,p,q,r 스칼라들) 만들어 controller로 줌. **최상위 Sensors 출력=원시센서버스(VisionSensors/HALSensors/SensorCalibration), 추정치 아님.** 참R(R_meas)·참ω(Omega_meas)는 UD_R/UD_Omega로 이미 controller까지 내려가 자세제어가 씀(그래서 tilt는 회복). 근데 고도·위치·yaw제어는 **estimator(statesEstim_t) 씀** → 플립이 estimator 박살냄.

**진단:** 360°플립 후 stateEstimator가 **연직속도 dz 부호까지 반대**(참-2.2 vs 추정+1.5), **yaw는 수백~수천도 오차**(참12° vs 추정-823°). ⇒ 고도PD가 반대로 제동해 폭주, XY명령이 garbage yaw로 회전돼 방향 틀림. **자세(R_meas 참값)만 멀쩡.**

**해결1=참값 주입 (구현·검증완료 for 고도):** estimator→controller 선에 **Bus Assignment**로 참값 덮어씀. 배선:
- 최상위: `Bus Selector1`(Airframe States서 V_ned,X_ned,DCM_be 뽑음) → `Mux`[X_ned(3),V_ned(3),yaw(1)]=7폭. `TrueYaw` MATLAB Fn(`yaw=atan2(R(1,2),R(1,1))`, R=DCM_be) → `Data Type Conversion`(single, ∵FCS입력 single인데 Airframe참값 double) → `Unit Delay`(0.005,IC0, ∵대수루프 차단-UD_R와 동일이유) → FCS 6번입력.
- 깊은곳(Flight Control System): `PosVel_true` Inport → `Demux`(7) → `Bus Assignment`(estimator출력 버스에 assign) AssignedSignals=`X,Y,Z,dx,dy,dz,yaw`.
- ✅고도 완벽(3m 정착). ⚠️Bus Assignment는 controller가는 가지에만(landing/crash/logging은 원시추정 유지).

**해결2=조건부 R_des (구현완료):** `GeometricController(R,W,refAtt)`로 변경. `if R_b2e(3,3)>0`(수평)→refAtt=[pitch;roll]로 R_des=Ry(p)*Rx(r) / `else`(플립중)→eye(3). refAttitude(원래 Demux1로 죽어있던 것)를 GeomCtrl 3번입력에 연결(Digital Clock t 교체·t는 함수서 안썼음). ⚠️**Stateflow 함정: if/else의 R_des 타입 일치 필수** — refAtt=single이라 if가지 R_des=single, else의 eye=double → 파싱에러. **double() 캐스팅으로 통일**(R_b2e=double(R'), p=double(refAtt(1)) 등)해서 해결. (한글주석 문제 아니었음.)

**현재 상태(2026-07-10 저장시점):**
- ✅ 고도 3m 정착, ✅ 자세 tilt 회복, ✅ 플립 완주(178°,-360°)
- ✅ 위치제어 **반응 시작**(전엔 완전정지 (57,95)고정 → 이제 원점방향 기울여 이동, 부호맞음)
- ❌ 아직 깔끔수렴 안 함(중간서 헤맴)=**추정 yaw 오염이 원인**(사용자 yaw직감 맞았음)
- 🔧 **방금 참yaw 주입 배선 마침, 미테스트.** 다음=컴파일+시뮬 돌려 XY수렴 확인, **틀리면 TrueYaw 부호 뒤집기**(atan2(R(1,2)..)↔atan2(R(2,1)..) 또는 -). yaw로도 부족하면 p,q,r도 주입. 큰기울기/게인 튜닝 남을수 있음.
- 시나리오: 드론 시작 (57,95), 명령 (0,0) = 110m 떨어진 가혹한 테스트. 외란=[0 0.24 0], 플립 t≈5.2s, StopTime 40.

### (구버전) 위치제어 해법 = 조건부 R_des (미구현, 2026-07-10)
- **아이디어:** 자세가 수평 근처(R_b2e(3,3)>0)면 refAttitude(피치/롤 명령) 따라 기울이고, 뒤집힌 동안(≤0)엔 eye(3) 고정(플립 방해 안 함). step①의 "항상 eye"를 조건부로.
```matlab
function [tau_pitch,tau_roll] = GeometricController(R, W, refAtt)
  kR=0.06; kW=0.006; J=diag([5.82857e-5,7.16914e-5,1e-4]);
  R_b2e = R';
  if R_b2e(3,3) > 0                 % 수평 근처: 위치명령대로 기울임
    p=refAtt(1); r=refAtt(2);
    Ry=[cos(p) 0 sin(p);0 1 0;-sin(p) 0 cos(p)];
    Rx=[1 0 0;0 cos(r) -sin(r);0 sin(r) cos(r)];
    R_des = Ry*Rx;
  else                              % 플립 중: 수평 고정
    R_des = eye(3);
  end
  W_des=[0;0;0];
  eR=vee(0.5*(R_des.'*R_b2e - R_b2e.'*R_des));
  eW=W - R_b2e.'*R_des*W_des;
  tau=-kR*eR - kW*eW + cross(W,J*W);
  tau_pitch=single(tau(2)); tau_roll=single(tau(1));
end
```
- **필요 배선:** refAttitude(현재 Term_ref로 죽음)를 GeometricController 새 입력으로 연결.
- **⚠️ 검증필요:** refAtt 순서가 `[pitch;roll]`인지, 부호가 맞는지 돌려봐야 확정.
- **미해결 결정(사용자에게 물을 것):** 위치제어 먼저 vs 요 tau(3) 교체 먼저 — 사용자 아직 선택 안 함.

## 파라미터 (Mambo, asb 모델 실측)
m=0.063 kg / J=diag([5.82857e-5, 7.16914e-5, 1e-4]) (Jyy=7.16914e-5) / g=9.81 / L(피치암)=0.0441 m / fmax=0.3266 N·fmin=0.0065 N/모터 / T/W=2.11 / τ_max=0.0282 N·m / a_max=394 rad/s² / 게인 kR=0.06 kW=0.006 / 제어 200Hz(Ts=0.005, ode3).

## 참고 논문
- ★ **Lee, Leok, McClamroch (2010)** "Geometric tracking control of a quadrotor UAV on SE(3)" — 제어기 뼈대, almost-global, upside-down **되돌림** 복구(플립완주는 안 다룸=우리 빈칸).
- **Antal et al. (2024)** "Backflipping With Miniature Quadcopters ..." IEEE TCST — Crazyflie 백플립. 참고만(GP-planning, 우리와 방식 다름).
- 서베이: 표준 복구 전부 최단경로 되돌림(Brescianini2020, arXiv 2002.09425, 2406.11723). momentum 플립완주 빈칸 확인.

## 역할분담 & AI (파킹)
- 3DOF=설계·해석 ground-truth·baseline. 6DOF=학습 데이터·평가(배포충실도). AI(모션프리미티브/펀넬 라이브러리: 90° crossing ω를 인덱스로 플립 꺼내기)는 **6DOF서 A+B 돌기 시작한 다음** 단계. 지금은 파킹.

## 관련
[[project-quadcopter-disturbance]] (별개 프로젝트, 오염 출처) · [[feedback-collaboration]] (단계별 안내, 사용자가 직접) · [[feedback-dark-plots]] · [[feedback-matlab-simulink]] (.slx는 MCP/batch로만, XML편집 금지·크래시)
