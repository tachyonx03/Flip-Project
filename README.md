# SO(3) Quadcopter Flip Simulation

MathWorks의 `asbQuadcopter`(Parrot Mambo) 예제를 SO(3) 자세 동역학 기반으로 개조한 Simulink 모델입니다. 회전행렬을 직접 적분하기 때문에 360° 플립 같은 대각도 기동을 안정적으로 시뮬레이션할 수 있습니다.

## 배경

기존 모델은 자세를 오일러 각(roll, pitch, yaw)으로 표현했습니다. 이 방식은 평상시 비행에는 문제가 없지만, 롤이나 피치가 90°에 가까워지면 짐벌락(gimbal lock)이 발생해 각도가 발산하고 플립 같은 기동을 구현할 수 없습니다.

이를 해결하기 위해 기체의 자세를 회전행렬(SO(3))로 정의했습니다. 회전행렬은 특이점이 없으므로 어떤 자세에서도 발산 없이 동작하며, 360° 회전을 포함한 임의의 자세 변화를 다룰 수 있습니다.

## 주요 특징

- SO(3) 회전 동역학: 오일러 각 대신 회전행렬(DCM)을 직접 적분하여 짐벌락 없이 대각도 기동을 지원합니다.
- 외란 토크·힘 주입: 동체에 임의의 외란 토크와 힘을 입력해 제어기 강건성을 시험할 수 있습니다.
- 스프레드시트 명령 입력: `cmdData.xlsx`로 위치와 자세 명령 시퀀스를 정의합니다.
- PID 자세 제어: 원본 Mambo의 roll/pitch/yaw PID 구조를 그대로 사용합니다.

## 요구 사항

- MATLAB / Simulink R2025b
- Aerospace Blockset
- Simulink 3D Animation (3D 시각화용)

## 시작하기

1. `Quadcopter.prj`를 열어 프로젝트를 로드합니다. 파라미터는 자동으로 초기화됩니다.
2. `mainModels/cmdData.xlsx`에서 명령 시퀀스를 편집합니다.
3. `asbQuadcopter` 모델을 실행합니다.

## 명령 입력 (`cmdData.xlsx`)

시트 `data`에 시간에 따른 명령을 입력합니다. 컬럼은 `time`, `X`, `Y`, `Z`, `Yaw`, `Pitch`, `Roll` 순서이며, 각각 시간과 위치(X, Y, Z), 자세(Yaw, Pitch, Roll) 레퍼런스를 의미합니다. 위치 명령 유무에 따라 위치제어와 자세제어 모드가 자동으로 전환됩니다.

## 명령 타입
VSS_COMMAND = 0;  % signal editor 
VSS_COMMAND = 1;  % Joystick + signal editor
VSS_COMMAND = 2;  % matfile 명령
VSS_COMMAND = 3;  % xlsx 스프레드시트 명령


## 모델 구성

- `asbQuadcopter` — 최상위 모델 (명령·제어·기체·시각화 통합)
- `nonlinearAirframe` — SO(3) 비선형 기체 동역학
- `flightController` — 위치·고도·자세(PID) 제어
- `flightControlSystem` — 센서·추정·제어 통합
- `stateEstimator` — 상태 추정

## 다음 단계

- 상태 추정기(SO(3) 기반): 현재 제어기는 시뮬레이션 물리 모델의 참(true) 자세를 직접 사용합니다. 실기체에 적용하려면 `stateEstimator`를 SO(3) 기반으로 재정의해 센서로부터 추정한 회전행렬을 피드백으로 사용해야 합니다.
- 플립 전후 안정화 제어: 플립 기동에 진입하기 전과 완료한 후에 자세를 안정화(호버 복귀)하는 제어 로직이 필요합니다.

## 참고

- `States.Euler`의 순서는 `[yaw, pitch, roll]`입니다.
- 제어기는 시뮬레이션 물리 모델의 참(true) 자세와 각속도를 직접 사용합니다.
