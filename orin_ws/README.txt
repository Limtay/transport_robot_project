orin_ws — 커맨드 치트시트
=========================

상세 운용 문서는 ../DOC/ 에 있다:
  DOC/01_setup.md          빌드·RT 세팅·배포
  DOC/02_web.md            웹 UI
  DOC/03_cli.md            CLI
  DOC/04_experiment.md     실험 기록 규격
  DOC/05_troubleshooting.md 안 될 때

여기는 자주 쓰는 것만 모은다.


## 기본 흐름

  ./build.sh                    # 빌드 (--test 로 단위 테스트)
  ./setup_rt.sh                 # latency_timer=1 + 포트 권한 (재부팅마다)
  ./web.sh                      # 웹 → http://localhost:8080  (웹이 브리지를 띄운다)

CLI 로만 쓸 때 (웹과 동시에 쓰지 않는다):

  ./run.sh control current &    # 브리지 직접 기동
  ./cli.sh status
  ./cli.sh arm on
  ./cli.sh stream current 2.0


## 브리지 기동 모드

  ./run.sh                      # project (기본 주행)
  ./run.sh control current      # 제어 실험 (auto_mode: none|current|direct|velocity|position)
  ./run.sh traction             # 견인 = control + auto_mode:none + control_test 프리셋
  ./run.sh manual               # 자동 설정 전무 (raw_read/raw_write 전용)

  ⚠ traction_test_mode 는 폐지됐다 — 주면 기동 게이트가 거부한다 (exit≠0).
  ⚠ enable_infra1/2 라는 파라미터는 존재하지 않는다 (구 메모 잔재).


## 실험 기록

  ./cli.sh run data/profiles/x.yaml --record --name s1_w20
      → data/rosbags/s1_w20_<MM-DD_HH-MM>/{bag,profile.yaml,result.json}

  ⚠ cli.sh 는 CWD 를 레포 루트로 옮긴다. 파일 경로는 레포 루트 기준 또는 절대경로.
    (orin_ws 에서 `../data/...` 로 주면 한 단계 더 위를 본다)

  ./rosbag_test.sh s1_w20       # 수동 조작용 임시 bag (실험 폴더 규격 아님)

  ⚠ 기록 토픽은 /carrier/control/feedback 이다.
    구 /carrier/testbed/feedback 로 record 하면 빈 bag 이 남는다.


## Orin 배포

  ./deploy.sh                   # src/ 전체 + 스크립트
  ./deploy.sh --host 192.168.55.1   # USB 이더넷
  ./deploy.sh --dry             # 무엇이 갈지만
  ./deploy.sh --build           # 전송 후 원격 빌드

  ping 192.168.55.1             # USB 연결 확인
  ssh swarm@192.168.55.1        # USB 이더넷
  ssh swarm@10.251.24.214       # 유선/무선
  (10.108.169.214 는 구 주소)


## 확인·디버깅

  ros2 topic hz /carrier/control/feedback     # control 모드에서 200Hz
  ros2 topic echo /carrier/ecu/status --once  # lc/hs/degraded_cnt/hw_error
  ros2 topic echo /carrier/ecu/motor --once   # error_code / temp
  ./cli.sh status --json | python3 -m json.tool

  cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer   # 1 이어야 한다
  ulimit -r                                              # 80 이상 (SCHED_FIFO)

  ros2 run plotjuggler plotjuggler
  ros2 bag play <폴더>


## 함정

  · colcon 은 반드시 orin_ws 루트에서. src/ 안에서 돌리면 유령 install 이 생기고
    나중에 그것이 import 돼서 "고친 코드가 아닌 옛 코드" 가 돈다.
      rm -rf src/*/build src/*/install src/*/log
  · pkill 로 브리지를 죽이면 웹 supervisor 가 자식을 잃고 상태가 어긋난다.
    웹에서 정지시킨다.
  · 포트 권한은 USB 재열거 때 조용히 바뀐다. 증상은
    "Open Failed: Bad file descriptor" 무한 재시도.
