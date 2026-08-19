# tmux 사용 설명서

## 설치
`sudo apt install tmux -y`

## 개념
- Prefix 키(기본: `Ctrl+B`)를 먼저 누르고 손을 뗀 뒤, 다음 키를 입력하는 방식
- SSH 연결이 끊겨도 세션 내부 프로세스는 계속 실행됨

## 세션 생성 / 진입
- 새 세션 생성: `tmux new -s <세션이름>`
  - 예: `tmux new -s camera`
- 기존 세션 재접속: `tmux attach -t <세션이름>`
- 세션 목록 확인 (tmux 밖 일반 bash 명령): `tmux ls`

## 세션 나가기 (백그라운드 유지)
- `Ctrl+B` → `D` : Detach, 프로세스는 유지된 채 세션에서 빠져나옴

## 창(Window) 관련
- `Ctrl+B` → `C` : 새 창 생성
- `Ctrl+B` → `W` : 창 목록 보기 (인터랙티브 선택)
- `Ctrl+B` → `N` : 다음 창 이동
- `Ctrl+B` → `P` : 이전 창 이동

## 화면 분할 (Pane)
- `Ctrl+B` → `%` : 좌우 분할
- `Ctrl+B` → `"` : 상하 분할
- `Ctrl+B` → `방향키` : 분할된 pane 간 이동

## 스크롤
- `Ctrl+B` → `[` : 스크롤 모드 진입 (`Q`로 종료)

## 세션 종료 (삭제)
- 세션 안에서 프로세스 종료 후 나가기: `Ctrl+C`(프로세스 중지) → `exit`
- 밖에서 강제 종료 (attach 불필요): `tmux kill-session -t <세션이름>`
- tmux 서버 전체(모든 세션) 종료: `tmux kill-server`

## 실전 워크플로우 (예: run_camera.sh)
1. `tmux new -s camera`
2. `~/orin_ws/run_camera.sh` 실행
3. `Ctrl+B` → `D` 로 빠져나오기 (SSH 끊어도 무관)
4. 재확인 필요 시 `tmux attach -t camera`
5. 완전히 종료할 때 `tmux kill-session -t camera`