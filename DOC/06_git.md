# 06 — 브랜치 운용 · 커밋 규칙 · 훅

> 이 레포는 브랜치 둘이 **파일로 나뉜다.** 무엇을 어디에 커밋하는지만 알면 된다.

## 1. 브랜치 배치

```
main            코드 + 문서
                  orin_ws/ · stm_ws/ · DOC/ · redesign/ · 루트 *.md
                  ← 동료(DPC_B / hand_ctrl 펌웨어)도 여기에 올린다

feature/ecu_ctr  main 전부  +  analysis/  +  test_plan_*.md
```

`feature/ecu_ctr` 은 main 이 **절대 건드리지 않는 파일만** 더 갖는다. 그래서 두 브랜치가
같은 파일을 두고 충돌할 일이 구조적으로 없다.

| 무엇을 고치나 | 어디서 커밋 |
|---|---|
| `orin_ws/` · `stm_ws/` · `DOC/` · `redesign/` · 루트 `*.md` | **main** |
| `analysis/` · `test_plan_*.md` | **feature/ecu_ctr** |

> **왜 나눴나** (2026-07-30): 로봇을 돌리려는 사람이 견인력 분석 파이프라인까지 읽어야 하는
> 상태였다. main 에는 **로봇을 돌리는 데 필요한 것만** 둔다.

## 2. 평소 작업 — main

```bash
git checkout main
# ... orin_ws / stm_ws / DOC 작업 ...
git add -A && git commit -m "FIX: ..."
git push origin main
```

동료가 먼저 올렸으면 push 가 거부된다. **그때 절대 `--force` 를 쓰지 않는다:**

```bash
git pull --rebase origin main      # 내 커밋을 상대 것 위로 올린다
git push origin main
```

## 3. 분석을 고칠 때 — feature/ecu_ctr

```bash
git checkout feature/ecu_ctr
git merge main                     # main 최신을 먼저 끌어온다 (충돌 없음)
# ... analysis/ 작업 ...
git add -A && git commit -m "ANALYSIS: ..."
git push origin feature/ecu_ctr
```

### ⚠ `merge` 를 쓴다 — `rebase` 가 아니라

`rebase` 는 커밋 해시를 바꿔서 매번 `--force-with-lease` 가 필요하다. 협업자가 있는 지금
force-push 는 **남의 작업을 지울 수 있는 유일한 명령**이다. `merge` 는 덧붙이기만 하므로
force 가 영영 필요 없다. 히스토리에 merge 커밋이 남지만 그건 문제가 아니다.

### 브랜치 전환은 싸다

`analysis/` 와 `test_plan_*.md` 만 나타났다 사라진다. `orin_ws/` 는 양쪽이 같아서
**colcon 재빌드가 필요 없고**, `build/`·`install/` 은 gitignore 라 전환해도 남는다.

### ⚠ 단, 추적 상태가 다른 파일은 전환 때 지워진다

git 은 "지금 커밋에는 추적되는데 옮겨갈 커밋에는 없는" 파일을 **삭제한다.** 방금 `git rm
--cached` 한 파일이 다른 브랜치에서는 아직 추적 중이면, 그 브랜치로 갔다 오는 사이에
로컬 파일이 사라진다. **2026-07-30 에 실제로 그렇게 `.obsidian/`·`Debug/` 가 지워졌다**
(`7332be9^` 에서 복구했다).

두 브랜치 모두 그 커밋을 포함한 뒤에는 생기지 않는 문제다. 그래도 원본이 필요하면:

```bash
git checkout <그 파일이 살아 있던 커밋>^ -- <경로>
git rm -r --cached <경로>          # 다시 추적 상태로 만들지 않기
```

## 4. 커밋 훅

`tools/hooks/pre-commit` 이 **커밋 직전에 자동으로** 세 가지를 막는다.

| | 막는 것 |
|---|---|
| ① | main 에 `analysis/` · `test_plan_*.md` 가 섞이는 것 |
| ② | 빌드 산출물(`Debug/` `*.cyclo` `build/`) · 개인 설정(`.obsidian/` `.claude/settings.local.json`) |
| ③ | 충돌 마커(`<<<<<<<`)가 그대로 커밋되는 것 |

### 설치 (사람마다 한 번)

```bash
git config core.hooksPath tools/hooks
```

훅 파일은 레포에 있지만 **git 이 자동으로 켜 주지 않는다.** 이 한 줄이 있어야 동작한다.
`.git/hooks/` 에 복사하는 방식도 되지만, 그러면 훅을 고칠 때마다 각자 다시 복사해야 한다.
`core.hooksPath` 는 레포 안의 파일을 그대로 보므로 고치면 바로 반영된다.

### 확인

```bash
git config core.hooksPath        # tools/hooks 가 나와야 한다
```

### 걸렸을 때

메시지가 무엇을 어떻게 빼는지 알려 준다. 정말 그대로 가야 하면:

```bash
git commit --no-verify
```

### 해제

```bash
git config --unset core.hooksPath
```

### 왜 훅까지 두는가

이 레포에서 이미 겪었다 — `.gitignore` 에 `stm_ws/ECU_V3/Debug/` **하나만** 적혀 있어서
다른 보드의 CubeIDE 산출물 **121개**가 몇 달 동안 조용히 커밋에 딸려 들어갔고,
2026-07-30 정리 때야 드러났다. **사람이 매번 확인하는 규칙은 잊히고, 자동으로 막는 것은
잊히지 않는다.**

## 5. 되돌리기

### 정리 전 상태 — `backup/pre-reorg-260730` 태그

파일이 아니라 **커밋을 가리키는 이름표**다. 2026-07-30 정리 직전(로컬 49커밋 + 그때
삭제한 문서 7개)을 붙잡아 둔 것이고 **로컬에만** 있다 (GitHub 엔 없다).

```bash
git show backup/pre-reorg-260730:HANDOFF_260721.md          # 내용 보기
git show backup/pre-reorg-260730:TASKS.md > TASKS.md        # 되살리기
git ls-tree -r --name-only backup/pre-reorg-260730          # 그때 있던 파일 전부
git log backup/pre-reorg-260730 --oneline                   # 접기 전 49커밋
```

필요 없어지면 `git tag -d backup/pre-reorg-260730`. **지워도 GitHub 엔 영향 없다.**

### 그때 지운 문서 7개

`HANDOFF_260721.md` · `HANDOFF_260724.md` · `prompt_260720.md` · `_index.md` ·
`TASKS.md` · `ORIN_USER_GUIDE.md` · `redesign/_HANDOFF.md`

지우기 전에 코드 인용을 전수 확인했고, **지우면 안 되는 것 3건**이 나와 남겼다:
`redesign/`(68개 파일이 인용) · `testbed_spec.md`(24개) ·
`failsafe_analysis_260717.md`(ECU_V3 펌웨어 5개).

## 6. 커밋 메시지

머리말은 `FEAT` / `FIX` / `REFACTOR` / `DOC` / `CHORE` / `BUILD` / `ANALYSIS`.

본문에는 **무엇을 했는지보다 왜 했는지**를 적는다 — 무엇은 diff 가 말해 준다.
고친 결함이면 **그게 어떻게 안 보이고 있었는지**를 같이 적는다. 이 레포에서 반복해서
나온 사고가 "배관은 있는데 생산자가 없다"·"두 곳에 적어서 갈라졌다" 부류라,
그 서술이 다음 사람에게 가장 쓸모 있다.
