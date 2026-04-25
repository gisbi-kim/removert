# GPU Box Handoff Prompt

이 문서는 NVIDIA GPU 머신에서 새 Claude Code 세션을 열고 P5/P6
구현을 검증·완성할 때 그대로 복사해 붙여 넣을 self-contained 프롬프트입니다.
이전 대화 컨텍스트가 없는 가정 하에 작성됐습니다.

---

```
저장소: https://github.com/gisbi-kim/removert (master + PR #34 = 브랜치 v2-impl)

## 컨텍스트 (먼저 읽기)
- 사전 합의된 설계 플랜: docs/v2-plan.md (반드시 정독)
- v2 구현 PR: gh pr view 34 --comments
- 핵심: 강화 visibility (multi-hit + free-segment carving + log-odds)
  로 동적 포인트 제거. ROS 없는 순수 C++17 오프라인 도구.
- CPU 경로(REMOVERT_BUILD_GPU=OFF)는 23/23 유닛 테스트 통과 검증됨.
- GPU 경로(REMOVERT_BUILD_GPU=ON)는 P5/P6 본문 작성됐지만
  하드웨어에서 한 번도 빌드/실행 안 됨. TODO(local-test) 표시 있음.

## 환경 요구
- Linux + NVIDIA GPU (compute capability ≥ 7.0 권장)
- CUDA Toolkit 12.6+ (CCCL = Thrust + CUB + libcudacxx 번들)
- cuVS 24.10+ (RAPIDS, conda install 권장:
  `conda install -c rapidsai cuvs`)
- Eigen3, HDF5 + HighFive, sqlite3, GoogleTest

## 작업 (단계별)

1. **체크아웃 + 빌드 시도**
   ```
   git clone https://github.com/gisbi-kim/removert
   cd removert
   git checkout v2-impl
   cmake -B build -DREMOVERT_BUILD_GPU=ON \
                   -DREMOVERT_BUILD_TESTS=ON \
                   -DREMOVERT_WITH_HDF5=ON \
                   -DREMOVERT_WITH_SQLITE=ON
   cmake --build build -j
   ```

2. **컴파일 에러 디버깅** (예상: atomic_ref ordering, cuVS namespace drift)
   - gpu/src/projection.cu — libcudacxx 버전별 cuda::atomic_ref API 차이
   - gpu/src/knn_gpu.cpp — cuvs::neighbors::brute_force 시그니처
     (24.04 ↔ 24.10 사이 인자 순서/네임스페이스 변경)
   - 필요하면 https://docs.rapids.ai/api/cuvs/stable/cpp_api/neighbors_bruteforce/
     로 확인

3. **pipeline 와이어링** (현재 cfg.use_gpu=true여도 CPU 경로 강제)
   - core/src/pipeline.cpp 에서 use_gpu 분기 추가
   - host→device 업로드: map points + scan points + T_lidar_world
   - state_log_odds 디바이스 버퍼 할당 + log_odds_prior 초기화
   - 매 scan마다 run_fused_projection 호출
   - 끝나고 device→host로 log_odds 회수 → 임계값 비교 → dynamic 인덱스
   - cuVS KNN backend도 IKnnBackend 인터페이스 정의해서 swap 가능하게

4. **GPU/CPU 패리티 테스트** (tests/test_parity.cpp 확장)
   - 합성 시 (정적 벽 + 동적 점 1개) CPU와 GPU 결과 비교
   - 허용 오차: range diff |Δ| < 1e-3, dynamic 인덱스 집합 일치율 ≥ 99.9%

5. **벤치마크** (bench/bench_projection.cpp 확장)
   - CPU vs GPU 단일 핫스팟 (range image projection)
   - 엔드투엔드 (KITTI seq 01 100 프레임)
   - 목표: 단일 핫스팟 ≥ 50×, 엔드투엔드 ≥ 30× (플랜 §4.7)

6. **실데이터 검증**
   ```
   bash tools/fetch_kitti_seq01.sh        # ~5GB 다운로드
   python3 tools/build_seq01_sqlite.py    # 9초, data/seq01.sqlite 생성
   build/app/cli/removert_cli \
       --dataset sqlite \
       --input  data/seq01.sqlite \
       --config config/v2_default.json \
       --output-dir /tmp/seq01_out
   ```
   - SemanticKITTI 라벨로 TP/FP/FN 평가 (옵션)
   - 출력 PCD를 CloudCompare/Open3D로 시각 확인

7. **PR #34 업데이트**
   - 위 작업을 v2-impl 브랜치에 추가 커밋 (phase 단위로 의미있게 분할)
   - PR body의 "What's not done" 섹션 업데이트
   - 패리티 + 벤치 결과 표로 첨부
   - 모든 게 OK면 Ready-for-review 전환 → master 머지

## 원칙 (재확인)
- C1~C8 헌장 (docs/v2-plan.md §0) 엄수
- master 직접 push 금지, 모든 변경 v2-impl → PR
- 각 커밋 끝 `Co-Authored-By: Claude ...` 추가
- 빌드 무한 디버깅 X — 명확히 보고하고 다음 단계로

시작하세요.
```
