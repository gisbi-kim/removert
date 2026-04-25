# Removert v2 — 재설계 플랜

> 본 문서는 `gisbi-kim/removert`(v1)을 GPU 가속 + 순수 함수형 + ROS 분리 아키텍처로
> 재구현하기 위한 설계 플랜입니다. 알고리즘적 동작(Remove/Revert를 통한 동적
> 포인트 제거)은 **동일한 출력**을 보장하되, 구조·속도·유지보수성을 전면 개선합니다.

---

## 0. 설계 헌장 (Design Charter)

| # | 원칙 | 강제 수단 |
|---|------|-----------|
| C1 | ROS(app)와 알고리즘(lib)은 **물리적으로 분리**된 별도 CMake 타겟 | `libremovert_core`는 ROS 헤더를 단 1줄도 include 하지 않음. CI에서 `grep -r "ros/" core/` = 0건 강제 |
| C2 | 모든 알고리즘 함수는 **pure** (입력만으로 출력 결정, 부수효과 없음) | 멤버 상태 변경 금지, I/O 금지, 전역 변수 금지. 반환은 `[[nodiscard]]` |
| C3 | 함수 본문 ≤ 50 줄 | clang-tidy `readability-function-size` (line-threshold=50) |
| C4 | functional 우선: STL `<algorithm>`, `<numeric>`, range-based 변환 | 명시적 raw 루프는 PR 리뷰 사유 필요 |
| C5 | C++17 표준 (필요 시 C++14/11로 다운그레이드 허용, 11 미만 금지) | `-std=c++17`, `__cplusplus >= 201703L` 가드 |
| C6 | 1 클래스 = 1 책임 (SRP) | 클래스당 public 메서드 ≤ 7개, 멤버 ≤ 8개를 가이드라인으로 |
| C7 | 모든 하이퍼파라미터는 외부 주입 (`Config` 구조체 1개로 통일) | 코드 내 매직 넘버 = 0 (named constant도 `Config`로 흡수) |
| C8 | GPU 메모리 ≤ 8 GB, 단일 스캔 처리 속도 v1 대비 **≥ 30×** | 벤치마크 슈트가 회귀 시 CI 실패 |

---

## 1. 현재(v1) 핵심 문제 요약

| 영역 | 문제 | v2에서의 해결 |
|------|------|----------------|
| 결합 | `Removerter : public RosParamServer` — 알고리즘이 ROS에 직접 상속 의존 | `core/`에서 ROS 완전 분리 |
| 상태 | 단일 클래스 멤버 변수 48개, scan/static/dynamic 클라우드 모두 상주 | 함수형 파이프라인, 중간 결과는 호출자 소유 |
| 중복 | `map2RangeImg` ↔ `scan2RangeImg` 거의 동일 | 단일 `project_to_range_image()` 함수 |
| 동시성 | OpenMP race condition을 "실무상 OK"로 방치 (`Removerter.cpp:208-214`) | atomic min-reduction (CUDA `atomicMin`) 또는 sorted-write |
| 매직넘버 | `kFlagNoPOINT=10000`, `kBallSize=80`, `adaptive_coeff=0.05` 등 산재 | `Config` 일원화 |
| 미완성 | Revert / 실시간 콜백 / dynamic tracking TODO | 같은 파이프라인의 대칭 함수로 완성 |
| 성능 | 매 반복마다 range image 재생성, KdTree 재빌드 | GPU 상주, projection 1회 + 다중 해상도 mip-pyramid |

---

## 2. 타겟 아키텍처

```
removert-v2/
├── core/                 # 순수 C++17, ROS·CUDA 헤더 없음 (CPU 레퍼런스)
│   ├── include/removert/
│   │   ├── types.hpp           # Point, Pose, RangeImage, Config
│   │   ├── geometry.hpp        # cart2sph, transform, project
│   │   ├── range_image.hpp     # 범위 이미지 생성/비교
│   │   ├── remove.hpp          # Remove 단계 순수 함수
│   │   ├── revert.hpp          # Revert 단계 순수 함수
│   │   ├── knn.hpp             # KNN 추상 인터페이스 (전략 패턴)
│   │   └── pipeline.hpp        # 단계 합성 (compose)
│   └── src/                    # 위 헤더의 CPU 구현
│
├── gpu/                  # CUDA 구현, core 인터페이스를 만족
│   ├── include/removert/gpu/
│   │   ├── device_buffers.hpp  # RAII GPU 버퍼
│   │   ├── projection.cuh
│   │   ├── range_diff.cuh
│   │   └── knn_gpu.cuh
│   └── src/
│
├── io/                   # PCD/BIN/TXT 로더 (PCL 의존, ROS 무관)
│   └── ...
│
├── app/
│   ├── cli/              # ROS 없이 CLI 단독 실행 (배치 오프라인)
│   │   └── removert_cli.cpp
│   └── ros1/             # 얇은 ROS 래퍼 (subscribe → core 호출 → publish)
│       └── removert_node.cpp
│
├── tests/                # GoogleTest. core/는 100% 라인 커버리지 목표
├── bench/                # Google Benchmark, KITTI 미니셋으로 회귀
└── config/               # YAML, JSON 스키마
```

**의존 방향 (단방향)**:
```
app/ros1 ──┐
app/cli  ──┼──> io ──> core <── gpu (선택)
           │            ▲
           └────────────┘
```
`core`는 어디에도 의존하지 않음. `gpu`는 `core`의 인터페이스만 구현.

---

## 3. 모듈별 설계

### 3.1 `core/types.hpp` — 값 타입

```cpp
struct Point  { float x, y, z, intensity; };          // POD, trivially copyable
struct Pose   { Eigen::Matrix4f T; };                 // SE3 (float for GPU)
struct SphericalPoint { float az, el, r; };

struct RangeImage {
    int rows, cols;
    std::vector<float> range;     // rows*cols, kFlagNoPoint = +inf
    std::vector<int>   src_idx;   // 픽셀 → 원본 포인트 인덱스
};

struct ScanFrame {
    std::vector<Point> points_local;
    Pose               T_world_lidar;
    std::int64_t       stamp_ns;
};
```

- 모두 **값 타입**, 복사·이동 모두 noexcept 기본.
- `RangeImage`는 `cv::Mat` 의존 제거 (OpenCV는 시각화 전용 어댑터에만).

### 3.2 `core/types.hpp` — `Config` (단일 진실 소스)

```cpp
struct SensorConfig   { float vfov_deg, hfov_deg; Eigen::Matrix4f T_pose_lidar; };
struct RemoveConfig   { std::vector<float> resolutions_deg; float diff_upper, diff_lower; };
struct RevertConfig   { std::vector<float> resolutions_deg; /* ... */ };
struct KnnConfig      { int k; float radius_m; float scan_map_avg_diff_thr; };
struct DownsampleConfig { float voxel_size_m; };
struct PipelineConfig {
    SensorConfig     sensor;
    RemoveConfig     remove;
    RevertConfig     revert;
    KnnConfig        knn;
    DownsampleConfig downsample;
    int  start_idx, end_idx, keyframe_gap;
    bool use_gpu;
};
```

- `Config` 로딩은 `io/config_loader.{hpp,cpp}`에서 단 1곳.
- v1에서 흩어졌던 yaml 파라미터 + 하드코딩 상수 **전부 흡수**:
  `kFlagNoPOINT`, `kValidDiffUpperBound`, `kBallSize`, `adaptive_coeff`,
  `num_nn_points_within`, `dist_nn_points_within`, voxel size, FOV …
- JSON Schema (`config/schema.json`)로 검증, 누락 시 명시적 에러.

### 3.3 순수 함수 시그니처 (대표)

모든 함수는 **자유 함수**. 클래스는 자원(GPU 버퍼) 소유 시에만 사용.

```cpp
// geometry.hpp
[[nodiscard]] SphericalPoint cart_to_sph(Point p) noexcept;
[[nodiscard]] Point          transform(Point p, Eigen::Matrix4f const& T) noexcept;
[[nodiscard]] std::vector<Point>
               transform_all(std::vector<Point> const&, Eigen::Matrix4f const&);

// range_image.hpp
[[nodiscard]] RangeImage
project_to_range_image(std::vector<Point> const& pts,
                       SensorConfig const& sensor,
                       float resolution_deg);

[[nodiscard]] std::vector<int>
diff_dynamic_indices(RangeImage const& scan_rimg,
                     RangeImage const& map_rimg,
                     RemoveConfig const& cfg);

// remove.hpp
struct RemoveOutput { std::vector<int> dynamic_idx_in_map; };
[[nodiscard]] RemoveOutput
remove_once(std::vector<Point> const& map_global,
            ScanFrame const& scan,
            SensorConfig const&, RemoveConfig const&,
            float resolution_deg);

// pipeline.hpp — compose
[[nodiscard]] std::vector<Point>
remove_pipeline(std::vector<Point> map_global,           // 값으로 받아 이동
                std::vector<ScanFrame> const& scans,
                PipelineConfig const& cfg,
                IKnn const& knn_backend);
```

- 함수 본문 50줄 한계를 강제하기 위해 단계별 헬퍼로 분해 (`std::transform`,
  `std::accumulate`, `std::partition` 적극 활용).
- Range, view 라이브러리는 C++17이라 표준에 없음 → 자체 mini view (zip/enumerate)
  `core/include/removert/util/views.hpp`에 한정 제공.

### 3.4 클래스는 "자원 소유"에만

| 클래스 | 단일 책임 | 멤버 |
|--------|-----------|------|
| `DeviceContext` | CUDA stream/메모리 풀 RAII | `cudaStream_t`, allocator |
| `DeviceBuffer<T>` | GPU 메모리 1개 RAII | ptr, size |
| `KdTreeIndex` | KdTree 인덱스 1개 (PCL FLANN 래핑) | `pcl::KdTreeFLANN` |
| `RosBridge` (app/ros1) | ROS ↔ core 어댑터 | NodeHandle, pub/sub |

→ v1의 거대한 `Removerter` 클래스 **삭제**. 알고리즘은 자유 함수.

---

## 4. GPU 가속 전략 (≥ 30× 목표)

### 4.1 기술 스택 선택 (2026 기준)

**결정: 직접 CUDA 커널은 최소화하고, 검증된 상위 라이브러리를 조합한다.**

NVIDIA 생태계는 2024~25년 사이에 재편됐다. 과거의 `Thrust`, `CUB`, `libcudacxx`는
이제 **CCCL (CUDA Core Compute Libraries)** 라는 단일 저장소로 통합 배포되며
(구 `NVIDIA/thrust` repo는 archived), KNN 같은 도메인 알고리즘은 **cuVS / RAFT**
(RAPIDS)가 사실상 표준이 되었다 (FAISS GPU도 v1.10부터 cuVS 백엔드 사용).

| 라이브러리 | 역할 | v2에서의 사용 |
|-----------|------|---------------|
| **CCCL — Thrust** | STL-스타일 알고리즘 (transform/reduce/sort/scan) | element-wise diff, 좌표변환, sort_by_key 다운샘플 |
| **CCCL — CUB** | block/warp 단위 primitive | DeviceTransform/DeviceReduce 한 줄 호출 |
| **CCCL — libcudacxx** | device 코드용 std:: (atomic_ref, span, tuple) | projection 커널의 `cuda::atomic_ref<float>` min |
| **cuVS** (RAPIDS) | GPU 벡터 검색 — brute force / IVF / CAGRA | scan-self KNN, scan↔map KNN |
| **cuCollections** | GPU hash map/set | (대안) 해시 기반 voxel downsample |

**채택 안 한 것**:
- **stdpar / nvc++**: `std::execution::par`만으로 GPU 오프로드되는 건 매력적이나
  `nvc++` 컴파일러를 강제 → ROS/PCL 빌드 체인과 충돌 가능. 보류.
- **Kokkos / SYCL**: AMD까지 보면 매력적이나 본 프로젝트 범위 초과. P9+ 검토.
- **cuKDTree**: 유지보수 미약, cuVS brute-force가 더 빠르고 안정적.

### 4.2 핫스팟별 가속 계획 (직접 커널 = 1개)

| v1 핫스팟 | 위치 | 구현 방식 | 기대 배속 |
|-----------|------|-----------|-----------|
| Range image projection (scatter + min) | `Removerter.cpp:175-271` | **얇은 직접 CUDA 커널** + `cuda::atomic_ref<float>` min — 융합 1패스가 핵심이므로 raw kernel | 50× |
| Diff & 임계값 (`cv::absdiff` + threshold) | `Removerter.cpp:559-` | **Thrust** `transform` + `copy_if`, 또는 위 projection 커널과 융합 | 100× |
| `transformPointCloud` (4×4 × points) | 다수 | **Thrust** `transform` + Eigen functor (cuBLAS GEMM은 over-engineering) | 30× |
| KNN (scan-self, scan↔map) | `Removerter.cpp:725,731` | **cuVS** `cuvs::neighbors::brute_force::build` + `search` | 20~50× |
| Voxel downsample (global map) | 맵 빌드 | **Thrust** `sort_by_key` + `reduce_by_key` (또는 cuCollections hash) | 40× |

**핵심 원칙**:
1. **상주화**: map은 한 번 GPU 업로드 후 모든 해상도/스캔에서 재사용.
2. **융합**: projection + diff + dynamic mask를 **단일 커널 1패스**로 묶음
   (이 부분만 raw CUDA. Thrust로는 fusion이 어려움).
3. **나머지는 라이브러리**: 직접 짜면 손해. cuVS의 brute-force KNN은 H100에서
   30 GB 데이터셋 기준 300× 보고된 사례 있음.

### 4.3 8 GB 메모리 예산 산정

KITTI 00 시퀀스 기준 (약 4500 스캔, 평균 12만 포인트, global map 1500만 포인트):
- Map points (Float4 = 16 B) : 15M × 16 = **240 MB**
- cuVS brute-force index (precomputed norms 등) : ~ **300 MB**
- Range image pyramid (5 해상도, 2048×128 float) : 5 × 1 MB ≈ **5 MB**
- Per-batch scan buffer (32 스캔 × 128k × 16 B) : **64 MB**
- Thrust temp storage / CUB workspace : ~ **500 MB**
- 작업 메모리 / 오버헤드 여유 : ~ **7 GB**

→ Global map 전체 + 다중 해상도 range image GPU 상주 가능. **8 GB 이내**.

### 4.4 인터페이스 (백엔드 추상화)

`core/`는 GPU 헤더 무의존. 백엔드는 인터페이스로만 의존:

```cpp
// core/include/removert/backends.hpp
class IRangeImageBackend {
public:
    virtual RangeImage project(std::vector<Point> const&,
                               SensorConfig const&, float res_deg) const = 0;
    virtual ~IRangeImageBackend() = default;
};

class IKnnBackend {
public:
    struct Result { std::vector<int> idx; std::vector<float> dist; };
    virtual Result knn(std::vector<Point> const& dataset,
                       std::vector<Point> const& queries, int k) const = 0;
    virtual ~IKnnBackend() = default;
};
```

`gpu/`에서 구현:
- `CudaRangeImageBackend` — raw CUDA 커널 + libcudacxx atomic
- `CuvsKnnBackend`        — cuVS brute_force 래핑
- `ThrustDownsampleBackend` — Thrust sort+reduce

런타임에 `cfg.use_gpu`로 CPU/GPU 백엔드 스위칭 → CPU만으로도 회귀 검증 가능.

### 4.5 빌드 통합 (CMake)

```cmake
# CCCL: CUDA Toolkit 12.6+에 번들로 포함됨, 별도 설치 불필요
find_package(CUDAToolkit 12.6 REQUIRED)

# cuVS: RAPIDS 24.10+ (conda 또는 system install)
find_package(cuvs 24.10 REQUIRED)

target_link_libraries(removert_gpu PRIVATE
    CUDA::cudart
    cuvs::cuvs       # KNN
    # Thrust/CUB/libcudacxx은 CUDAToolkit에 포함됨
)
```

### 4.6 정확성 보장

- CPU 레퍼런스와 GPU 결과의 픽셀 단위 비교 테스트 (`tests/parity_test.cpp`).
- 허용 오차: range diff `|Δ| < 1e-3 m`, dynamic 인덱스 집합 일치율 ≥ 99.9 %.
- KNN: cuVS brute-force는 정확 검색 → PCL FLANN과 거리값 `|Δ| < 1e-5` 기대.

### 4.7 단계별 예상 성능 이득 분석

**기준 워크로드**: KITTI 00 시퀀스 (스캔 ~4,500장, 다운샘플 후 ~12k pts/scan,
global map ~15M pts).
**v1 환경**: CPU 8코어 OpenMP / **v2 환경**: 동일 CPU + RTX 4070 (8 GB).

| # | 단계 (입출력) | 기존 알고리즘 요약 | 기존 예상 시간 | 변경 구현 요약 | 변경 예상 시간 | 이득 |
|---|---|---|---|---|---|---|
| 1 | **Scan 로드 + 다운샘플**<br>`.bin/.pcd → vector<Point>` | 파일별 fread + PCL `VoxelGrid` (0.05 m), 직렬 | ~30 s | 비동기 I/O (`std::async`) + Thrust `sort_by_key/reduce_by_key` 다운샘플 (배치 GPU 업로드) | ~10 s | **3×** |
| 2 | **Pose 적용 + Global map 빌드**<br>local scans → world map | scan별 `transformPointCloud` 후 누적, octree 다운샘플 | ~60 s | Thrust `transform`(Eigen functor) + voxel hash 1패스 (cuCollections) | ~5 s | **12×** |
| 3 | **Map → Range image (멀티해상도)**<br>per scan × N res | OpenMP scatter, race를 "실무상 OK"로 방치 | 3 res × 4,500 × 50 ms<br>≈ **675 s** | **융합 CUDA 커널** (project + diff + dynamic mask 1패스) + `cuda::atomic_ref<float>` min, map은 GPU 상주 | 3 res × 4,500 × 0.7 ms<br>≈ **9 s** | **75×** |
| 4 | **Scan → Range image**<br>per scan × N res | `scan2RangeImg`, OpenMP | 3 × 4,500 × 5 ms<br>≈ 67 s | 위 융합 커널에 흡수 (별도 단계 X) | (위에 포함) | — |
| 5 | **Diff & dynamic 인덱스 추출** | `cv::absdiff` + 임계값, per-pixel CPU loop | 3 × 4,500 × 2 ms ≈ 27 s | 위 융합 커널에 흡수 | (위에 포함) | — |
| 6 | **Map 갱신 (dynamic 제거)** | `pcl::ExtractIndices` per scan | ~20 s | Thrust `partition` (in-place, GPU) | ~1 s | **20×** |
| 7 | **Revert 단계** *(v1 미구현)* | — (TODO) | n/a | 동일 융합 커널의 대칭 버전 (혹은 같은 커널 재사용) | ~10 s | (신규) |
| 8 | **Scan-side KNN 제거**<br>scan-self KNN + scan↔map KNN | per scan PCL FLANN 빌드 ×2 + per-point search | 4,500 × 200 ms<br>≈ **900 s** | **cuVS** `brute_force::build` 1회 (map) + 배치 `search` (모든 scan 병합 query) | ~15 s | **60×** |
| 9 | **출력 저장**<br>static/dynamic PCD, clean scans | PCL `savePCDFileBinary`, 직렬 | ~30 s | 비동기 I/O + GPU→host 멤캐시 압축 | ~15 s | **2×** |
| | **합계** | | **~30 분** (≈1,810 s) | | **~65 s** | **~28×** |

#### 주요 관찰

- **병목은 단계 3·8에 집중** (전체의 ~87 %) → 두 곳에 집중 투자하면 30× 목표 도달.
- **단계 1·9는 I/O 바운드** → GPU로 큰 이득 없음, 비동기화로만 부분 개선.
- **단계 4·5는 단계 3에 융합** → 커널 1개로 묶어 메모리 트래픽 절감.
- **30× 마진 확보**: 합산 28×는 보수 추정. cuVS brute-force가 H100에서 300×
  보고 사례 있음 → RTX 4070에서도 50~80× 가능, 실측 시 30~40× 안정 예상.
- **Amdahl 한계**: I/O 단계(1, 9)가 v2 합계의 약 38 %를 차지 → 50× 이상을 노리려면
  PCD 포맷 자체를 재설계해야 함 (예: zstd-PCD, mmap binary). 본 v2 범위 외.

#### 가정·면책

- 시간 추정은 KITTI 00 풀 시퀀스 기준 **수치 모델 + 유사 구현 벤치 참고치**, 실측 아님.
- 단계 3의 v1 50 ms/scan은 OpenMP 8코어에서 map 부분집합 추출 + 투영 비용 포함 추정.
- cuVS KNN의 15 s는 4,500 query 배치 + 12k pts/query 가정. 배치가 작으면 더 길어질 수 있음.
- **실측은 P0에서 v1 베이스라인 측정, P7 완료 시점에 본 표 갱신**.

---

## 5. 마이그레이션 단계 (Phase Plan)

각 단계는 **독립적으로 머지 가능**하며, 단계 끝마다 KITTI 00 미니셋으로 회귀.

| Phase | 내용 | 산출물 | 완료 기준 |
|-------|------|--------|-----------|
| **P0** | 베이스라인 캡처 | v1 출력 PCD를 골든 픽스처로 저장 | 회귀 비교 스크립트 작동 |
| **P1** | `core/` 스켈레톤 + `Config` + `types.hpp` | 빌드만 통과 | `libremovert_core.a` 생성 |
| **P2** | `io/` 모듈 (BIN/PCD/포즈 로더) 분리 | KITTI 로더가 ROS 없이 동작 | 단위 테스트 통과 |
| **P3** | `core/` CPU 레퍼런스 구현 (Remove 경로) | CLI에서 v1과 동일 출력 | 픽셀 단위 일치 |
| **P4** | `app/ros1/` 어댑터 (얇게) | ROS launch가 v1처럼 작동 | 사용자 호환성 유지 |
| **P5** | Revert 단계 완성 (v1의 TODO 해소) | Revert 함수 + 테스트 | 정성 평가 |
| **P6** | `gpu/` projection+diff 융합 커널 (raw CUDA + libcudacxx) | 단일 핫스팟 30× 달성 | 벤치 통과 |
| **P7** | cuVS KNN + Thrust downsample/transform 통합 | 엔드투엔드 30× 달성 | 8GB 이내, 30× 이상 |
| **P8** | 실시간 콜백 (스트리밍 모드) | `cloudHandler` 완성 | 30 Hz 라이브 |

---

## 6. 빌드/의존성

- **CMake 3.18+**, `target_compile_features(... cxx_std_17)`.
- 옵션: `REMOVERT_BUILD_GPU=ON/OFF`, `REMOVERT_BUILD_ROS1=ON/OFF`,
  `REMOVERT_BUILD_TESTS=ON/OFF`.
- 의존성:
  - 필수: PCL (IO + CPU KdTree 폴백), Eigen
  - 선택(GPU): CUDA Toolkit 12.6+ (CCCL = Thrust+CUB+libcudacxx 번들), cuVS 24.10+
  - 선택(ROS): ROS1 Noetic 또는 ROS2 Humble (어댑터 디렉토리 분리)
- 죽은 의존성 제거 (v1 `CMakeLists.txt:27,46`의 GTSAM 주석 등).
- `clang-format`, `clang-tidy`, `cppcheck` 프리커밋.

---

## 7. 테스트 & 벤치

- **단위**: 각 순수 함수에 대해 GoogleTest. `core/`는 라인 커버리지 95% 목표.
- **속성 기반**: range image projection은 round-trip 테스트
  (project → unproject → 동일 픽셀 복원).
- **회귀**: KITTI 00의 처음 200 스캔으로 골든 PCD 비교 (chamfer < ε).
- **벤치**: Google Benchmark.
  - `BM_project_cpu`, `BM_project_gpu`, `BM_pipeline_end_to_end`
  - CI에서 v1 대비 30× 미만 시 실패.
- **GPU/CPU 패리티**: P6 이후 매 PR.

---

## 8. 외부 인터페이스 호환성

- v1의 yaml 키는 `io/config_loader`에서 신규 `Config`로 매핑하는 어댑터 제공
  → 사용자는 동일 launch/yaml로 v2 실행 가능 (deprecation 경고 출력).
- ROS 토픽명은 그대로 유지 (`/removert/*`).
- 출력 PCD 파일명·구조 유지.

---

## 9. 위험 & 대응

| 위험 | 대응 |
|------|------|
| GPU KNN의 정확도가 PCL FLANN과 미세 차이 | 허용 오차 정의 + CPU fallback 옵션 |
| 8 GB 초과 (대형 시퀀스) | 타일링 (map을 공간 분할) 옵션 추가 |
| ROS1 EOL | `app/ros2/` 추가가 쉬운 구조로 설계 (어댑터 1파일) |
| 함수 50줄 제약이 가독성 해침 | 헬퍼 분해 + `inline` 권장, 강제는 lint 경고 수준에서 시작 |

---

## 10. 첫 PR 체크리스트 (P1 시작용)

- [ ] `core/` 디렉토리 + `CMakeLists.txt` 추가, ROS include 0건 검증 스크립트
- [ ] `types.hpp` (Point, Pose, RangeImage, ScanFrame, Config 전체)
- [ ] `io/config_loader` + JSON Schema, v1 yaml 호환 어댑터
- [ ] clang-tidy/clang-format 설정, CI green
- [ ] `tests/types_test.cpp` 컴파일·실행

---

*문서 버전*: 2026-04-25 초안.
