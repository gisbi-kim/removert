# Removert v2 — 재설계 플랜

> 본 문서는 `gisbi-kim/removert`(v1)을 GPU 가속 + 순수 함수형 + ROS 분리 아키텍처로
> 재구현하기 위한 설계 플랜입니다. 알고리즘적 동작(Remove/Revert를 통한 동적
> 포인트 제거)은 **동일한 출력**을 보장하되, 구조·속도·유지보수성을 전면 개선합니다.

---

## 0. 설계 헌장 (Design Charter)

| # | 원칙 | 강제 수단 |
|---|------|-----------|
| C1 | **순수 오프라인 배치 도구**. ROS·미들웨어 의존 0. 입력은 디스크상의 데이터셋 (HDF5/SQLite/디렉토리) | `libremovert_core`/`io`/`app`은 ROS 헤더를 단 1줄도 include 하지 않음. CI에서 `grep -r "ros/" .` = 0건 강제 |
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
| 결합 | `Removerter : public RosParamServer` — 알고리즘이 ROS에 직접 상속 의존 | **ROS 완전 제거**. 오프라인 배치만 지원, 입력은 HDF5/SQLite/디렉토리 |
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
├── core/                 # 순수 C++17, 외부 의존 0 (CPU 레퍼런스)
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
├── gpu/                  # CUDA 구현, core 인터페이스 만족
│   ├── include/removert/gpu/
│   │   ├── device_buffers.hpp  # RAII GPU 버퍼
│   │   ├── projection.cuh
│   │   ├── range_diff.cuh
│   │   └── knn_gpu.cuh
│   └── src/
│
├── io/                   # 데이터셋 로더 (HDF5/SQLite/KITTI 디렉토리)
│   ├── include/removert/io/
│   │   ├── dataset_reader.hpp  # IDatasetReader 추상
│   │   ├── hdf5_reader.hpp
│   │   ├── sqlite_reader.hpp
│   │   ├── kitti_dir_reader.hpp
│   │   ├── pcd_writer.hpp
│   │   └── config_loader.hpp   # YAML/JSON → Config
│   └── src/
│
├── app/
│   └── cli/              # 단독 실행 바이너리 (유일한 frontend)
│       └── removert_cli.cpp
│
├── tests/                # GoogleTest. core/는 100% 라인 커버리지 목표
├── bench/                # Google Benchmark, KITTI 미니셋으로 회귀
└── config/               # YAML, JSON 스키마
```

**의존 방향 (단방향, 단순)**:
```
app/cli ──> io ──> core <── gpu (선택)
                    ▲
                    └── (직접 의존 가능)
```
`core`는 어디에도 의존하지 않음. `gpu`는 `core` 인터페이스만 구현.
**ROS·미들웨어·메시지 라이브러리는 어디에도 없음**.

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
| `KdTreeIndex` | KdTree 인덱스 1개 (PCL FLANN 래핑, CPU fallback) | `pcl::KdTreeFLANN` |
| `Hdf5DatasetReader` | HDF5 파일 1개 read-only 핸들 | `HighFive::File` |
| `SqliteDatasetReader` | SQLite DB 1개 read-only 핸들 | `sqlite3*` |

→ v1의 거대한 `Removerter` 클래스 **삭제**. `RosParamServer`/`RosNodeHandle`도 삭제.
알고리즘은 자유 함수.

### 3.5 데이터셋 로더 인터페이스

전제: **순수 오프라인 배치**. 입력은 디스크 상의 데이터셋 파일/디렉토리.
ROS bag, 토픽 구독, 시간 동기화 등은 v2 범위 외 (필요하면 외부 도구로 사전 변환).

`io/`에 단일 추상:

```cpp
class IDatasetReader {
public:
    [[nodiscard]] virtual std::size_t size() const = 0;        // 스캔 개수
    [[nodiscard]] virtual SensorConfig sensor() const = 0;     // 1회 호출
    [[nodiscard]] virtual ScanFrame    read(std::size_t idx) const = 0;
    // 배치 read는 기본 구현이 read()를 N번 호출, 백엔드가 오버라이드 가능
    [[nodiscard]] virtual std::vector<ScanFrame>
                  read_batch(std::size_t begin, std::size_t end) const;
    virtual ~IDatasetReader() = default;
};
```

구현체:

| 구현 | 입력 포맷 | 사용처 |
|------|-----------|--------|
| `Hdf5DatasetReader` | 단일 `.h5` (권장) | 표준, 압축 + chunk read 빠름 |
| `SqliteDatasetReader` | 단일 `.db` | 메타데이터 위주, BLOB로 포인트 저장 |
| `KittiDirReader` | KITTI 디렉토리 (`.bin` + `poses.txt`) | v1 호환, 마이그레이션 보조 |

**권장: HDF5**. 포인트 클라우드는 `(N, 4) float32` 배열의 자연스러운 매체이고,
chunk + 압축(zstd/lz4) + 부분 읽기 + 인덱스 기반 random access가 모두 무료다.
SQLite는 메타데이터 + 작은 BLOB에 강하고 inspectability가 좋지만 큰 배열은 비효율.

**HDF5 권장 스키마**:
```
/sensor                       group
  @vfov_deg, @hfov_deg        attrs (float)
  @T_pose_lidar               attr (4×4 float, row-major)

/poses                        dataset (M, 16) float32   # 모든 스캔의 SE3 포즈
                              chunked (1, 16), zstd-3
/timestamps_ns                dataset (M,)  int64

/scans/{i:06d}/points         dataset (Ni, 4) float32   # x, y, z, intensity
                              chunked (Ni, 4), zstd-3
                              @stamp_ns (int64)
```

- 스캔별 `Ni`가 다르므로 그룹/데이터셋 1개씩 (HDF5의 hierarchical 특성 활용).
- 모든 포즈는 `/poses`에 한 번에 → 시퀀스 헤더 빠르게 읽음.
- `HighFive` (header-only C++17 wrapper)로 bind, OpenMP/thread 안전 모드 켬.

**입력 포맷 변환은 외부 일회성 스크립트** (`tools/kitti_to_h5.py`)로 제공.
core/io 코드는 변환 로직을 갖지 않음.

### 3.6 출력

- `pcd_writer`만 제공 (`static_map.pcd`, `dynamic_map.pcd`, `clean_scans/{i}.pcd`).
- 옵션으로 출력도 HDF5 (`output.h5`)로 떨어뜨림 — 후속 학습 파이프라인 연동 용이.

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

### 4.8 알고리즘 강화: 통합 Visibility 모델 (no MOS)

**문제 의식**. 도시 시퀀스에서 자주 보이는 *moving object의 누적 블롭* (예: 같은
차로에 수십 프레임에 걸쳐 쌓인 차량 흔적)은 보통 **MOS (Moving Object
Segmentation) → fine cleaning** 의 2단 파이프라인으로 처리된다. 그러나
여러 알고리즘을 직렬로 붙이면 의존성·튜닝 면이 늘고 SRP 헌장과도 맞지 않는다.

**가설**. 누적 블롭은 visibility 원리의 한계가 아니라 **v1 visibility 모델이 약한**
탓이다. 모델을 강화하면 단일 visibility 파이프라인만으로 흡수된다.

#### v1 visibility의 세 약점

| # | 약점 | 결과 |
|---|------|------|
| W1 | 픽셀당 map nearest **1점만** 비교 | 같은 방향에 여러 layer로 쌓인 동적점 중 1개만 검출 |
| W2 | "뒤에 있나?"만 판정, **자유 광선 통과 증거 미사용** | 광선이 통과한 모든 map 점에 누적 가능한 증거를 버림 |
| W3 | per-scan binary 라벨 + 정수 카운트 (`n_SM`/`n_DM`) | 시간 누적 증거가 약하고 hand-tuned 가중치 (`α_SM=0.3, α_DM=−0.7, τ_S=−0.1`)에 민감 |

#### 강화 모델 (3종 결합)

| 강화 | 수학 | 효과 |
|------|------|------|
| **A1: Multi-hit (depth peeling)** | 픽셀당 top-K nearest map 점 유지 (K=4~8) | 누적 layer 동시 테스트, W1 해소 |
| **A2: Free-segment ray carving** | 광선 (origin → query hit) 사이 모든 map 점에 "자유" 증거 += 1 | "통과" 정보 활용, W2 해소 |
| **A3: Log-odds 시간 누적** | `ℓ(p) ← ℓ(p) + log(p_obs/(1−p_obs))` per 관측 | 정적 prior 강하게, 카브 100회 누적 시 자연 제거. `α/τ_S` 매직 넘버 폐기 |

이 세 가지를 결합하면, 본질적으로 **range-image 도메인에서 작동하는 GPU
occupancy grid**가 된다 (수학적으로 voxel ray-tracing과 동치, 그러나 센서 네이티브
좌표를 유지하므로 GPU 효율 우수).

#### MOS 대비 장단

| | 강화 visibility (단일) | MOS + visibility (복합) |
|---|---|---|
| 정확도 (누적 블롭) | log-odds 누적이 충분히 카브 | MOS가 큰 덩어리 빠르게 제거, 잔여는 비슷 |
| 정확도 (희소 동적) | 동등 | MOS가 약함 (학습 도메인 밖) |
| 알고리즘 복잡도 | **1개** (visibility) | 2개 (MOS + visibility) + 인터페이스 |
| 학습 의존성 | 없음 | 있음 (도메인 의존) |
| GPU 적합성 | 융합 커널 1개 | 두 파이프라인 각각 |
| SRP 헌장 부합 | ✅ | ✗ |

**채택: 강화 visibility 단일 경로.**

#### 남는 한계 (visibility 원리상 풀 수 없음)

- **dynamic이 dynamic을 영구 가림** (예: 거대 트럭이 모든 시점에서 자전거 무리를
  가리는 경우) — MOS도 못 풀고, 추적/사전지식이 필요. v2 범위 외.
- **transient occlusion FP** (지나가는 차가 잠깐 가린 건물 점이 잘못 카브됨) —
  log-odds prior를 정적 쪽으로 강하게 (예: `ℓ_prior = +2.0`) 잡으면 1~2회 카브로는
  플립되지 않음. 해소.
- **정적이 정적을 영구 가림** — 사실 문제 아님 (가려진 정적점은 그대로 둬도 됨).

#### v2 구현 영향

- **§4.2의 융합 커널이 자연스럽게 흡수**: projection 시 픽셀당 top-K min을
  `cuda::atomic_ref<float>` + small-K register sort로 한 번에 수집.
  Free-segment carving은 같은 패스에서 ray-march 없이 `range < I_query`인 모든
  map 픽셀에 atomic add 1줄 추가.
- **메모리**: K=4 기준 range image 메모리 4배. KITTI에서 4 × 1 MB × 5 res = 20 MB,
  8 GB 예산 내 무시 가능.
- **Config 추가**: `KnnConfig` 옆에 `VisibilityConfig { int top_k; float
  log_odds_prior; float log_odds_carve; float log_odds_hit; float decision_thr; }`.
  v1의 `α_SM, α_DM, τ_S, τ_D, τ_S_thr` 전부 흡수·치환.
- **Phase 영향**: 본 강화는 P3 (CPU 레퍼런스) 단계부터 도입. P3에서 정확성 검증,
  P5 융합 커널 작성 시 동일 모델로 GPU 포팅. 새 phase 추가 없음.
- **회귀 비교**: v1과 출력이 다르므로 P0 골든 픽스처는 *동등하지 않음*. 대신
  SemanticKITTI 라벨 기준 TP/FP/FN 지표로 비교 — v1보다 **나아야** 통과.

---

## 5. 마이그레이션 단계 (Phase Plan)

각 단계는 **독립적으로 머지 가능**하며, 단계 끝마다 KITTI 00 미니셋으로 회귀.

| Phase | 내용 | 산출물 | 완료 기준 |
|-------|------|--------|-----------|
| **P0** | 베이스라인 캡처 + KITTI → HDF5 변환 스크립트 | v1 출력 + 변환된 `kitti00.h5` | 회귀 비교 스크립트 작동 |
| **P1** | `core/` 스켈레톤 + `Config` + `types.hpp` | 빌드만 통과 | `libremovert_core.a` 생성 |
| **P2** | `io/` 모듈: `IDatasetReader` + HDF5 구현 + `KittiDirReader` (호환) | HDF5/KITTI 둘 다 로드 | 단위 테스트 통과 |
| **P3** | `core/` CPU 레퍼런스 구현 (강화 visibility: multi-hit + ray carving + log-odds, §4.8) + `app/cli/` | CLI 단독 실행, SemanticKITTI 지표상 v1 ≥ | TP↑, FP/FN↓ |
| **P4** | Revert 단계 = log-odds prior 조정으로 흡수 (v1 TODO 해소) | Revert 함수 + 테스트 | 정성 평가 |
| **P5** | `gpu/` projection+carving 융합 커널 (raw CUDA + libcudacxx) | 단일 핫스팟 30× 달성 | 벤치 통과 |
| **P6** | cuVS KNN + Thrust downsample/transform 통합 | 엔드투엔드 30× 달성 | 8GB 이내, 30× 이상 |
| **P7** | 출력 HDF5 writer 옵션 + SQLite reader (선택) | 후속 파이프라인 연동 | 후속 도구 검증 |

---

## 6. 빌드/의존성

- **CMake 3.18+**, `target_compile_features(... cxx_std_17)`.
- 옵션: `REMOVERT_BUILD_GPU=ON/OFF`, `REMOVERT_BUILD_TESTS=ON/OFF`,
  `REMOVERT_WITH_SQLITE=ON/OFF` (기본 OFF).
- 의존성:
  - 필수: Eigen, HighFive (HDF5 C++17 wrapper, header-only), HDF5
  - 권장: PCL (PCD writer + CPU KdTree fallback). PCD 출력 불필요 시 미사용 가능
  - 선택(GPU): CUDA Toolkit 12.6+ (CCCL = Thrust+CUB+libcudacxx 번들), cuVS 24.10+
  - 선택(SQLite): `sqlite3` system lib
  - **ROS 의존성 없음** (의도된 결정, §0 C1)
- 죽은 의존성 제거 (v1 `CMakeLists.txt:27,46`의 GTSAM 주석 등).
- `clang-format`, `clang-tidy`, `cppcheck` 프리커밋.

---

## 7. 테스트 & 벤치

- **단위**: 각 순수 함수에 대해 GoogleTest. `core/`는 라인 커버리지 95% 목표.
- **속성 기반**: range image projection은 round-trip 테스트
  (project → unproject → 동일 픽셀 복원).
- **회귀**:
  - KITTI 00 처음 200 스캔 — 강화 visibility 적용으로 v1과 출력 다름 → SemanticKITTI 라벨 기준 **TP/FP/FN 지표가 v1 이상**이어야 통과.
  - 동일 입력 + 동일 Config → 동일 출력 (결정성 회귀).
- **벤치**: Google Benchmark.
  - `BM_project_cpu`, `BM_project_gpu`, `BM_pipeline_end_to_end`
  - CI에서 v1 대비 30× 미만 시 실패.
- **GPU/CPU 패리티**: P5 이후 매 PR.

---

## 8. 데이터셋 포맷 & 마이그레이션

v2는 **HDF5 기본 입력**. v1 사용자를 위한 호환 경로 제공:

| 시나리오 | 경로 |
|---------|------|
| 기존 KITTI 디렉토리(`.bin` + `poses.txt`) | `KittiDirReader` 직접 사용 (변환 불필요) |
| ROS bag → 오프라인 | 외부 도구 (`rosbag` 또는 `rosbags` Python lib)로 추출 후 `tools/bag_to_h5.py`로 변환 |
| 기타 (PCD 시퀀스, custom) | `tools/<src>_to_h5.py` 변환 스크립트 추가 |

- v1의 yaml 파라미터는 `io/config_loader`가 신규 `Config`로 매핑 (deprecation 경고).
- 출력 PCD 파일명·구조는 v1과 동일 유지 (`static_map.pcd` 등).
- ROS 토픽 발행은 v2에 없음 — 결과 PCD/HDF5를 원하는 시각화 도구로 로드.

---

## 9. 위험 & 대응

| 위험 | 대응 |
|------|------|
| GPU KNN의 정확도가 PCL FLANN과 미세 차이 | 허용 오차 정의 + CPU fallback 옵션 |
| 8 GB 초과 (대형 시퀀스) | 타일링 (map을 공간 분할) 옵션 추가 |
| HDF5 chunk 설정 잘못으로 random read 느림 | 권장 chunk 크기 (스캔당 1 chunk) 문서화 + 변환 스크립트 기본값 강제 |
| HDF5 멀티스레드 안전성 | HighFive `HDF5_USE_THREADSAFE=ON`, 또는 reader당 1 thread 정책 |
| 함수 50줄 제약이 가독성 해침 | 헬퍼 분해 + `inline` 권장, 강제는 lint 경고 수준에서 시작 |

---

## 10. 첫 PR 체크리스트 (P1 시작용)

- [ ] `core/` 디렉토리 + `CMakeLists.txt` 추가, ROS/외부 의존 0건 검증 스크립트
- [ ] `types.hpp` (Point, Pose, RangeImage, ScanFrame, Config, VisibilityConfig 전체)
- [ ] `io/config_loader` + JSON Schema (Config 검증), v1 yaml 호환 어댑터
- [ ] clang-tidy/clang-format 설정, CI green
- [ ] `tests/types_test.cpp` 컴파일·실행

---

*문서 버전*: 2026-04-25 초안.
