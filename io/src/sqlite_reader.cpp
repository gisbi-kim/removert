#include "removert/io/sqlite_reader.hpp"

#if REMOVERT_WITH_SQLITE

#include <sqlite3.h>

#include <cstring>
#include <stdexcept>
#include <string>

namespace removert::io {

namespace {

[[noreturn]] void die(sqlite3* db, std::string const& what) {
    std::string msg = what;
    if (db) { msg += ": "; msg += sqlite3_errmsg(db); }
    throw std::runtime_error(msg);
}

// Fetch a calib BLOB by key. Returns false if missing.
bool fetch_calib_blob(sqlite3* db, std::string const& key,
                      int& rows, int& cols, std::vector<double>& data) {
    sqlite3_stmt* st = nullptr;
    if (sqlite3_prepare_v2(db,
            "SELECT rows, cols, data FROM calib WHERE key = ?",
            -1, &st, nullptr) != SQLITE_OK) die(db, "prepare calib");
    sqlite3_bind_text(st, 1, key.c_str(), -1, SQLITE_TRANSIENT);
    bool ok = false;
    if (sqlite3_step(st) == SQLITE_ROW) {
        rows = sqlite3_column_int(st, 0);
        cols = sqlite3_column_int(st, 1);
        auto const  n  = static_cast<std::size_t>(rows) *
                         static_cast<std::size_t>(cols);
        auto const  bytes = static_cast<std::size_t>(sqlite3_column_bytes(st, 2));
        if (bytes >= n * sizeof(double)) {
            data.resize(n);
            std::memcpy(data.data(), sqlite3_column_blob(st, 2), n * sizeof(double));
            ok = true;
        }
    }
    sqlite3_finalize(st);
    return ok;
}

Eigen::Matrix4f read_pose_blob(void const* blob, int bytes) {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    if (bytes >= static_cast<int>(16 * sizeof(float)))
        std::memcpy(T.data(), blob, 16 * sizeof(float));
    // Eigen is column-major by default; the DB stores row-major float32.
    // Swap by transposing the in-memory Map.
    Eigen::Matrix4f Trm;
    std::memcpy(Trm.data(), blob, 16 * sizeof(float));
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> rm;
    std::memcpy(rm.data(), blob, 16 * sizeof(float));
    T = rm;  // row-major -> column-major copy
    return T;
}

}  // namespace

SqliteDatasetReader::SqliteDatasetReader(std::filesystem::path const& db_path,
                                         SensorConfig override_sensor) {
    if (sqlite3_open_v2(db_path.c_str(), &db_,
                        SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK)
        die(db_, "open " + db_path.string());

    sensor_ = override_sensor;
    if (sensor_.vfov_deg <= 0.0f) sensor_.vfov_deg = 50.0f;
    if (sensor_.hfov_deg <= 0.0f) sensor_.hfov_deg = 360.0f;

    load_calib_();
    count_frames_();
}

SqliteDatasetReader::~SqliteDatasetReader() {
    if (db_) sqlite3_close(db_);
}

SqliteDatasetReader::SqliteDatasetReader(SqliteDatasetReader&& other) noexcept
    : db_(other.db_), sensor_(other.sensor_),
      Tr_velo_cam_(other.Tr_velo_cam_), frame_count_(other.frame_count_) {
    other.db_ = nullptr;
}

SqliteDatasetReader&
SqliteDatasetReader::operator=(SqliteDatasetReader&& other) noexcept {
    if (this != &other) {
        if (db_) sqlite3_close(db_);
        db_           = other.db_;
        sensor_       = other.sensor_;
        Tr_velo_cam_  = other.Tr_velo_cam_;
        frame_count_  = other.frame_count_;
        other.db_     = nullptr;
    }
    return *this;
}

void SqliteDatasetReader::load_calib_() {
    std::vector<double> R, t;
    int rR{}, cR{}, rT{}, cT{};
    bool const have_R = fetch_calib_blob(db_, "velo_to_cam_R", rR, cR, R);
    bool const have_T = fetch_calib_blob(db_, "velo_to_cam_T", rT, cT, t);
    if (!have_R || !have_T || R.size() != 9 || t.size() != 3) return;

    Eigen::Matrix4f Tr = Eigen::Matrix4f::Identity();
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            Tr(r, c) = static_cast<float>(R[static_cast<std::size_t>(r * 3 + c)]);
    Tr(0, 3) = static_cast<float>(t[0]);
    Tr(1, 3) = static_cast<float>(t[1]);
    Tr(2, 3) = static_cast<float>(t[2]);
    Tr_velo_cam_ = Tr;
}

void SqliteDatasetReader::count_frames_() {
    sqlite3_stmt* st = nullptr;
    if (sqlite3_prepare_v2(db_, "SELECT COUNT(*) FROM frames",
                           -1, &st, nullptr) != SQLITE_OK) die(db_, "count frames");
    if (sqlite3_step(st) == SQLITE_ROW)
        frame_count_ = static_cast<std::size_t>(sqlite3_column_int64(st, 0));
    sqlite3_finalize(st);
}

ScanFrame SqliteDatasetReader::read(std::size_t idx) const {
    sqlite3_stmt* st = nullptr;
    if (sqlite3_prepare_v2(db_,
            "SELECT f.timestamp_ns, f.pose_cam0, s.num_points, s.points "
            "FROM frames f JOIN scans s USING(frame_idx) "
            "WHERE f.frame_idx = ?",
            -1, &st, nullptr) != SQLITE_OK) die(db_, "prepare frame");
    sqlite3_bind_int64(st, 1, static_cast<sqlite3_int64>(idx));

    ScanFrame frame;
    if (sqlite3_step(st) != SQLITE_ROW) {
        sqlite3_finalize(st);
        throw std::out_of_range("SqliteDatasetReader::read idx out of range");
    }
    frame.stamp_ns = sqlite3_column_int64(st, 0);

    auto const pose_blob  = sqlite3_column_blob(st, 1);
    auto const pose_bytes = sqlite3_column_bytes(st, 1);
    Eigen::Matrix4f const T_pose = read_pose_blob(pose_blob, pose_bytes);
    frame.T_world_lidar.T = T_pose * Tr_velo_cam_;

    int  const npts  = sqlite3_column_int(st, 2);
    auto const blob  = sqlite3_column_blob(st, 3);
    auto const bytes = sqlite3_column_bytes(st, 3);
    int  const have  = static_cast<int>(static_cast<std::size_t>(bytes) / sizeof(Point));
    int  const n     = (npts > 0 && npts <= have) ? npts : have;
    frame.points_local.resize(static_cast<std::size_t>(n));
    if (n > 0)
        std::memcpy(frame.points_local.data(), blob,
                    static_cast<std::size_t>(n) * sizeof(Point));

    sqlite3_finalize(st);
    return frame;
}

}  // namespace removert::io

#endif  // REMOVERT_WITH_SQLITE
