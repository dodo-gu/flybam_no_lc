#pragma once
// Temporary manual map calibration. Include after stdafx.h.
// One new key: A. Original arrows, Tab, F8 and backslash remain available.
// No map transformation, map editing, automatic fitting, or camera/DAQ calls.

#include <windows.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace manual_map {

// Suggested capture gates, not calibrated hardware latency estimates.
static const ULONGLONG PAUSE_MS = 300;
static const ULONGLONG MAX_SNAPSHOT_AGE_MS = 200;
static const double MAX_TARGET_MOTION_PX = 1.0;
static const double MAX_HEADING_CHANGE_DEG = 12.0;
static const unsigned long long MAX_PENDING_FRAMES = 2;

// Windows locks avoid mixing new C++ mutex primitives with the legacy
// ConcRT queue implementation. The locks are never held during image saving.
class Mutex {
    CRITICAL_SECTION cs_;
public:
    Mutex() { InitializeCriticalSection(&cs_); }
    ~Mutex() { DeleteCriticalSection(&cs_); }
    void lock() { EnterCriticalSection(&cs_); }
    void unlock() { LeaveCriticalSection(&cs_); }
private:
    Mutex(const Mutex&);
    Mutex& operator=(const Mutex&);
};
class Guard {
    Mutex& m_;
public:
    explicit Guard(Mutex& m) : m_(m) { m_.lock(); }
    ~Guard() { m_.unlock(); }
private:
    Guard(const Guard&);
    Guard& operator=(const Guard&);
};

// Counters measure application queue backlog, not camera exposure latency.
struct Counters {
    std::atomic<unsigned long long> arena_in, arena_out, fly_in, fly_out;
    Counters() : arena_in(0), arena_out(0), fly_in(0), fly_out(0) {}
};
inline Counters& counters() { static Counters c; return c; }
inline unsigned long long pending(const std::atomic<unsigned long long>& in,
                                  const std::atomic<unsigned long long>& out) {
    const unsigned long long a = in.load(), b = out.load();
    return a > b ? a - b : 0;
}
inline bool keyDown(int key) { return (GetAsyncKeyState(key) & 0x8000) != 0; }
inline bool ownsForeground() {
    HWND w = GetForegroundWindow();
    DWORD pid = 0;
    if (!w) return false;
    GetWindowThreadProcessId(w, &pid);
    return pid == GetCurrentProcessId();
}
inline bool finitePoint(cv::Point2f p) { return std::isfinite(p.x) && std::isfinite(p.y); }
inline double distance(cv::Point2f a, cv::Point2f b) {
    const double x = double(a.x) - b.x, y = double(a.y) - b.y;
    return std::sqrt(x*x + y*y);
}
inline double angleDifference(double a, double b) {
    return std::fabs(std::atan2(std::sin(a-b), std::cos(a-b))) * 180.0 / 3.14159265358979323846;
}
inline int nearest(cv::Point2f p, const std::vector<cv::Point2f>& points) {
    if (!finitePoint(p) || points.empty()) return -1;
    double best = (std::numeric_limits<double>::max)();
    int result = -1;
    for (std::size_t i = 0; i < points.size(); ++i) {
        const double x = double(p.x) - points[i].x, y = double(p.y) - points[i].y;
        const double d = x*x + y*y;
        if (d < best) { best = d; result = static_cast<int>(i); }
    }
    return result;
}

struct Pose {
    ULONGLONG ms;
    cv::Point2f head;
    double heading;
};
struct ArenaSnapshot {
    bool valid, head_mode;
    int fly_id, cycle_count;
    unsigned int flips;
    ULONGLONG processed_ms;
    cv::Point2f body, head;
    double heading;
    cv::Mat raw;
    ArenaSnapshot() : valid(false), head_mode(false), fly_id(0), cycle_count(0),
        flips(0), processed_ms(0), heading(0.0) {}
};

inline std::string pauseProblem(const ArenaSnapshot& a, const std::deque<Pose>& history,
                               ULONGLONG now, double& max_motion, double& max_turn) {
    max_motion = max_turn = 0.0;
    if (!a.valid) return "No valid head contour: wait for detection and initialization.";
    if (!a.head_mode) return "Use F8 to select HEAD before recording a head sample.";
    if (now < a.processed_ms || now-a.processed_ms > MAX_SNAPSHOT_AGE_MS)
        return "Arena snapshot is stale; check camera/processing progress.";
    if (history.size() < 2 || history.back().ms-history.front().ms < PAUSE_MS)
        return "Wait for a short stable pause, then press A again.";
    for (std::size_t i = 0; i < history.size(); ++i) {
        const double d = distance(history[i].head, a.head);
        const double t = angleDifference(history[i].heading, a.heading);
        if (d > max_motion) max_motion = d;
        if (t > max_turn) max_turn = t;
    }
    if (max_motion > MAX_TARGET_MOTION_PX || max_turn > MAX_HEADING_CHANGE_DEG)
        return "Fly/estimated head is moving: recenter during a pause and retry.";
    return std::string();
}

class Recorder {
    Mutex snapshot_mutex_;
    ArenaSnapshot arena_;
    std::deque<Pose> history_;
    cv::Mat fly_raw_;
    ULONGLONG fly_processed_ms_;
    int fly_stamp_;
    std::vector<cv::Point2f> points_, angles_;
    std::string directory_;
    std::ofstream csv_;
    bool ready_;
    unsigned int sequence_;
    ULONGLONG last_manual_move_ms_; // Keyboard worker only.

    static std::string createDirectory() {
        char cwd[MAX_PATH];
        const DWORD n = GetCurrentDirectoryA(MAX_PATH, cwd);
        if (!n || n >= MAX_PATH)
            throw std::runtime_error("Cannot resolve output directory.");

        // Walk upward from the launch working directory. A Git worktree
        // can have a .git file rather than a directory; both are accepted.
        // This also works when launched from a nested Debug/Release folder.
        const auto parentFolder = [](std::string path) -> std::string {
            while (path.size() > 3 &&
                   (path.back() == '\\' || path.back() == '/'))
                path.pop_back();
            if (path.size() == 3 && path[1] == ':') return std::string();
            if (path.compare(0, 2, "\\\\") == 0) {
                const std::size_t server_end = path.find('\\', 2);
                if (server_end == std::string::npos ||
                    path.find('\\', server_end + 1) == std::string::npos)
                    return std::string(); // Do not walk above a UNC share.
            }
            const std::size_t slash = path.find_last_of("\\/");
            if (slash == std::string::npos || slash == 0) return std::string();
            if (slash == 2 && path[1] == ':') return path.substr(0, 3);
            return path.substr(0, slash);
        };

        std::string folder(cwd), repository_root, solution_root;
        while (!folder.empty()) {
            if (GetFileAttributesA((folder + "\\.git").c_str()) !=
                INVALID_FILE_ATTRIBUTES) {
                repository_root = folder;
                break;
            }
            if (solution_root.empty() &&
                GetFileAttributesA((folder + "\\flybam.sln").c_str()) !=
                    INVALID_FILE_ATTRIBUTES)
                solution_root = folder;
            folder = parentFolder(folder);
        }
        // Also support a source copy downloaded without Git metadata.
        if (repository_root.empty()) repository_root = solution_root;
        if (repository_root.empty())
            throw std::runtime_error(
                "Cannot locate the FlyBAM repository above the working directory. "
                "Launch from the repository or one of its build subfolders.");
        const std::string parent = parentFolder(repository_root);
        if (parent.empty())
            throw std::runtime_error("Repository has no usable parent directory.");
        const std::string output_root = parent +
            ((parent.back() == '\\' || parent.back() == '/') ? "" : "\\") +
            "manual calibration";

        if (!CreateDirectoryA(output_root.c_str(), 0)) {
            const DWORD error = GetLastError();
            const DWORD attributes = GetFileAttributesA(output_root.c_str());
            if (error != ERROR_ALREADY_EXISTS ||
                attributes == INVALID_FILE_ATTRIBUTES ||
                !(attributes & FILE_ATTRIBUTE_DIRECTORY))
                throw std::runtime_error(
                    "Cannot create calibration output root: " + output_root +
                    " (Windows error " + std::to_string(error) + ")");
        }

        SYSTEMTIME t; GetLocalTime(&t);
        for (int k = 0; k < 1000; ++k) {
            char name[160];
            std::snprintf(name, sizeof(name),
                "manual_map_%04u%02u%02u_%02u%02u%02u_pid%lu_%03d",
                unsigned(t.wYear), unsigned(t.wMonth), unsigned(t.wDay),
                unsigned(t.wHour), unsigned(t.wMinute), unsigned(t.wSecond),
                static_cast<unsigned long>(GetCurrentProcessId()), k);
            const std::string dir = output_root + "\\" + name;
            if (CreateDirectoryA(dir.c_str(), 0)) return dir;
            if (GetLastError() != ERROR_ALREADY_EXISTS)
                throw std::runtime_error("Cannot create calibration output folder: " + dir);
        }
        throw std::runtime_error("Cannot allocate a new calibration output folder.");
    }
    static void writeText(const std::string& path, const std::string& text) {
        std::ofstream out(path.c_str(), std::ios::binary);
        if (!out) throw std::runtime_error("Cannot create: " + path);
        out.write(text.data(), static_cast<std::streamsize>(text.size()));
        out.flush();
        if (!out) throw std::runtime_error("Write failed: " + path);
    }
    void reject(const std::string& why) const {
        std::printf("[MANUAL MAP] Not saved: %s\n", why.c_str());
    }

public:
    Recorder(const char* source_path, const std::vector<cv::Point2f>& points,
             const std::vector<cv::Point2f>& angles)
        : fly_processed_ms_(0), fly_stamp_(0), points_(points), angles_(angles),
          ready_(false), sequence_(0), last_manual_move_ms_(0) {
        try {
            if (points.empty() || points.size() != angles.size())
                throw std::runtime_error("Loaded baseline map is empty or inconsistent.");
            for (std::size_t i = 0; i < points.size(); ++i)
                if (!finitePoint(points[i]) || !finitePoint(angles[i]))
                    throw std::runtime_error("Loaded baseline has non-finite numbers.");
            directory_ = createDirectory();
            // Authoritative baseline: exactly the valid rows loaded by FlyBAM.
            // No second numeric parsing and no exact-float comparison of readers.
            std::ostringstream loaded;
            loaded.imbue(std::locale::classic());
            loaded << std::setprecision(std::numeric_limits<float>::max_digits10);
            for (std::size_t i = 0; i < points.size(); ++i)
                loaded << angles[i].x << ' ' << angles[i].y << ' '
                       << points[i].x << ' ' << points[i].y << '\n';
            writeText(directory_ + "\\baseline_loaded.txt", loaded.str());
            std::ostringstream info;
            info << "Temporary manual FlyBAM map calibration\n"
                 << "source_branch=HeadDirection\n"
                 << "source_commit=b5b2e641a9fac62ea46aee1da963524340cee22d\n"
                 << "source_path_as_configured=" << source_path << '\n'
                 << "baseline_valid_rows=" << points.size() << '\n'
                 << "baseline_loaded.txt=authoritative valid map rows used at startup\n"
                 << "source_copy.txt=optional unparsed copy of source file, not revalidated\n"
                 << "map_index=zero-based index into baseline_loaded.txt\n"
                 << "delta_angle=manual_command_minus_baseline_nearest_at_current_head\n"
                 << "angles=commanded galvo angles in existing project units (degrees)\n"
                 << "image_coordinates=arena ROI, x right, y down\n"
                 << "target=head estimate; review raw images to verify the same anatomical landmark\n"
                 << "timing=host processing timestamps, NOT synchronized exposure timestamps\n"
                 << "centering=operator annotation, NOT verified optical-axis feedback\n"
                 << "original_map_modified=false\n";
            writeText(directory_ + "\\session.txt", info.str());
            std::ifstream src(source_path, std::ios::binary);
            if (src) {
                std::ostringstream bytes; bytes << src.rdbuf();
                writeText(directory_ + "\\source_copy.txt", bytes.str());
            }
            csv_.open((directory_ + "\\samples.csv").c_str(), std::ios::out);
            csv_.imbue(std::locale::classic());
            csv_ << std::setprecision(17);
            csv_ << "sample_id,host_ms,fly_id,head_mode,head_flips,body_x,body_y,head_x,head_y,heading_rad,"
                 << "map_index,map_x,map_y,map_distance_px,baseline_angle_x,baseline_angle_y,"
                 << "manual_angle_x,manual_angle_y,delta_angle_x,delta_angle_y,"
                 << "target_motion_px,heading_change_deg,arena_processed_age_ms,fly_processed_age_ms,"
                 << "arena_pending_estimate,fly_pending_estimate,arena_cycle_count,fly_stamp,fly_center_x,fly_center_y,arena_png,fly_png\n";
            csv_.flush();
            if (!csv_) throw std::runtime_error("Cannot initialize sample CSV.");
            ready_ = true;
            std::printf("[MANUAL MAP] Output: %s\n", directory_.c_str());
            std::printf("[MANUAL MAP] Arrows: center head. Release, pause, A: record. Tab: arena tracking.\n");
        } catch (const std::exception& e) {
            std::printf("[MANUAL MAP] Recorder disabled: %s\n", e.what());
        }
    }

    // Called by the arena worker even when detection fails.
    // The raw Mat is a reference-counted snapshot; do not subsequently edit its pixels.
    void publishArena(const cv::Mat& raw, bool valid, bool head_mode, int fly_id,
                      cv::Point2f body, cv::Point2f head, double heading,
                      unsigned int flips, int cycle_count) {
        const ULONGLONG now = GetTickCount64();
        Guard lock(snapshot_mutex_);
        valid = valid && finitePoint(head) && finitePoint(body) && std::isfinite(heading);
        if (!valid || !head_mode || arena_.fly_id != fly_id || arena_.flips != flips ||
            !arena_.valid || !arena_.head_mode ||
            (arena_.processed_ms && now-arena_.processed_ms > MAX_SNAPSHOT_AGE_MS)) history_.clear();
        arena_.raw = raw; arena_.valid = valid; arena_.head_mode = head_mode;
        arena_.fly_id = fly_id; arena_.body = body; arena_.head = head;
        arena_.heading = heading; arena_.flips = flips; arena_.cycle_count = cycle_count;
        arena_.processed_ms = now;
        if (valid && head_mode) {
            Pose p; p.ms = now; p.head = head; p.heading = heading;
            history_.push_back(p);
            while (history_.size() > 2 && now-history_[1].ms >= PAUSE_MS) history_.pop_front();
            while (history_.size() > 128) history_.pop_front();
        }
    }
    void publishFly(const cv::Mat& raw, int stamp) {
        Guard lock(snapshot_mutex_);
        fly_raw_ = raw; fly_stamp_ = stamp; fly_processed_ms_ = GetTickCount64();
    }
    void noteManualMove() { last_manual_move_ms_ = GetTickCount64(); }

    // Called only from the keyboard worker after arrows have been released.
    // The caller captures the commanded angles while holding the galvo lock.
    void save(cv::Point2f manual_angles) {
        if (!ready_) { reject("Recorder is disabled; check startup console."); return; }
        const ULONGLONG now = GetTickCount64();
        if (now-last_manual_move_ms_ < PAUSE_MS) {
            reject("Release arrows and wait at least 0.3 seconds before A."); return;
        }
        ArenaSnapshot a; std::deque<Pose> h; cv::Mat fly;
        ULONGLONG fly_ms; int fly_stamp;
        {
            Guard lock(snapshot_mutex_);
            a = arena_; h = history_; fly = fly_raw_;
            fly_ms = fly_processed_ms_; fly_stamp = fly_stamp_;
        }
        double motion = 0, turn = 0;
        const std::string problem = pauseProblem(a, h, now, motion, turn);
        if (!problem.empty()) { reject(problem); return; }
        if (fly.empty() || !fly_ms || now < fly_ms || now-fly_ms > MAX_SNAPSHOT_AGE_MS) {
            reject("Fly-image snapshot is stale or missing."); return;
        }
        if (a.raw.empty() || !finitePoint(manual_angles)) {
            reject("Missing arena image or non-finite galvo command."); return;
        }
        const unsigned long long aq = pending(counters().arena_in, counters().arena_out);
        const unsigned long long fq = pending(counters().fly_in, counters().fly_out);
        if (aq > MAX_PENDING_FRAMES || fq > MAX_PENDING_FRAMES) {
            reject("Camera processing has a queue backlog; wait or resolve freezing first."); return;
        }
        const int j = nearest(a.head, points_);
        if (j < 0) { reject("No valid baseline map point."); return; }
        const double dx = double(manual_angles.x)-angles_[j].x;
        const double dy = double(manual_angles.y)-angles_[j].y;
        // Allocate a new id even after a failed disk write, never overwrite an old image.
        const unsigned int id = ++sequence_;
        char prefix[40]; std::snprintf(prefix, sizeof(prefix), "sample_%04u", id);
        const std::string arena_name = std::string(prefix) + "_arena.png";
        const std::string fly_name = std::string(prefix) + "_fly.png";
        try {
            if (!cv::imwrite(directory_+"\\"+arena_name, a.raw) ||
                !cv::imwrite(directory_+"\\"+fly_name, fly))
                throw std::runtime_error("Could not write sample image(s).");
            csv_ << id << ',' << now << ',' << a.fly_id << ",HEAD," << a.flips << ','
                 << a.body.x << ',' << a.body.y << ',' << a.head.x << ',' << a.head.y << ',' << a.heading << ','
                 << j << ',' << points_[j].x << ',' << points_[j].y << ',' << distance(a.head,points_[j]) << ','
                 << angles_[j].x << ',' << angles_[j].y << ',' << manual_angles.x << ',' << manual_angles.y << ','
                 << dx << ',' << dy << ',' << motion << ',' << turn << ','
                 << now-a.processed_ms << ',' << now-fly_ms << ',' << aq << ',' << fq << ','
                 << a.cycle_count << ',' << fly_stamp << ',' << fly.cols/2 << ',' << fly.rows/2 << ','
                 << arena_name << ',' << fly_name << '\n';
            csv_.flush();
            if (!csv_) { ready_ = false; throw std::runtime_error("CSV write failed; recording disabled."); }
            std::printf("[MANUAL MAP] Saved sample %u: head=(%.3f,%.3f), delta angles=(%+.6f,%+.6f)\n",
                id, double(a.head.x), double(a.head.y), dx, dy);
        } catch (const std::exception& e) { reject(e.what()); }
    }
};
} // namespace manual_map
