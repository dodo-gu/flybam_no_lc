#pragma once
// Header-only Windows/OpenCV adapter. Include AFTER stdafx.h.
// All key polling stays in the existing keyboard section.
// This module NEVER calls the camera SDK, the DAQ, imshow, or waitKey.
#ifndef _WIN32
#error map_diagnostic.h requires the Windows build of FlyBAM.
#endif

#include "map_diagnostic_core.h"
#include <windows.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cwchar>
#include <future>
#include <memory>
#include <utility>

namespace flybam_mapdiag {

namespace win_detail {
struct Handle {
    HANDLE value;
    explicit Handle(HANDLE h) : value(h) {}
    ~Handle() { if (value != INVALID_HANDLE_VALUE) CloseHandle(value); }
private:
    Handle(const Handle&);
    Handle& operator=(const Handle&);
};
inline std::runtime_error error(const char* operation) {
    return std::runtime_error(std::string(operation) + "; Windows error " +
                              std::to_string(GetLastError()));
}
inline std::wstring wide(const std::string& s) {
    if (s.empty()) return std::wstring();
    int n = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, s.data(), int(s.size()), 0, 0);
    if (!n) throw error("UTF-8 path conversion failed");
    std::wstring result(n, L'\0');
    MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, s.data(), int(s.size()), &result[0], n);
    return result;
}
inline std::string utf8(const std::wstring& s) {
    if (s.empty()) return std::string();
    int n = WideCharToMultiByte(CP_UTF8, 0, s.data(), int(s.size()), 0, 0, 0, 0);
    if (!n) throw error("Path display conversion failed");
    std::string result(n, '\0');
    WideCharToMultiByte(CP_UTF8, 0, s.data(), int(s.size()), &result[0], n, 0, 0);
    return result;
}
inline std::wstring fullPath(const char* relative) {
    const std::wstring input = wide(relative);
    DWORD needed = GetFullPathNameW(input.c_str(), 0, 0, 0);
    if (!needed) throw error("Cannot resolve source map path");
    std::vector<wchar_t> buffer(needed + 1);
    DWORD got = GetFullPathNameW(input.c_str(), DWORD(buffer.size()), &buffer[0], 0);
    if (!got || got >= buffer.size()) throw error("Cannot resolve source map path");
    return std::wstring(&buffer[0], got);
}
inline std::string readFile(const std::wstring& path) {
    Handle h(CreateFileW(path.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE,
                         0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0));
    if (h.value == INVALID_HANDLE_VALUE) throw error("Cannot read source map");
    LARGE_INTEGER size;
    if (!GetFileSizeEx(h.value, &size)) throw error("Cannot read source map size");
    if (size.QuadPart < 0 || size.QuadPart > 64LL * 1024 * 1024)
        throw std::runtime_error("Source map exceeds the diagnostic 64 MiB size limit.");
    std::string bytes(static_cast<std::size_t>(size.QuadPart), '\0');
    std::size_t offset = 0;
    while (offset < bytes.size()) {
        DWORD read = 0;
        const DWORD requested = DWORD((std::min)(bytes.size() - offset, std::size_t(1024 * 1024)));
        if (!ReadFile(h.value, &bytes[offset], requested, &read, 0) || !read)
            throw error("Source map read failed");
        offset += read;
    }
    return bytes;
}
inline void writeNew(const std::wstring& path, const std::string& bytes) {
    // CREATE_NEW refuses to overwrite an existing file.
    Handle h(CreateFileW(path.c_str(), GENERIC_WRITE, 0, 0, CREATE_NEW,
                         FILE_ATTRIBUTE_NORMAL, 0));
    if (h.value == INVALID_HANDLE_VALUE) throw error("Cannot create export file");
    std::size_t offset = 0;
    while (offset < bytes.size()) {
        DWORD wrote = 0;
        const DWORD requested = DWORD((std::min)(bytes.size() - offset, std::size_t(1024 * 1024)));
        if (!WriteFile(h.value, bytes.data() + offset, requested, &wrote, 0) || !wrote)
            throw error("Export write failed");
        offset += wrote;
    }
    if (!FlushFileBuffers(h.value)) throw error("Export flush failed");
}
inline std::wstring exportDirectory(const std::wstring& source_path, std::string& time_text) {
    const std::size_t slash = source_path.find_last_of(L"\\/");
    if (slash == std::wstring::npos) throw std::runtime_error("Source map has no parent folder.");
    const std::wstring parent = source_path.substr(0, slash) + L"\\map_diagnostic_exports";
    if (!CreateDirectoryW(parent.c_str(), 0)) {
        if (GetLastError() != ERROR_ALREADY_EXISTS) throw error("Cannot create export parent folder");
        const DWORD attributes = GetFileAttributesW(parent.c_str());
        if (attributes == INVALID_FILE_ATTRIBUTES || !(attributes & FILE_ATTRIBUTE_DIRECTORY))
            throw std::runtime_error("Export parent path is not a directory.");
    }
    SYSTEMTIME t; GetLocalTime(&t);
    wchar_t stamp[128];
    std::swprintf(stamp, 128, L"%04u%02u%02u_%02u%02u%02u_%03u_pid%lu",
                  unsigned(t.wYear), unsigned(t.wMonth), unsigned(t.wDay),
                  unsigned(t.wHour), unsigned(t.wMinute), unsigned(t.wSecond),
                  unsigned(t.wMilliseconds), static_cast<unsigned long>(GetCurrentProcessId()));
    time_text = utf8(stamp);
    for (unsigned n = 0; n < 10000; ++n) {
        const std::wstring directory = parent + L"\\map_" + stamp + L"_" + std::to_wstring(n);
        if (CreateDirectoryW(directory.c_str(), 0)) return directory;
        if (GetLastError() != ERROR_ALREADY_EXISTS) throw error("Cannot create unique export folder");
    }
    throw std::runtime_error("Cannot allocate a unique export folder.");
}
inline bool ownsForeground() {
    HWND window = GetForegroundWindow();
    if (!window) return false;
    DWORD process = 0;
    GetWindowThreadProcessId(window, &process);
    return process == GetCurrentProcessId();
}
} // namespace win_detail

struct View {
    Settings settings;
    std::vector<cv::Point2f> points;
};
typedef std::shared_ptr<const View> ViewPtr;

class MapDiagnostic {
    struct ExportResult { bool ok; std::string message; };
    std::shared_ptr<const Document> document_;
    std::wstring source_path_;
    ViewPtr current_;
    bool ready_;
    int width_, height_;
    bool was_down_[9];
    ULONGLONG last_poll_;
    std::future<ExportResult> export_job_;
    std::atomic<int> export_status_; // 0 none, 1 writing, 2 saved, 3 failed

    ViewPtr buildView(const Settings& settings) const {
        std::shared_ptr<View> v(new View);
        v->settings = settings;
        const std::vector<Point> points = buildPoints(*document_, settings);
        v->points.reserve(points.size());
        for (std::size_t i = 0; i < points.size(); ++i)
            v->points.push_back(cv::Point2f(points[i].x, points[i].y));
        return v;
    }
    static ExportResult exportFiles(std::shared_ptr<const Document> document,
                                    ViewPtr view, std::wstring source,
                                    int width, int height, bool head_mode) {
        ExportResult result;
        result.ok = false;
        std::wstring directory;
        try {
            const ExportText text = serializeMap(*document, view->settings, width, height);
            // Round-trip check using the SAME validity rule as the existing loader.
            const Document reloaded = parseMap(text.map_text);
            if (reloaded.valid_points.size() != view->points.size())
                throw std::runtime_error("Export validation failed: valid point count changed.");
            for (std::size_t i = 0; i < view->points.size(); ++i) {
                if (reloaded.valid_points[i].x != view->points[i].x ||
                    reloaded.valid_points[i].y != view->points[i].y ||
                    reloaded.valid_angles[i].x != document->valid_angles[i].x ||
                    reloaded.valid_angles[i].y != document->valid_angles[i].y)
                    throw std::runtime_error("Export validation failed: preview/map mismatch.");
            }
            std::string time;
            directory = win_detail::exportDirectory(source, time);
            const std::string metadata = report(*document, view->settings, text,
                win_detail::utf8(source), time, head_mode ? "HEAD" : "CENTROID", width, height);
            win_detail::writeNew(directory + L"\\source_map.txt", document->original_bytes);
            win_detail::writeNew(directory + L"\\map_modified.txt", text.map_text);
            win_detail::writeNew(directory + L"\\row_deltas.csv", text.delta_csv);
            win_detail::writeNew(directory + L"\\transform.txt", metadata);
            // Write this last. Partial exports never get a completion marker.
            win_detail::writeNew(directory + L"\\COMPLETE.txt",
                "Export completed and float-coordinate round-trip verified.\n"
                "This is a candidate map, not an automatic accuracy validation.\n");
            result.ok = true;
            result.message = "Saved revision " + std::to_string(view->settings.revision) +
                " to " + win_detail::utf8(directory) +
                " | zero-guard rows=" + std::to_string(text.zero_guard_rows) +
                " | outside-image rows=" + std::to_string(text.outside_image_rows);
        } catch (const std::exception& e) {
            result.message = std::string("EXPORT FAILED: ") + e.what();
            if (!directory.empty()) result.message += " | incomplete folder=" + win_detail::utf8(directory);
        }
        return result;
    }
    void requestExport(bool head_mode) {
        if (export_job_.valid()) {
            std::printf("[MAP DIAG] Export already running; no second job was queued.\n");
            return;
        }
        const ViewPtr view = snapshot();
        if (!view->settings.preview) {
            std::printf("[MAP DIAG] ORIGINAL comparison is active. Press B before exporting.\n");
            return;
        }
        try {
            export_status_.store(1);
            export_job_ = std::async(std::launch::async, &MapDiagnostic::exportFiles,
                document_, view, source_path_, width_, height_, head_mode);
            std::printf("[MAP DIAG] Exporting snapshot rev=%lu dx=%.4f dy=%.4f scale=%.9f\n",
                view->settings.revision, view->settings.tx(), view->settings.ty(), view->settings.scale());
        } catch (const std::exception& e) {
            export_status_.store(3);
            std::printf("[MAP DIAG] Cannot start export: %s\n", e.what());
        }
    }

public:
    MapDiagnostic(const char* source_path, const std::vector<cv::Point2f>& points,
                  const std::vector<cv::Point2f>& angles, int width, int height)
        : ready_(false), width_(width), height_(height), last_poll_(0), export_status_(0) {
        for (int i = 0; i < 9; ++i) was_down_[i] = false;
        std::shared_ptr<View> fallback(new View);
        fallback->settings = Settings((width - 1) / 2.0, (height - 1) / 2.0);
        fallback->points = points;
        current_ = fallback;
        try {
            source_path_ = win_detail::fullPath(source_path);
            std::shared_ptr<Document> d(new Document(parseMap(win_detail::readFile(source_path_))));
            if (d->valid_points.size() != points.size() || points.size() != angles.size())
                throw std::runtime_error("Source map does not match the application's loaded arrays.");
            for (std::size_t i = 0; i < points.size(); ++i) {
                if (d->valid_points[i].x != points[i].x || d->valid_points[i].y != points[i].y ||
                    d->valid_angles[i].x != angles[i].x || d->valid_angles[i].y != angles[i].y)
                    throw std::runtime_error("Source map rows do not match the application's loaded arrays.");
            }
            document_ = d;
            current_ = buildView(fallback->settings);
            ready_ = true;
            std::printf("\n[MAP DIAG] TEMPORARY diagnostic enabled. Baseline map is READ-ONLY.\n");
            std::printf("[MAP DIAG] Source: %s\n", win_detail::utf8(source_path_).c_str());
            std::printf("[MAP DIAG] W/A/S/D shift map; Z/X expand/contract; B compare; R reset; P export.\n");
            std::printf("[MAP DIAG] Click an OpenCV image window. One action per key press, no modifiers.\n");
        } catch (const std::exception& e) {
            std::printf("[MAP DIAG] DISABLED: %s\n", e.what());
        }
    }
    ~MapDiagnostic() { if (export_job_.valid()) export_job_.wait(); }
    ViewPtr snapshot() const { return std::atomic_load(&current_); }

    // Call ONLY from the original keyboard section; never from an image worker.
    void keyboardTick(bool arena_controls_active, bool head_mode) {
        const ULONGLONG now = GetTickCount64();
        if (now - last_poll_ < 4) return; // Rate limit this module, not the original keyboard loop.
        last_poll_ = now;
        if (export_job_.valid() && export_job_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            const ExportResult result = export_job_.get();
            export_status_.store(result.ok ? 2 : 3);
            std::printf("[MAP DIAG] %s\n", result.message.c_str());
        }
        const char keys[9] = {'W', 'A', 'S', 'D', 'Z', 'X', 'P', 'R', 'B'};
        bool pressed[9];
        for (int i = 0; i < 9; ++i) {
            const bool down = (GetAsyncKeyState(keys[i]) & 0x8000) != 0;
            pressed[i] = down && !was_down_[i];
            was_down_[i] = down;
        }
        // New diagnostic keys do not fire while typing in another application,
        // using Ctrl/Alt/Shift shortcuts, or operating manual/bead control.
        const bool modifiers = ((GetAsyncKeyState(VK_CONTROL) | GetAsyncKeyState(VK_MENU) |
                                 GetAsyncKeyState(VK_SHIFT)) & 0x8000) != 0;
        if (!ready_ || !arena_controls_active || !win_detail::ownsForeground() || modifiers) return;
        Settings next = snapshot()->settings;
        bool changed = false;
        for (int i = 0; i < 9; ++i) {
            if (!pressed[i] || keys[i] == 'P') continue;
            if (changeSettings(next, keys[i])) changed = true;
            else std::printf("[MAP DIAG] Adjustment ignored: ORIGINAL preview or configured limit.\n");
        }
        if (changed) {
            try {
                const ViewPtr replacement = buildView(next);
                // Publish a fully built, immutable point array. No array is mutated
                // while the arena worker searches it. Old snapshots remain alive.
                std::atomic_store(&current_, replacement);
                std::printf("[MAP DIAG] rev=%lu %s dx=%+.3f dy=%+.3f scale=%.9f\n",
                    next.revision, next.preview ? "ADJUSTED" : "ORIGINAL", next.tx(), next.ty(), next.scale());
            } catch (const std::exception& e) {
                std::printf("[MAP DIAG] Adjustment rejected: %s\n", e.what());
            }
        }
        if (pressed[6]) requestExport(head_mode);
    }

    void drawOverlay(cv::Mat& frame, const ViewPtr& view) const {
        const Settings& s = view->settings;
        std::ostringstream text;
        text.imbue(std::locale::classic());
        text << "MAP DIAG " << (ready_ ? (s.preview ? "ADJ" : "ORIG") : "DISABLED")
             << " r" << s.revision << std::fixed << std::setprecision(2)
             << " dx=" << s.tx() << " dy=" << s.ty()
             << std::setprecision(3) << " scale=" << s.scale();
        cv::putText(frame, text.str(), cv::Point(8, 82), cv::FONT_HERSHEY_SIMPLEX,
                    0.38, cv::Scalar(255, 255, 0), 1);
        cv::putText(frame, "WASD shift | Z/X scale | B compare | R reset | P save",
                    cv::Point(8, 98), cv::FONT_HERSHEY_SIMPLEX, 0.36, cv::Scalar(255, 255, 0), 1);
        const int status = export_status_.load();
        const char* label = status == 1 ? "Export: writing snapshot" :
            status == 2 ? "Export: saved (see console path)" :
            status == 3 ? "Export: FAILED (see console)" : "Export: none; original map is unchanged";
        cv::putText(frame, label, cv::Point(8, 114), cv::FONT_HERSHEY_SIMPLEX,
                    0.36, cv::Scalar(255, 255, 0), 1);
    }
};

} // namespace flybam_mapdiag
