
#pragma once
// Temporary FlyBAM map diagnostic: portable C++11 geometry and export logic.
// This file contains no camera, DAQ, OpenCV, keyboard, or filesystem calls.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace flybam_mapdiag {

static const char* const SOURCE_COMMIT =
    "25cff36faf80ed1d2aec6db83ae58cabca4f2a84";

// Tune these before rebuilding; do not change steps halfway through a session.
static const double SHIFT_STEP_PX = 0.25;
static const double SCALE_FACTOR_PER_PRESS = 1.001;
static const double MAX_ABS_SHIFT_PX = 20.0;
static const double MIN_SCALE = 0.90;
static const double MAX_SCALE = 1.10;
// The legacy loader rejects a row if either image coordinate is exactly zero.
// Preserve originally invalid rows. For a valid transformed row ONLY, replace
// a newly produced exact zero by this negligible, documented displacement.
static const float VALID_ZERO_GUARD_PX = 0.000001f;

struct Point {
    float x, y;
    Point(float a = 0, float b = 0) : x(a), y(b) {}
};

struct Settings {
    int x_steps, y_steps, zoom_steps;
    double pivot_x, pivot_y;
    bool preview;
    unsigned long revision;
    Settings(double x = 255.5, double y = 255.5)
        : x_steps(0), y_steps(0), zoom_steps(0), pivot_x(x), pivot_y(y),
          preview(true), revision(0) {}
    double tx() const { return x_steps * SHIFT_STEP_PX; }
    double ty() const { return y_steps * SHIFT_STEP_PX; }
    double scale() const { return std::pow(SCALE_FACTOR_PER_PRESS, zoom_steps); }
};

inline bool withinLimits(const Settings& s) {
    return std::isfinite(s.pivot_x) && std::isfinite(s.pivot_y) &&
        std::fabs(s.tx()) <= MAX_ABS_SHIFT_PX &&
        std::fabs(s.ty()) <= MAX_ABS_SHIFT_PX &&
        std::isfinite(s.scale()) && s.scale() >= MIN_SCALE && s.scale() <= MAX_SCALE;
}

// Applied afresh to the original points, never to a previously modified map.
inline Point transformPoint(Point p, const Settings& s, bool* zero_guard = 0) {
    if (zero_guard) *zero_guard = false;
    if (!s.preview) return p;
    const double scale = s.scale();
    float x = static_cast<float>(s.pivot_x + scale * (p.x - s.pivot_x) + s.tx());
    float y = static_cast<float>(s.pivot_y + scale * (p.y - s.pivot_y) + s.ty());
    if (!std::isfinite(x) || !std::isfinite(y))
        throw std::runtime_error("Transformed coordinate is not finite.");
    if (x == 0.0f) { x = VALID_ZERO_GUARD_PX; if (zero_guard) *zero_guard = true; }
    if (y == 0.0f) { y = VALID_ZERO_GUARD_PX; if (zero_guard) *zero_guard = true; }
    return Point(x, y);
}

// Deterministic one-event update. Unknown keys and rejected moves do nothing.
inline bool changeSettings(Settings& s, char key) {
    Settings next = s;
    if (key == 'B') next.preview = !next.preview;
    else if (key == 'R') {
        next.x_steps = next.y_steps = next.zoom_steps = 0;
        next.preview = true;
    } else {
        if (!next.preview) return false;
        switch (key) {
        case 'W': --next.y_steps; break;
        case 'S': ++next.y_steps; break;
        case 'A': --next.x_steps; break;
        case 'D': ++next.x_steps; break;
        case 'Z': ++next.zoom_steps; break;
        case 'X': --next.zoom_steps; break;
        default: return false;
        }
    }
    if (!withinLimits(next)) return false;
    next.revision = s.revision + 1;
    s = next;
    return true;
}

struct Row {
    std::string angle_x_token, angle_y_token, x_token, y_token;
    Point angle, position;
    bool valid;
};

inline float parseNumber(const std::string& token) {
    std::istringstream in(token);
    in.imbue(std::locale::classic());
    double value = 0;
    if (!(in >> value) || !std::isfinite(value))
        throw std::runtime_error("Non-numeric or non-finite map value: " + token);
    in >> std::ws;
    if (!in.eof()) throw std::runtime_error("Invalid numeric map token: " + token);
    float result = static_cast<float>(value);
    if (!std::isfinite(result)) throw std::runtime_error("Map value exceeds float range.");
    return result;
}

struct Document {
    std::string original_bytes;
    std::vector<Row> rows;
    std::vector<Point> valid_points;
    std::vector<Point> valid_angles;
};

inline Document parseMap(const std::string& bytes) {
    Document d;
    d.original_bytes = bytes;
    std::istringstream in(bytes);
    in.imbue(std::locale::classic());
    std::string first;
    while (in >> first) {
        Row r;
        r.angle_x_token = first;
        if (!(in >> r.angle_y_token >> r.x_token >> r.y_token))
            throw std::runtime_error("Incomplete map row: expected four numeric columns.");
        r.angle = Point(parseNumber(r.angle_x_token), parseNumber(r.angle_y_token));
        r.position = Point(parseNumber(r.x_token), parseNumber(r.y_token));
        r.valid = (r.position.x != 0.0f && r.position.y != 0.0f);
        d.rows.push_back(r);
        if (r.valid) {
            d.valid_points.push_back(r.position);
            d.valid_angles.push_back(r.angle);
        }
    }
    if (d.valid_points.empty()) throw std::runtime_error("Map has no valid calibration rows.");
    return d;
}

inline std::vector<Point> buildPoints(const Document& d, const Settings& s) {
    if (!withinLimits(s)) throw std::runtime_error("Transform exceeds configured limits.");
    std::vector<Point> result;
    result.reserve(d.valid_points.size());
    for (std::size_t i = 0; i < d.valid_points.size(); ++i)
        result.push_back(transformPoint(d.valid_points[i], s));
    return result;
}

struct ExportText {
    std::string map_text, delta_csv;
    std::size_t zero_guard_rows, outside_image_rows;
    double max_displacement_px;
    ExportText() : zero_guard_rows(0), outside_image_rows(0), max_displacement_px(0) {}
};

inline ExportText serializeMap(const Document& d, const Settings& s,
                               int width, int height) {
    if (!s.preview) throw std::runtime_error("Export blocked in ORIGINAL preview; press B first.");
    if (!withinLimits(s)) throw std::runtime_error("Transform exceeds configured limits.");
    ExportText result;
    std::ostringstream map, delta;
    map.imbue(std::locale::classic()); delta.imbue(std::locale::classic());
    map << std::setprecision(std::numeric_limits<float>::max_digits10);
    delta << std::setprecision(17);
    delta << "row,original_valid,angle_x,angle_y,original_x,original_y,modified_x,modified_y,dx,dy,zero_guard,outside_image\n";
    for (std::size_t i = 0; i < d.rows.size(); ++i) {
        const Row& r = d.rows[i];
        bool zero_guard = false;
        Point q = r.valid ? transformPoint(r.position, s, &zero_guard) : r.position;
        bool outside = r.valid && (q.x < 0 || q.y < 0 || q.x > width - 1 || q.y > height - 1);
        if (zero_guard) ++result.zero_guard_rows;
        if (outside) ++result.outside_image_rows;
        const double dx = double(q.x) - r.position.x, dy = double(q.y) - r.position.y;
        const double distance = std::sqrt(dx * dx + dy * dy);
        if (distance > result.max_displacement_px) result.max_displacement_px = distance;
        // Angle tokens and row order are never changed. Invalid zero rows remain invalid.
        map << r.angle_x_token << ' ' << r.angle_y_token << ' ';
        if (r.valid) map << q.x << ' ' << q.y;
        else map << r.x_token << ' ' << r.y_token;
        map << '\n';
        delta << i + 1 << ',' << (r.valid ? 1 : 0) << ','
              << r.angle_x_token << ',' << r.angle_y_token << ','
              << r.x_token << ',' << r.y_token << ',' << q.x << ',' << q.y << ','
              << dx << ',' << dy << ',' << (zero_guard ? 1 : 0) << ',' << (outside ? 1 : 0) << '\n';
    }
    result.map_text = map.str(); result.delta_csv = delta.str();
    return result;
}

inline std::string fnv1a64(const std::string& bytes) {
    std::uint64_t h = UINT64_C(14695981039346656037);
    for (std::size_t i = 0; i < bytes.size(); ++i) {
        h ^= static_cast<unsigned char>(bytes[i]);
        h *= UINT64_C(1099511628211);
    }
    std::ostringstream out;
    out << std::hex << std::setfill('0') << std::setw(16) << h;
    return out.str();
}

inline std::string report(const Document& d, const Settings& s, const ExportText& e,
                          const std::string& source_path, const std::string& time_label,
                          const std::string& target_mode, int width, int height) {
    std::ostringstream out;
    out.imbue(std::locale::classic()); out << std::setprecision(17);
    out << "FlyBAM temporary map diagnostic export\n"
        << "source_branch=HeadDirection\nsource_commit=" << SOURCE_COMMIT << '\n'
        << "export_time_local=" << time_label << '\n'
        << "source_map_absolute_path=" << source_path << '\n'
        << "source_bytes=" << d.original_bytes.size() << '\n'
        << "source_fnv1a64_noncryptographic=" << fnv1a64(d.original_bytes) << '\n'
        << "modified_fnv1a64_noncryptographic=" << fnv1a64(e.map_text) << '\n'
        << "original_copy=source_map.txt\nmodified_map=map_modified.txt\n"
        << "row_deltas=row_deltas.csv\n"
        << "source_total_rows=" << d.rows.size() << '\n'
        << "valid_rows=" << d.valid_points.size() << '\n'
        << "invalid_zero_rows_preserved=" << d.rows.size() - d.valid_points.size() << '\n'
        << "image_width=" << width << "\nimage_height=" << height << '\n'
        << "coordinate_system=arena_ROI_image_pixels_x_right_y_down\n"
        << "angle_units=unchanged_from_source_map\n"
        << "formula_x=pivot_x+scale*(original_x-pivot_x)+shift_x\n"
        << "formula_y=pivot_y+scale*(original_y-pivot_y)+shift_y\n"
        << "pivot_x=" << s.pivot_x << "\npivot_y=" << s.pivot_y << '\n'
        << "shift_x_px=" << s.tx() << "\nshift_y_px=" << s.ty() << '\n'
        << "scale=" << s.scale() << "\nscale_percent=" << 100 * s.scale() << '\n'
        << "x_steps=" << s.x_steps << "\ny_steps=" << s.y_steps
        << "\nzoom_steps=" << s.zoom_steps << '\n'
        << "shift_step_px=" << SHIFT_STEP_PX << '\n'
        << "scale_factor_per_press=" << SCALE_FACTOR_PER_PRESS << '\n'
        << "map_revision=" << s.revision << '\n'
        << "target_preference_at_export_request=" << target_mode << '\n'
        << "new_exact_zero_rows_guarded=" << e.zero_guard_rows << '\n'
        << "zero_guard_px=" << VALID_ZERO_GUARD_PX << '\n'
        << "valid_rows_outside_image_not_clipped=" << e.outside_image_rows << '\n'
        << "maximum_row_displacement_px=" << e.max_displacement_px << '\n'
        << "angles_unchanged=true\nrow_order_preserved=true\n"
        << "export_matches_preview_float_coordinates=true\n"
        << "original_map_overwritten=false\n"
        << "completion=Only exports containing COMPLETE.txt finished successfully.\n"
        << "validation=Operator-selected candidate; not an automatic accuracy assessment.\n"
        << "scope=Global translation and uniform scale only; no local warp or rotation.\n"
        << "loader_note=Originally invalid rows are unchanged. A newly exact zero in a valid\n"
        << "row is replaced by zero_guard_px in both preview and export.\n"
        << "deployment=Stop the diagnostic executable, remove/disable diagnostic code,\n"
        << "back up the actual map.txt, then copy map_modified.txt to that path.\n"
        << "Do not apply the same transform twice.\n";
    return out.str();
}

} // namespace flybam_mapdiag
