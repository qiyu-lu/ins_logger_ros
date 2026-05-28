#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

struct InsLoggerConfig {
    std::string port = "/dev/ttyUSB0";
    int baud = 460800;
    std::string packet = "opvt2a";
    double poll_hz = 100.0;

    std::string frame_id = "map";
    std::string child_frame_id = "ins_link";
    bool publish_tf = true;
    int path_max_size = 20000;

    bool record_enabled = true;
    std::string record_out_dir = "/home/sd101t/ins-d/data/ins_rviz_run";
    std::string record_file_name = "ins_raw.csv";
    std::string record_schema = "minimal";

    std::string convert_input_file;
    std::string convert_output_file;
    std::string convert_output_format = "tum";
    std::string convert_origin = "first_valid";
};

struct InsSample {
    double t = 0.0;
    std::size_t sample_index = 0;
    std::string packet;

    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;
    double east_speed = 0.0;
    double north_speed = 0.0;
    double vertical_speed = 0.0;

    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
    bool has_quaternion = false;

    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
    bool has_ypr = false;

    double gyro_x = 0.0;
    double gyro_y = 0.0;
    double gyro_z = 0.0;
    double acc_x = 0.0;
    double acc_y = 0.0;
    double acc_z = 0.0;
    double mag_x = 0.0;
    double mag_y = 0.0;
    double mag_z = 0.0;
    double temperature = 0.0;
    double vdd = 0.0;
    double p_bar = 0.0;
    double h_bar = 0.0;

    double gnss_lat = 0.0;
    double gnss_lon = 0.0;
    double gnss_alt = 0.0;
    double gnss_hor_speed = 0.0;
    double gnss_track_ground = 0.0;
    double gnss_ver_speed = 0.0;
    double latency_pos = 0.0;
    double latency_vel = 0.0;
    double latency_head = 0.0;
};

inline std::string trim(std::string s)
{
    auto not_space = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
    s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
    return s;
}

inline std::string toLower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

inline std::vector<std::string> split(const std::string& line, char delim)
{
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, delim)) {
        out.push_back(item);
    }
    return out;
}

inline double parseDouble(const std::string& value, double fallback = 0.0)
{
    char* end = nullptr;
    const double result = std::strtod(value.c_str(), &end);
    return end == value.c_str() ? fallback : result;
}

inline bool parseBool(std::string value, bool fallback)
{
    value = toLower(trim(value));
    if (value == "true" || value == "1" || value == "yes") {
        return true;
    }
    if (value == "false" || value == "0" || value == "no") {
        return false;
    }
    return fallback;
}

inline bool isValidLla(double lat, double lon)
{
    return std::isfinite(lat) && std::isfinite(lon) &&
           (std::abs(lat) > 1e-8 || std::abs(lon) > 1e-8);
}

inline void normalizeQuaternion(InsSample& sample)
{
    const double norm = std::sqrt(sample.qx * sample.qx + sample.qy * sample.qy +
                                  sample.qz * sample.qz + sample.qw * sample.qw);
    if (norm > 1e-12) {
        sample.qx /= norm;
        sample.qy /= norm;
        sample.qz /= norm;
        sample.qw /= norm;
        sample.has_quaternion = true;
    }
}

inline void yprDegToQuat(double yaw_deg, double pitch_deg, double roll_deg,
                         double& qx, double& qy, double& qz, double& qw)
{
    const double deg = M_PI / 180.0;
    const double yaw = yaw_deg * deg;
    const double pitch = pitch_deg * deg;
    const double roll = roll_deg * deg;

    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);

    qw = cr * cp * cy + sr * sp * sy;
    qx = sr * cp * cy - cr * sp * sy;
    qy = cr * sp * cy + sr * cp * sy;
    qz = cr * cp * sy - sr * sp * cy;
}

inline std::string recordPath(const InsLoggerConfig& cfg)
{
    return cfg.record_out_dir + "/" + cfg.record_file_name;
}

inline std::string defaultTumPath(const InsLoggerConfig& cfg)
{
    return cfg.record_out_dir + "/gt.tum";
}

inline void writeCsvHeader(std::ostream& out, const std::string& schema)
{
    out << "host_time_sec,sample_index,packet,latitude_deg,longitude_deg,altitude_m,"
        << "east_speed_mps,north_speed_mps,vertical_speed_mps,"
        << "qx,qy,qz,qw,yaw_deg,pitch_deg,roll_deg,"
        << "gnss_latitude_deg,gnss_longitude_deg,gnss_altitude_m";
    if (toLower(schema) == "full") {
        out << ",gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,mag_x,mag_y,mag_z,"
            << "temperature_c,vdd_v,p_bar,h_bar,gnss_hor_speed,gnss_track_ground,"
            << "gnss_ver_speed,latency_ms_pos,latency_ms_vel,latency_ms_head";
    }
    out << '\n';
}

inline void writeCsvSample(std::ostream& out, const InsSample& s, const std::string& schema)
{
    out << std::fixed << std::setprecision(9)
        << s.t << ','
        << s.sample_index << ','
        << s.packet << ','
        << std::setprecision(10)
        << s.lat << ','
        << s.lon << ','
        << std::setprecision(4)
        << s.alt << ','
        << s.east_speed << ','
        << s.north_speed << ','
        << s.vertical_speed << ','
        << std::setprecision(9)
        << s.qx << ','
        << s.qy << ','
        << s.qz << ','
        << s.qw << ','
        << std::setprecision(6)
        << s.yaw << ','
        << s.pitch << ','
        << s.roll << ','
        << std::setprecision(10)
        << s.gnss_lat << ','
        << s.gnss_lon << ','
        << std::setprecision(4)
        << s.gnss_alt;
    if (toLower(schema) == "full") {
        out << ',' << std::setprecision(6)
            << s.gyro_x << ',' << s.gyro_y << ',' << s.gyro_z << ','
            << s.acc_x << ',' << s.acc_y << ',' << s.acc_z << ','
            << s.mag_x << ',' << s.mag_y << ',' << s.mag_z << ','
            << s.temperature << ',' << s.vdd << ',' << s.p_bar << ',' << s.h_bar << ','
            << s.gnss_hor_speed << ',' << s.gnss_track_ground << ',' << s.gnss_ver_speed << ','
            << s.latency_pos << ',' << s.latency_vel << ',' << s.latency_head;
    }
    out << '\n';
}

inline std::map<std::string, std::string> readSimpleYaml(const std::string& path)
{
    std::ifstream in(path.c_str());
    if (!in.is_open()) {
        throw std::runtime_error("failed to open config: " + path);
    }

    std::map<std::string, std::string> values;
    std::string section;
    std::string line;
    while (std::getline(in, line)) {
        const std::size_t comment = line.find('#');
        if (comment != std::string::npos) {
            line = line.substr(0, comment);
        }
        if (trim(line).empty()) {
            continue;
        }
        if (!std::isspace(static_cast<unsigned char>(line[0]))) {
            const std::size_t colon = line.find(':');
            if (colon == std::string::npos) {
                continue;
            }
            section = trim(line.substr(0, colon));
            continue;
        }

        const std::size_t colon = line.find(':');
        if (colon == std::string::npos || section.empty()) {
            continue;
        }
        std::string key = trim(line.substr(0, colon));
        std::string value = trim(line.substr(colon + 1));
        if (value.size() >= 2 && value.front() == '"' && value.back() == '"') {
            value = value.substr(1, value.size() - 2);
        }
        values[section + "." + key] = value;
    }
    return values;
}

inline InsLoggerConfig configFromYaml(const std::string& path)
{
    InsLoggerConfig cfg;
    const auto values = readSimpleYaml(path);
    auto get = [&values](const std::string& key, const std::string& fallback) {
        const auto it = values.find(key);
        return it == values.end() ? fallback : it->second;
    };

    cfg.port = get("serial.port", cfg.port);
    cfg.baud = std::atoi(get("serial.baud", std::to_string(cfg.baud)).c_str());
    cfg.packet = toLower(get("serial.packet", cfg.packet));
    cfg.poll_hz = parseDouble(get("serial.poll_hz", std::to_string(cfg.poll_hz)), cfg.poll_hz);
    cfg.frame_id = get("ros.frame_id", cfg.frame_id);
    cfg.child_frame_id = get("ros.child_frame_id", cfg.child_frame_id);
    cfg.publish_tf = parseBool(get("ros.publish_tf", cfg.publish_tf ? "true" : "false"), cfg.publish_tf);
    cfg.path_max_size = std::atoi(get("ros.path_max_size", std::to_string(cfg.path_max_size)).c_str());
    cfg.record_enabled = parseBool(get("record.enabled", cfg.record_enabled ? "true" : "false"), cfg.record_enabled);
    cfg.record_out_dir = get("record.out_dir", cfg.record_out_dir);
    cfg.record_file_name = get("record.file_name", cfg.record_file_name);
    cfg.record_schema = toLower(get("record.schema", cfg.record_schema));
    cfg.convert_input_file = get("convert.input_file", cfg.convert_input_file);
    cfg.convert_output_file = get("convert.output_file", cfg.convert_output_file);
    cfg.convert_output_format = toLower(get("convert.output_format", cfg.convert_output_format));
    cfg.convert_origin = toLower(get("convert.origin", cfg.convert_origin));
    return cfg;
}
