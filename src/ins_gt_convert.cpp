#include "ins_gt_common.hpp"

#include <GeographicLib/LocalCartesian.hpp>

#include <iostream>

namespace {

std::map<std::string, std::string> parseArgs(int argc, char** argv)
{
    std::map<std::string, std::string> args;
    for (int i = 1; i < argc; ++i) {
        std::string key = argv[i];
        if (key == "--help" || key == "-h") {
            args[key] = "1";
            continue;
        }
        if (key.rfind("--", 0) != 0 || i + 1 >= argc) {
            throw std::runtime_error("usage: ins_gt_convert --config config.yaml [--input ins_raw.csv] [--output gt.tum]");
        }
        args[key] = argv[++i];
    }
    return args;
}

std::string argOr(const std::map<std::string, std::string>& args,
                  const std::string& key,
                  const std::string& fallback)
{
    const auto it = args.find(key);
    return it == args.end() ? fallback : it->second;
}

void printHelp()
{
    std::cout
        << "Usage: ins_gt_convert --config config/ins_gt_logger.yaml [options]\n"
        << "  --input ins_raw.csv   override convert.input_file\n"
        << "  --output gt.tum       override convert.output_file\n";
}

std::vector<InsSample> readCsvSamples(const std::string& path)
{
    std::ifstream in(path.c_str());
    if (!in.is_open()) {
        throw std::runtime_error("failed to open input CSV: " + path);
    }

    std::string header;
    if (!std::getline(in, header)) {
        throw std::runtime_error("empty input CSV: " + path);
    }
    const auto fields = split(header, ',');
    std::map<std::string, std::size_t> idx;
    for (std::size_t i = 0; i < fields.size(); ++i) {
        idx[trim(fields[i])] = i;
    }

    auto has = [&idx](const std::string& key) {
        return idx.find(key) != idx.end();
    };
    auto get = [&idx](const std::vector<std::string>& cols, const std::string& key, double fallback = 0.0) {
        const auto it = idx.find(key);
        if (it == idx.end() || it->second >= cols.size()) {
            return fallback;
        }
        return parseDouble(cols[it->second], fallback);
    };
    auto getString = [&idx](const std::vector<std::string>& cols, const std::string& key, const std::string& fallback) {
        const auto it = idx.find(key);
        if (it == idx.end() || it->second >= cols.size()) {
            return fallback;
        }
        return trim(cols[it->second]);
    };

    const std::vector<std::string> required = {
        "host_time_sec", "latitude_deg", "longitude_deg", "altitude_m"
    };
    for (const auto& key : required) {
        if (!has(key)) {
            throw std::runtime_error("missing CSV column: " + key);
        }
    }
    if (!(has("qx") && has("qy") && has("qz") && has("qw")) &&
        !(has("yaw_deg") && has("pitch_deg") && has("roll_deg"))) {
        throw std::runtime_error("CSV must contain qx/qy/qz/qw or yaw_deg/pitch_deg/roll_deg");
    }

    std::vector<InsSample> samples;
    std::string line;
    while (std::getline(in, line)) {
        if (trim(line).empty()) {
            continue;
        }
        const auto cols = split(line, ',');
        InsSample s;
        s.t = get(cols, "host_time_sec");
        s.sample_index = static_cast<std::size_t>(get(cols, "sample_index"));
        s.packet = getString(cols, "packet", "unknown");
        s.lat = get(cols, "latitude_deg");
        s.lon = get(cols, "longitude_deg");
        s.alt = get(cols, "altitude_m");
        s.east_speed = get(cols, "east_speed_mps");
        s.north_speed = get(cols, "north_speed_mps");
        s.vertical_speed = get(cols, "vertical_speed_mps");
        s.gnss_lat = get(cols, "gnss_latitude_deg");
        s.gnss_lon = get(cols, "gnss_longitude_deg");
        s.gnss_alt = get(cols, "gnss_altitude_m");

        if (has("qx") && has("qy") && has("qz") && has("qw")) {
            s.qx = get(cols, "qx");
            s.qy = get(cols, "qy");
            s.qz = get(cols, "qz");
            s.qw = get(cols, "qw", 1.0);
            normalizeQuaternion(s);
        } else {
            s.yaw = get(cols, "yaw_deg");
            s.pitch = get(cols, "pitch_deg");
            s.roll = get(cols, "roll_deg");
            s.has_ypr = true;
            yprDegToQuat(s.yaw, s.pitch, s.roll, s.qx, s.qy, s.qz, s.qw);
            normalizeQuaternion(s);
        }

        if (isValidLla(s.lat, s.lon) && s.has_quaternion) {
            samples.push_back(s);
        }
    }
    return samples;
}

void writeTum(const std::vector<InsSample>& samples, const std::string& output_path)
{
    if (samples.empty()) {
        throw std::runtime_error("no valid samples to convert");
    }

    GeographicLib::LocalCartesian enu;
    enu.Reset(samples.front().lat, samples.front().lon, samples.front().alt);

    std::ofstream out(output_path.c_str());
    if (!out.is_open()) {
        throw std::runtime_error("failed to open output: " + output_path);
    }

    out << std::fixed << std::setprecision(9);
    for (const auto& s : samples) {
        double east = 0.0;
        double north = 0.0;
        double up = 0.0;
        enu.Forward(s.lat, s.lon, s.alt, east, north, up);
        out << s.t << ' '
            << east << ' ' << north << ' ' << up << ' '
            << s.qx << ' ' << s.qy << ' ' << s.qz << ' ' << s.qw << '\n';
    }
}

} // namespace

int main(int argc, char** argv)
{
    try {
        const auto args = parseArgs(argc, argv);
        if (args.count("--help") || args.count("-h")) {
            printHelp();
            return 0;
        }

        const std::string config_path = argOr(args, "--config", "");
        if (config_path.empty()) {
            printHelp();
            return 1;
        }

        InsLoggerConfig cfg = configFromYaml(config_path);
        if (cfg.convert_output_format != "tum") {
            throw std::runtime_error("only convert.output_format=tum is supported");
        }
        if (cfg.convert_origin != "first_valid") {
            throw std::runtime_error("only convert.origin=first_valid is supported");
        }

        const std::string input = argOr(args, "--input",
                                        cfg.convert_input_file.empty() ? recordPath(cfg) : cfg.convert_input_file);
        const std::string output = argOr(args, "--output",
                                         cfg.convert_output_file.empty() ? defaultTumPath(cfg) : cfg.convert_output_file);

        const auto samples = readCsvSamples(input);
        writeTum(samples, output);
        std::cout << "Converted " << samples.size() << " samples to " << output << "\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "error: " << ex.what() << "\n";
        return 1;
    }
}
