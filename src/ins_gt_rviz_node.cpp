#include "ins_gt_common.hpp"

#include <GeographicLib/LocalCartesian.hpp>

#include <cerrno>
#include <csignal>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

extern "C" {
#include "InertialLabs_INS.h"
}

namespace {

volatile std::sig_atomic_t g_running = 1;

void handleSignal(int)
{
    g_running = 0;
}

void checkSerialAccess(const std::string& port)
{
    struct stat st;
    if (::stat(port.c_str(), &st) != 0) {
        throw std::runtime_error("serial port does not exist: " + port + " (" + std::strerror(errno) + ")");
    }
    if (!S_ISCHR(st.st_mode)) {
        throw std::runtime_error("serial path is not a character device: " + port);
    }
    if (::access(port.c_str(), R_OK | W_OK) != 0) {
        throw std::runtime_error("no read/write permission for " + port +
                                 " (" + std::strerror(errno) +
                                 "); run 'newgrp dialout' or log out/in after adding the user to dialout");
    }
}

void ensureDir(const std::string& dir)
{
    if (dir.empty()) {
        return;
    }
    if (::mkdir(dir.c_str(), 0755) != 0 && errno != EEXIST) {
        throw std::runtime_error("failed to create output directory: " + dir);
    }
}

template <typename T>
void paramOr(const ros::NodeHandle& nh, const std::string& name, T& value)
{
    nh.param<T>(name, value, value);
}

InsLoggerConfig readRosConfig(const ros::NodeHandle& nh)
{
    InsLoggerConfig cfg;
    paramOr(nh, "serial/port", cfg.port);
    paramOr(nh, "serial/baud", cfg.baud);
    paramOr(nh, "serial/packet", cfg.packet);
    paramOr(nh, "serial/poll_hz", cfg.poll_hz);
    paramOr(nh, "ros/frame_id", cfg.frame_id);
    paramOr(nh, "ros/child_frame_id", cfg.child_frame_id);
    paramOr(nh, "ros/publish_tf", cfg.publish_tf);
    paramOr(nh, "ros/path_max_size", cfg.path_max_size);
    paramOr(nh, "record/enabled", cfg.record_enabled);
    paramOr(nh, "record/out_dir", cfg.record_out_dir);
    paramOr(nh, "record/file_name", cfg.record_file_name);
    paramOr(nh, "record/schema", cfg.record_schema);
    cfg.packet = toLower(cfg.packet);
    cfg.record_schema = toLower(cfg.record_schema);
    return cfg;
}

IL_ERROR_CODE requestPacket(IL_INS* ins, const std::string& packet)
{
    if (packet == "opvt2a") {
        ins->cmd_flag = IL_OPVT2A_RECEIVE;
        return INS_OPVT2Adata_Receive(ins);
    }
    if (packet == "qpvt") {
        ins->cmd_flag = IL_QPVT_RECEIVE;
        return INS_QPVTdata_Receive(ins);
    }
    throw std::runtime_error("unsupported packet: " + packet + " (expected opvt2a or qpvt)");
}

bool readInsSample(IL_INS* ins, const InsLoggerConfig& cfg, std::size_t index, double stamp, InsSample& sample)
{
    INSCompositeData sensor;
    INSCompositeData pressure;
    INSPositionData pos;
    std::memset(&sensor, 0, sizeof(sensor));
    std::memset(&pressure, 0, sizeof(pressure));
    std::memset(&pos, 0, sizeof(pos));

    const IL_ERROR_CODE pos_err = INS_getPositionData(ins, &pos);
    if (pos_err != ILERR_NO_ERROR || !isValidLla(pos.Latitude, pos.Longitude)) {
        return false;
    }

    sample.t = stamp;
    sample.sample_index = index;
    sample.packet = cfg.packet;
    sample.lat = pos.Latitude;
    sample.lon = pos.Longitude;
    sample.alt = pos.Altitude;
    sample.east_speed = pos.East_Speed;
    sample.north_speed = pos.North_Speed;
    sample.vertical_speed = pos.Vertical_Speed;
    sample.gnss_lat = pos.GNSS_Latitude;
    sample.gnss_lon = pos.GNSS_Longitude;
    sample.gnss_alt = pos.GNSS_Altitude;
    sample.gnss_hor_speed = pos.GNSS_Horizontal_Speed;
    sample.gnss_track_ground = pos.GNSS_Trackover_Ground;
    sample.gnss_ver_speed = pos.GNSS_Vertical_Speed;

    if (INS_getLatencyData(ins, &pos) == ILERR_NO_ERROR) {
        sample.latency_pos = pos.Latency_ms_pos;
        sample.latency_vel = pos.Latency_ms_vel;
        sample.latency_head = pos.Latency_ms_head;
    }

    if (cfg.packet == "qpvt") {
        if (INS_getQuaternionData(ins, &sensor) != ILERR_NO_ERROR) {
            return false;
        }
        sample.qw = sensor.quaternion.Lk0;
        sample.qx = sensor.quaternion.Lk1;
        sample.qy = sensor.quaternion.Lk2;
        sample.qz = sensor.quaternion.Lk3;
        normalizeQuaternion(sample);
    } else {
        if (INS_YPR(ins, &sensor) != ILERR_NO_ERROR) {
            return false;
        }
        sample.yaw = sensor.ypr.yaw;
        sample.pitch = sensor.ypr.pitch;
        sample.roll = sensor.ypr.roll;
        sample.has_ypr = true;
        yprDegToQuat(sample.yaw, sample.pitch, sample.roll, sample.qx, sample.qy, sample.qz, sample.qw);
        normalizeQuaternion(sample);
    }

    if (INS_getGyroAccMag(ins, &sensor) == ILERR_NO_ERROR) {
        sample.gyro_x = sensor.gyro.c0;
        sample.gyro_y = sensor.gyro.c1;
        sample.gyro_z = sensor.gyro.c2;
        sample.acc_x = sensor.acceleration.c0;
        sample.acc_y = sensor.acceleration.c1;
        sample.acc_z = sensor.acceleration.c2;
        sample.mag_x = sensor.magnetic.c0;
        sample.mag_y = sensor.magnetic.c1;
        sample.mag_z = sensor.magnetic.c2;
        sample.temperature = sensor.Temper;
        sample.vdd = sensor.Vinp;
    }

    if (INS_getPressureBarometricData(ins, &pressure) == ILERR_NO_ERROR) {
        sample.p_bar = pressure.P_bar;
        sample.h_bar = pressure.H_bar;
    }

    return sample.has_quaternion;
}

} // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ins_gt_rviz_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    try {
        const InsLoggerConfig cfg = readRosConfig(nh);
        if (cfg.poll_hz <= 0.0) {
            throw std::runtime_error("serial.poll_hz must be positive");
        }
        if (cfg.record_schema != "minimal" && cfg.record_schema != "full") {
            throw std::runtime_error("record.schema must be minimal or full");
        }

        checkSerialAccess(cfg.port);
        if (cfg.record_enabled) {
            ensureDir(cfg.record_out_dir);
        }

        std::ofstream csv;
        if (cfg.record_enabled) {
            csv.open(recordPath(cfg).c_str(), std::ios::out);
            if (!csv.is_open()) {
                throw std::runtime_error("failed to open " + recordPath(cfg));
            }
            writeCsvHeader(csv, cfg.record_schema);
        }

        ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/ins/odom", 20);
        ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/ins/path", 5, true);
        tf2_ros::TransformBroadcaster tf_broadcaster;
        nav_msgs::Path path;
        path.header.frame_id = cfg.frame_id;

        IL_INS ins;
        std::memset(&ins, 0, sizeof(ins));
        IL_ERROR_CODE error = INS_connect(&ins, cfg.port.c_str(), cfg.baud);
        if (error != ILERR_NO_ERROR) {
            ROS_ERROR_STREAM("INS_connect failed with error " << error);
            return 2;
        }

        ins.cmd_flag = IL_STOP_CMD;
        INS_Stop(&ins);
        sleep(1);

        error = INS_SetMode(&ins, IL_SET_CONTINUES_MODE);
        if (error != ILERR_NO_ERROR) {
            ROS_ERROR_STREAM("INS_SetMode failed with error " << error);
            INS_disconnect(&ins);
            return 3;
        }

        error = requestPacket(&ins, cfg.packet);
        if (error != ILERR_NO_ERROR) {
            ROS_ERROR_STREAM("request packet failed with error " << error);
            INS_disconnect(&ins);
            return 4;
        }

        ROS_INFO_STREAM("Requested " << cfg.packet << " output. Waiting for valid INS position/orientation...");
        sleep(2);

        GeographicLib::LocalCartesian enu;
        bool has_origin = false;
        bool has_first_sample = false;
        double last_wait_log = ros::Time::now().toSec();
        std::size_t sample_index = 0;
        ros::Rate rate(cfg.poll_hz);

        while (ros::ok() && g_running) {
            const ros::Time stamp = ros::Time::now();
            InsSample sample;
            if (readInsSample(&ins, cfg, sample_index, stamp.toSec(), sample)) {
                if (!has_origin) {
                    enu.Reset(sample.lat, sample.lon, sample.alt);
                    has_origin = true;
                    ROS_INFO_STREAM("ENU origin set to lat=" << std::fixed << std::setprecision(10)
                                    << sample.lat << " lon=" << sample.lon << " alt=" << sample.alt);
                }

                double east = 0.0;
                double north = 0.0;
                double up = 0.0;
                enu.Forward(sample.lat, sample.lon, sample.alt, east, north, up);

                nav_msgs::Odometry odom;
                odom.header.stamp = stamp;
                odom.header.frame_id = cfg.frame_id;
                odom.child_frame_id = cfg.child_frame_id;
                odom.pose.pose.position.x = east;
                odom.pose.pose.position.y = north;
                odom.pose.pose.position.z = up;
                odom.pose.pose.orientation.x = sample.qx;
                odom.pose.pose.orientation.y = sample.qy;
                odom.pose.pose.orientation.z = sample.qz;
                odom.pose.pose.orientation.w = sample.qw;
                odom.twist.twist.linear.x = sample.east_speed;
                odom.twist.twist.linear.y = sample.north_speed;
                odom.twist.twist.linear.z = sample.vertical_speed;
                odom_pub.publish(odom);

                geometry_msgs::PoseStamped pose;
                pose.header = odom.header;
                pose.pose = odom.pose.pose;
                path.header.stamp = stamp;
                path.poses.push_back(pose);
                if (cfg.path_max_size > 0 && static_cast<int>(path.poses.size()) > cfg.path_max_size) {
                    path.poses.erase(path.poses.begin());
                }
                path_pub.publish(path);

                if (cfg.publish_tf) {
                    geometry_msgs::TransformStamped tf;
                    tf.header.stamp = stamp;
                    tf.header.frame_id = cfg.frame_id;
                    tf.child_frame_id = cfg.child_frame_id;
                    tf.transform.translation.x = east;
                    tf.transform.translation.y = north;
                    tf.transform.translation.z = up;
                    tf.transform.rotation = odom.pose.pose.orientation;
                    tf_broadcaster.sendTransform(tf);
                }

                if (cfg.record_enabled) {
                    writeCsvSample(csv, sample, cfg.record_schema);
                    csv.flush();
                }

                if (!has_first_sample) {
                    has_first_sample = true;
                    ROS_INFO_STREAM("First valid INS sample received. Publishing /ins/odom and /ins/path.");
                }
                ++sample_index;
            } else if (!has_first_sample && ros::Time::now().toSec() - last_wait_log >= 2.0) {
                last_wait_log = ros::Time::now().toSec();
                ROS_INFO("Waiting for valid INS position/orientation...");
            }

            ros::spinOnce();
            rate.sleep();
        }

        INS_disconnect(&ins);
        ROS_INFO_STREAM("Stopped INS RViz publisher after " << sample_index << " samples.");
        return 0;
    } catch (const std::exception& ex) {
        ROS_ERROR_STREAM("ins_gt_rviz_node error: " << ex.what());
        return 1;
    }
}
