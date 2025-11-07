#include "parameters.h"

bool is_first_frame = true;

double lidar_end_time = 0.0;
double first_lidar_time = 0.0;
double time_con = 0.0;

double last_timestamp_lidar = -1.0;
double last_timestamp_imu = -1.0;

int pcd_index = 0;

std::string lid_topic;
std::string imu_topic;

bool prop_at_freq_of_imu;
bool check_satu;
bool con_frame;
bool cut_frame;

bool use_imu_as_input;
bool space_down_sample;
bool publish_odometry_without_downsample;

int init_map_size;
int con_frame_num;

double match_s;
double satu_acc;
double satu_gyro;
double cut_frame_time_interval;

float plane_thr;

double filter_size_surf_min;
double filter_size_map_min;
double fov_deg;

double cube_len;
float DET_RANGE;

bool imu_en;
bool gravity_align;
bool non_station_start;

double imu_time_inte;

double laser_point_cov;
double acc_norm;

double vel_cov;
double acc_cov_input;
double gyr_cov_input;

double gyr_cov_output;
double acc_cov_output;
double b_gyr_cov;
double b_acc_cov;

double imu_meas_acc_cov;
double imu_meas_omg_cov;

int lidar_type;
int pcd_save_interval;

std::vector<double> gravity_init;
std::vector<double> gravity;

std::vector<double> extrinT;
std::vector<double> extrinR;

bool runtime_pos_log;
bool pcd_save_en;
bool path_en;
bool extrinsic_est_en = true;

bool scan_pub_en;
bool scan_body_pub_en;

shared_ptr<Preprocess> p_pre;
double time_lag_imu_to_lidar = 0.0;

void readParameters(std::shared_ptr<rclcpp::Node> node)
{
  p_pre.reset(new Preprocess());
  prop_at_freq_of_imu = node->declare_parameter<bool>("prop_at_freq_of_imu", true);
  use_imu_as_input = node->declare_parameter<bool>("use_imu_as_input", true);
  check_satu = node->declare_parameter<bool>("check_satu", true);
  init_map_size = node->declare_parameter<int>("init_map_size", 100);
  space_down_sample = node->declare_parameter<bool>("space_down_sample", true);
  satu_acc = node->declare_parameter<double>("mapping.satu_acc", 3.0);
  satu_gyro = node->declare_parameter<double>("mapping.satu_gyro", 35.0);
  acc_norm = node->declare_parameter<double>("mapping.acc_norm", 1.0);
  plane_thr = node->declare_parameter<float>("mapping.plane_thr", 0.05f);
  p_pre->point_filter_num = node->declare_parameter<int>("point_filter_num", 2);
  lid_topic = node->declare_parameter<std::string>("common.lid_topic", "livox.lidar");
  imu_topic = node->declare_parameter<std::string>("common.imu_topic", "livox.imu");
  con_frame = node->declare_parameter<bool>("common.con_frame", false);
  con_frame_num = node->declare_parameter<int>("common.con_frame_num", 1);
  cut_frame = node->declare_parameter<bool>("common.cut_frame", false);
  cut_frame_time_interval = node->declare_parameter<double>("common.cut_frame_time_interval", 0.1);
  time_lag_imu_to_lidar = node->declare_parameter<double>("common.time_lag_imu_to_lidar", 0.0);
  filter_size_surf_min = node->declare_parameter<double>("filter_size_surf", 0.5);
  filter_size_map_min = node->declare_parameter<double>("filter_size_map", 0.5);
  cube_len = node->declare_parameter<double>("cube_side_length", 200);
  DET_RANGE = node->declare_parameter<float>("mapping.det_range", 300.f);
  fov_deg = node->declare_parameter<double>("mapping.fov_degree", 180);
  imu_en = node->declare_parameter<bool>("mapping.imu_en", true);
  non_station_start = node->declare_parameter<bool>("mapping.start_in_aggressive_motion", false);
  extrinsic_est_en = node->declare_parameter<bool>("mapping.extrinsic_est_en", true);
  imu_time_inte = node->declare_parameter<double>("mapping.imu_time_inte", 0.005);
  laser_point_cov = node->declare_parameter<double>("mapping.lidar_meas_cov", 0.1);
  acc_cov_input = node->declare_parameter<double>("mapping.acc_cov_input", 0.1);
  vel_cov = node->declare_parameter<double>("mapping.vel_cov", 20);
  gyr_cov_input = node->declare_parameter<double>("mapping.gyr_cov_input", 0.1);
  gyr_cov_output = node->declare_parameter<double>("mapping.gyr_cov_output", 0.1);
  acc_cov_output = node->declare_parameter<double>("mapping.acc_cov_output", 0.1);
  b_gyr_cov = node->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
  b_acc_cov = node->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
  imu_meas_acc_cov = node->declare_parameter<double>("mapping.imu_meas_acc_cov", 0.1);
  imu_meas_omg_cov = node->declare_parameter<double>("mapping.imu_meas_omg_cov", 0.1);
  p_pre->blind = node->declare_parameter<double>("preprocess.blind", 1.0);
  lidar_type = node->declare_parameter<int>("preprocess.lidar_type", 1);
  p_pre->N_SCANS = node->declare_parameter<int>("preprocess.scan_line", 16);
  p_pre->SCAN_RATE = node->declare_parameter<int>("preprocess.scan_rate", 10);
  p_pre->time_unit = node->declare_parameter<int>("preprocess.timestamp_unit", 1);
  match_s = node->declare_parameter<double>("mapping.match_s", 81);
  gravity_align = node->declare_parameter<bool>("mapping.gravity_align", true);
  gravity = node->declare_parameter<std::vector<double>>("mapping.gravity", std::vector<double>());
  gravity_init = node->declare_parameter<std::vector<double>>("mapping.gravity_init", std::vector<double>());
  extrinT = node->declare_parameter<std::vector<double>>("mapping.extrinsic_T", std::vector<double>({0.007698, 0.014655, -0.00667}));
  extrinR = node->declare_parameter<std::vector<double>>("mapping.extrinsic_R", std::vector<double>({1, 0, 0, 0, 1, 0, 0, 0, 1}));
  publish_odometry_without_downsample = node->declare_parameter<bool>("odometry.publish_odometry_without_downsample", false);
  path_en = node->declare_parameter<bool>("publish.path_en", true);
  scan_pub_en = node->declare_parameter<bool>("publish.scan_publish_en", true);
  scan_body_pub_en = node->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
  runtime_pos_log = node->declare_parameter<bool>("runtime_pos_log_enable", false);
  pcd_save_en = node->declare_parameter<bool>("pcd_save.pcd_save_en", false);
  pcd_save_interval = node->declare_parameter<int>("pcd_save.interval", -1);
}