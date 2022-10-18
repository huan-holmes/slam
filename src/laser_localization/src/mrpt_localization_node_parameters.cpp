//*****************************************************************************
// Copyright (c) 2022 MOI SmartRobotTechnology Co.,Ltd.
//*****************************************************************************

#include "mrpt_localization_node.h"
#include "mrpt_localization_node_defaults.h"

PFLocalizationNode::Parameters::Parameters(PFLocalizationNode* p)
    : PFLocalization::Parameters(p), node("~") {
  node.param<double>("transform_tolerance", transform_tolerance, 0.1);
  LOG(INFO) << "transform_tolerance: " << transform_tolerance;
  node.param<double>("no_update_tolerance", no_update_tolerance, 1.0);
  LOG(INFO) << "no_update_tolerance: " << no_update_tolerance;
  // node.param<double>("no_inputs_tolerance", no_inputs_tolerance,
  // std::numeric_limits<double>::infinity());
  node.param<double>("no_inputs_tolerance", no_inputs_tolerance, 1.0);
  LOG(INFO) << "no_inputs_tolerance: "
            << no_inputs_tolerance;  // disabled by default
  node.param<double>("rate", rate, MRPT_LOCALIZATION_NODE_DEFAULT_RATE);
  LOG(INFO) << "rate: " << rate;
  node.getParam("gui_mrpt", gui_mrpt);
  LOG(INFO) << "gui_mrpt: " << gui_mrpt;
  node.param<int>("parameter_update_skip", parameter_update_skip,
                  MRPT_LOCALIZATION_NODE_DEFAULT_PARAMETER_UPDATE_SKIP);
  LOG(INFO) << "parameter_update_skip: " << parameter_update_skip;
  node.getParam("ini_file", ini_file);
  LOG(INFO) << "ini_file: " << ini_file.c_str();
  node.getParam("map_file", map_file);
  LOG(INFO) << "map_file: " << map_file.c_str();
  node.getParam("sensor_sources", sensor_sources);
  LOG(INFO) << "sensor_sources: " << sensor_sources.c_str();
  node.param<std::string>("tf_prefix", tf_prefix, "");
  LOG(INFO) << "tf_prefix: " << tf_prefix.c_str();
  node.param<std::string>("global_frame_id", global_frame_id, "map");
  LOG(INFO) << "global_frame_id: " << global_frame_id.c_str();
  node.param<std::string>("odom_frame_id", odom_frame_id, "odom");
  LOG(INFO) << "odom_frame_id: " << odom_frame_id.c_str();
  node.param<std::string>("base_frame_id", base_frame_id, "base_link");
  LOG(INFO) << "base_frame_id: " << base_frame_id.c_str();
  node.param<bool>("pose_broadcast", pose_broadcast, true);
  LOG(INFO) << "pose_broadcast: " << pose_broadcast;
  node.param<bool>("tf_broadcast", tf_broadcast, true);
  LOG(INFO) << "tf_broadcast: " << tf_broadcast;
  node.param<bool>("use_map_topic", use_map_topic, false);
  LOG(INFO) << "use_map_topic: " << use_map_topic;
  node.param<bool>("first_map_only", first_map_only, false);
  LOG(INFO) << "first_map_only: " << first_map_only;
  node.param<bool>("debug", debug, true);
  LOG(INFO) << "debug: " << debug;
  node.param<std::string>("pose_record_file", pose_record_file,
                          "/home/moi/.amcl_pose");
  LOG(INFO) << "pose_record_file: " << pose_record_file.c_str();
  node.param<double>("update_min_dist", update_min_dist, 0.05);
  LOG(INFO) << "update_min_dist: " << update_min_dist;
  node.param<double>("update_min_angle", update_min_angle, mrpt::DEG2RAD(2.));
  update_min_angle = mrpt::DEG2RAD(update_min_angle);
  LOG(INFO) << "update_min_angle: " << update_min_angle;
  node.param<int>("nomotion_update_skip", nomotion_update_skip, 10);
  LOG(INFO) << "nomotion_update_skip: " << nomotion_update_skip;
  node.param<int>("init_process_update_cnt", init_process_update_cnt, 100);
  LOG(INFO) << "init_process_update_cnt: " << init_process_update_cnt;
  log_path = node.param<std::string>("log_path", "/tmp/");
  LOG(INFO) << "log_path: " << log_path.c_str();

  reconfigure_cb_ = boost::bind(
      &PFLocalizationNode::Parameters::callbackParameters, this, _1, _2);
  reconfigure_server_.setCallback(reconfigure_cb_);
}

void PFLocalizationNode::Parameters::update(const unsigned long& loop_count) {
  if (loop_count % parameter_update_skip) return;
  node.getParam("debug", debug);
  if (loop_count == 0) LOG(INFO) << "debug: " << debug;
  {
    int v = particlecloud_update_skip;
    node.param<int>("particlecloud_update_skip", particlecloud_update_skip,
                    MRPT_LOCALIZATION_NODE_DEFAULT_PARTICLECLOUD_UPDATE_SKIP);
    if (v != particlecloud_update_skip)
      LOG(INFO) << "particlecloud_update_skip: " << particlecloud_update_skip;
  }
  // {
  //   int v = map_update_skip;
  //   node.param<int>("map_update_skip", map_update_skip,
  //   MRPT_LOCALIZATION_NODE_DEFAULT_MAP_UPDATE_SKIP); if (v !=
  //   map_update_skip) LOG(INFO)<<"map_update_skip: ", map_update_skip);
  // }
}

void PFLocalizationNode::Parameters::callbackParameters(
    mrpt_localization::MotionConfig& config, uint32_t level) {
  if (config.motion_noise_type == MOTION_MODEL_GAUSSIAN) {
    motion_model_options->modelSelection = CActionRobotMovement2D::mmGaussian;

    motion_model_options->gaussianModel.a1 = config.gaussian_alpha_1;
    motion_model_options->gaussianModel.a2 = config.gaussian_alpha_2;
    motion_model_options->gaussianModel.a3 = config.gaussian_alpha_3;
    motion_model_options->gaussianModel.a4 = config.gaussian_alpha_4;
    motion_model_options->gaussianModel.minStdXY = config.gaussian_alpha_xy;
    motion_model_options->gaussianModel.minStdPHI = config.gaussian_alpha_phi;
    LOG(INFO) << "gaussianModel.type: gaussian";
    LOG(INFO) << "gaussianModel.a1: " << motion_model_options->gaussianModel.a1;
    LOG(INFO) << "gaussianModel.a2: " << motion_model_options->gaussianModel.a2;
    LOG(INFO) << "gaussianModel.a3: " << motion_model_options->gaussianModel.a3;
    LOG(INFO) << "gaussianModel.a4: " << motion_model_options->gaussianModel.a4;
    LOG(INFO) << "gaussianModel.minStdXY: "
              << motion_model_options->gaussianModel.minStdXY;
    LOG(INFO) << "gaussianModel.minStdPHI: "
              << motion_model_options->gaussianModel.minStdPHI;
  } else {
    LOG(INFO) << "We support at the moment only gaussian motion models";
  }
  *use_motion_model_default_options = config.use_default_motion;
  LOG(INFO) << "use_motion_model_default_options: "
            << use_motion_model_default_options;
  motion_model_default_options->gaussianModel.minStdXY =
      config.default_noise_xy;
  LOG(INFO) << "default_noise_xy: "
            << motion_model_default_options->gaussianModel.minStdXY;
  motion_model_default_options->gaussianModel.minStdPHI =
      config.default_noise_phi;
  LOG(INFO) << "default_noise_phi: "
            << motion_model_default_options->gaussianModel.minStdPHI;
  update_while_stopped = config.update_while_stopped;
  LOG(INFO) << "update_while_stopped: " << update_while_stopped;
  update_sensor_pose = config.update_sensor_pose;
  LOG(INFO) << "update_sensor_pose: " << update_sensor_pose;
}
