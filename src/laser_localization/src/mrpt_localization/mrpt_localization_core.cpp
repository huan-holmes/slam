/***********************************************************************************
 * Revised BSD License *
 * Copyright (c) 2014, Markus Bader <markus.bader@tuwien.ac.at> *
 * All rights reserved. *
 *                                                                                 *
 * Redistribution and use in source and binary forms, with or without *
 * modification, are permitted provided that the following conditions are met: *
 *     * Redistributions of source code must retain the above copyright *
 *       notice, this list of conditions and the following disclaimer. *
 *     * Redistributions in binary form must reproduce the above copyright *
 *       notice, this list of conditions and the following disclaimer in the *
 *       documentation and/or other materials provided with the distribution. *
 *     * Neither the name of the Vienna University of Technology nor the *
 *       names of its contributors may be used to endorse or promote products *
 *       derived from this software without specific prior written permission. *
 *                                                                                 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND *
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 **
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE *
 * DISCLAIMED. IN NO EVENT SHALL Markus Bader BE LIABLE FOR ANY *
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES *
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 **
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND *
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 **
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **                       *
 ***********************************************************************************/
#include "mrpt_localization/mrpt_localization_core.h"

#include <mrpt/version.h>
#include <mrpt/maps/COccupancyGridMap2D.h>

using mrpt::maps::COccupancyGridMap2D;

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace std;

PFLocalizationCore::PFLocalizationCore() : state_(NA) {
  metric_map_ptr_.reset(new CMultiMetricMap());
}

void PFLocalizationCore::init() {
  mrpt::math::CMatrixDouble33 cov;
  cov(0, 0) = 1, cov(1, 1) = 1, cov(2, 2) = 2 * M_PI;
  initial_pose_ =
      mrpt::poses::CPosePDFGaussian(mrpt::poses::CPose2D(0, 0, 0), cov);
  initial_particle_count_ = 1000;

  motion_model_default_options_.modelSelection =
      CActionRobotMovement2D::mmGaussian;
  motion_model_default_options_.gaussianModel.minStdXY = 0.10;
  motion_model_default_options_.gaussianModel.minStdPHI = 2.0;
}

bool PFLocalizationCore::initializeFilter() {
#if MRPT_VERSION >= 0x199
  const auto [cov, mean_point] = initial_pose_.getCovarianceAndMean();
#else
  mrpt::math::CMatrixDouble33 cov;
  mrpt::poses::CPose2D mean_point;
  initial_pose_.getCovarianceAndMean(cov, mean_point);
#endif

  log_info("InitializeFilter: %4.3fm, %4.3fm, %4.3frad ", mean_point.x(),
           mean_point.y(), mean_point.phi());
  LOG(INFO) << "InitializeFilter:" << mean_point.x() << " m " << mean_point.y()
            << " m" << mean_point.phi() << " rad";
  double min_x = mean_point.x() - cov(0, 0);
  double max_x = mean_point.x() + cov(0, 0);
  double min_y = mean_point.y() - cov(1, 1);
  double max_y = mean_point.y() + cov(1, 1);
  double min_phi = mean_point.phi() - cov(2, 2);
  double max_phi = mean_point.phi() + cov(2, 2);
  log_info("min_x: %f, max_x: %f, min_y: %f, max_y: %f", min_x, max_x, min_y,
           max_y);

  LOG(INFO) << "min_x: " << min_x << " max_x:" << max_x << " min_y:" << min_y
            << " max_y:" << max_y;

#if MRPT_VERSION >= 0x199
  if (metric_map_ptr_->countMapsByClass<COccupancyGridMap2D>() &&
      !init_PDF_mode) {
    // 初始点周围没有freeCell会出错
    try {
      pdf_.resetUniformFreeSpace(
          metric_map_ptr_->mapByClass<COccupancyGridMap2D>().get(), 0.7f,
          initial_particle_count_, min_x, max_x, min_y, max_y, min_phi,
          max_phi);
    } catch (std::logic_error& e) {
      log_warn(
          "resetUniformFreeSpace failed, please change initial_pose or check "
          "the map!!");
      LOG(WARNING) << "resetUniformFreeSpace failed, please change "
                      "initial_pose or check the map!!";
      init_filter_failed_ = true;
      return false;
    }

    log_info("resetUniformFreeSpace ok!!");
    LOG(INFO) << ("resetUniformFreeSpace ok!!");

  } else /*if (metric_map_.countMapsByClass<CLandmarksMap>() || init_PDF_mode)*/
#else
  if (metric_map_.m_gridMaps.size() && !init_PDF_mode) {
    log_info("resetUniformFreeSpace: 0x%X, initial_particle_count: %d",
             metric_map_.m_gridMaps[0].get(), initial_particle_count_);
    pdf_.resetUniformFreeSpace(metric_map_.m_gridMaps[0].get(), 0.7f,
                               initial_particle_count_, min_x, max_x, min_y,
                               max_y, min_phi, max_phi);
    log_info("resetUniformFreeSpace: ok!");
  } else if (metric_map_.m_landmarksMap || init_PDF_mode)
#endif
  {
    pdf_.resetUniform(min_x, max_x, min_y, max_y, min_phi, max_phi,
                      initial_particle_count_);
  }
  state_ = RUN;
  log_info("initializeFilter ok!!");
  LOG(INFO) << ("initializeFilter ok!!");
  return true;
}

void PFLocalizationCore::updateFilter(const CActionCollection::Ptr& _action,
                                      const CSensoryFrame::Ptr& _sf) {
  if (!init_filter_failed_ && state_ == INIT) {
    if (!initializeFilter()) {
      return;
    }
  }
  tictac_.Tic();
  pf_.executeOn(pdf_, _action.get(), _sf.get(), &pf_stats_);
  time_last_update_ = _sf->getObservationByIndex(0)->timestamp;
  update_counter_++;
  // log_info("updateFilter!!!");
}

void PFLocalizationCore::observation(
    const CSensoryFrame::Ptr& _sf, const CObservationOdometry::Ptr& _odometry) {
  auto action = CActionCollection::Create();
  CActionRobotMovement2D odom_move;
  odom_move.timestamp = _sf->getObservationByIndex(0)->timestamp;
  if (_odometry) {
    if (fabs(odom_last_observation_.x() - 10000000.) < 1e-6) {
      log_info("reset odom_last_observation_");
      LOG(INFO) << ("reset odom_last_observation_");
      odom_last_observation_ = _odometry->odometry;
    }
    mrpt::poses::CPose2D incOdoPose =
        _odometry->odometry - odom_last_observation_;
    odom_last_observation_ = _odometry->odometry;
    odom_move.computeFromOdometry(incOdoPose, motion_model_options_);
    action->insert(odom_move);
    // log_info("odometry ok!");
    // LOG(INFO) << "incOdoPose: " << incOdoPose << std::endl;
    updateFilter(action, _sf);
  } else {
    if (use_motion_model_default_options_) {
      log_info("No odometry at update %4i -> using dummy", update_counter_);
      odom_move.computeFromOdometry(mrpt::poses::CPose2D(0, 0, 0),
                                    motion_model_default_options_);
      action->insert(odom_move);
      log_warn("no odometry!");
      updateFilter(action, _sf);
    } else {
      log_info("No odometry at update %4i -> skipping observation",
               update_counter_);
    }
  }
}
