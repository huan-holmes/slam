#pragma once

#include <iostream>
#include <ctime>
#include <thread>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"

class spd_logger {
 public:
  /**
   *
   * @param logger_name         logger名称，用于区分不同logger
   * @param filename            文件存储路径
   * @param logger_max_size     每个日志文件最大的文件体积，默认5M
   * @param logger_max_files    最多有多少个文件被循环写入，默认三个文件
   * @param interval            多长时间将缓存中的日志落盘，单位ms， 默认1000ms
   */
  spd_logger(const std::string& logger_name, const std::string& filename, size_t logger_max_size = 1048576 * 5,
             size_t logger_max_files = 3, size_t interval = 1000) {
    logger_ = spdlog::rotating_logger_mt(logger_name, filename, logger_max_size, logger_max_files);
    logger_->set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    logger_->set_level(spdlog::level::info);
    flush_interval_ = interval;
    logger_thread_ = std::thread(&spd_logger::flush_at_intervals, this);
  }

  ~spd_logger() {
    b_exit_ = true;
    if (logger_thread_.joinable()) {
      logger_thread_.join();
    }
  }

  std::shared_ptr<spdlog::logger>& get() { return logger_; }

 private:
  void flush_at_intervals() {
    while (!b_exit_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(flush_interval_));
      logger_->flush();
    }
  }

 private:
  std::shared_ptr<spdlog::logger> logger_{nullptr};
  std::thread logger_thread_{};
  size_t flush_interval_;
  bool b_exit_{false};
};
