#pragma once

#include <memory>
#include <string>
#include <spdlog/logger.h>

namespace ORB_SLAM3::logging {

// Initialize default logger for application with option of empty sink.
void InitializeAppLogger(const std::string& name, bool empty_sink = false);

// Create module logger cloned from default logger.
std::shared_ptr<spdlog::logger> CreateModuleLogger(const std::string& name);

} // namespace ORB_SLAM3::logging
