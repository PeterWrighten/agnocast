#include "agnocast/node/agnocast_context.hpp"
#include <rcl/arguments.h>
#include <rcl/log_level.h>
#include <rcutils/logging.h>

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <sstream>

#include "agnocast/agnocast_tracepoint_wrapper.h"

namespace agnocast
{

Context g_context;
std::mutex g_context_mtx;

/// Generate a ROS2-style log directory path: ~/.ros/log/<timestamp-hostname-pid>/
static std::string generate_log_directory()
{
  std::string base_dir;
  const char * ros_log_dir = std::getenv("ROS_LOG_DIR");
  const char * ros_home = std::getenv("ROS_HOME");
  if (ros_log_dir && ros_log_dir[0] != '\0') {
    base_dir = ros_log_dir;
  } else if (ros_home && ros_home[0] != '\0') {
    base_dir = std::string(ros_home) + "/log";
  } else {
    const char * home = std::getenv("HOME");
    base_dir = std::string(home ? home : "/tmp") + "/.ros/log";
  }

  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  auto us =
    std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() % 1000000;
  std::tm tm_buf{};
  localtime_r(&time_t_now, &tm_buf);

  char hostname[256] = {};
  gethostname(hostname, sizeof(hostname));

  std::ostringstream oss;
  oss << base_dir << "/" << std::put_time(&tm_buf, "%Y-%m-%d-%H-%M-%S") << "-"
      << std::setfill('0') << std::setw(6) << us << "-" << hostname << "-" << getpid();
  return oss.str();
}

static void initialize_spdlog(const std::string & log_dir)
{
  std::filesystem::create_directories(log_dir);
  std::string log_file = log_dir + "/agnocast.log";
  spdlog::drop("agnocast");
  auto logger = spdlog::basic_logger_mt("agnocast", log_file);
  logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] %v");
  logger->flush_on(spdlog::level::warn);
  spdlog::set_default_logger(logger);
}

static void noop_log_output_handler(
  const rcutils_log_location_t *, int, const char *, rcutils_time_point_value_t, const char *,
  va_list *)
{
}

void Context::init(int argc, char const * const * argv)
{
  if (initialized_) {
    return;
  }

  // Copy argv into a safe container to avoid pointer arithmetic
  std::vector<std::string> args;
  args.reserve(static_cast<size_t>(argc));
  for (int i = 0; i < argc; ++i) {
    args.emplace_back(argv[i]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }

  parsed_arguments_ = parse_arguments(args);

  // Apply --log-level settings from parsed arguments
  rcl_log_levels_t log_levels = rcl_get_zero_initialized_log_levels();
  rcl_ret_t ret = rcl_arguments_get_log_levels(parsed_arguments_.get(), &log_levels);
  if (RCL_RET_OK == ret) {
    if (log_levels.default_logger_level != RCUTILS_LOG_SEVERITY_UNSET) {
      rcutils_logging_set_default_logger_level(static_cast<int>(log_levels.default_logger_level));
    }
    for (size_t i = 0; i < log_levels.num_logger_settings; ++i) {
      rcutils_logging_set_logger_level(
        log_levels.logger_settings[i].name, static_cast<int>(log_levels.logger_settings[i].level));
    }
    rcl_log_levels_fini(&log_levels);
  } else {
    rcl_reset_error();
  }

  // Scan argv for flags that have no public rcl getter API.
  // rcl_logging_configure() is not used because it would replace the rcutils output handler
  // and couple agnocast to rcl's logging lifecycle.
  bool in_ros_args = false;
  bool disable_external_lib_logs = false;
  for (const auto & arg : args) {
    if (arg == "--ros-args") {
      in_ros_args = true;
    } else if (arg == "--") {
      in_ros_args = false;
    } else if (in_ros_args) {
      if (arg == "--disable-stdout-logs") {
        rcutils_logging_set_output_handler(noop_log_output_handler);
      } else if (arg == "--enable-rosout-logs") {
        enable_rosout_logs_ = true;
      } else if (arg == "--disable-external-lib-logs") {
        disable_external_lib_logs = true;
      }
    }
  }

  // Initialize spdlog file logging unless explicitly disabled.
  if (!disable_external_lib_logs) {
    initialize_spdlog(generate_log_directory());
  }

  initialized_ = true;

  TRACEPOINT(agnocast_init, static_cast<const void *>(this));
}

void init(int argc, char const * const * argv)
{
  std::lock_guard<std::mutex> lock(g_context_mtx);
  g_context.init(argc, argv);
}

}  // namespace agnocast
