#include "agnocast/node/agnocast_context.hpp"
#include <rcl/arguments.h>
#include <rcutils/logging.h>

namespace agnocast
{

Context g_context;
std::mutex g_context_mtx;

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

  // Apply --disable-stdout-logs if present within --ros-args scope.
  // There is no public rcl API to extract this flag from rcl_arguments_t, so we scan argv directly.
  // rcl_logging_configure() cannot be used as it would initialize spdlog and attempt to set up a
  // rosout publisher (which requires rcl_node_t), neither of which exist in agnocast.
  bool in_ros_args = false;
  for (const auto & arg : args) {
    if (arg == "--ros-args") {
      in_ros_args = true;
    } else if (arg == "--") {
      in_ros_args = false;
    } else if (in_ros_args && arg == "--disable-stdout-logs") {
      rcutils_logging_set_output_handler(noop_log_output_handler);
      break;
    }
  }

  initialized_ = true;
}

void init(int argc, char const * const * argv)
{
  std::lock_guard<std::mutex> lock(g_context_mtx);
  g_context.init(argc, argv);
}

}  // namespace agnocast
