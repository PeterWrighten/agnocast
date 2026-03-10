#include "agnocast/node/agnocast_context.hpp"

#include <gtest/gtest.h>
#include <rcl/logging.h>
#include <rcutils/logging.h>

// =========================================
// agnocast::Context --disable-stdout-logs tests
// =========================================

class AgnocastContextStdoutLogsTest : public ::testing::Test
{
protected:
  void SetUp() override { original_handler_ = rcutils_logging_get_output_handler(); }

  void TearDown() override
  {
    // Finalize rcl logging so the next test can call rcl_logging_configure again.
    rcl_logging_fini();
    // Restore the original handler so other tests are not affected.
    rcutils_logging_set_output_handler(original_handler_);
  }

  rcutils_logging_output_handler_t original_handler_;
};

TEST_F(AgnocastContextStdoutLogsTest, disable_stdout_logs_preserves_file_handler)
{
  // Arrange
  const char * argv[] = {"program", "--ros-args", "--disable-stdout-logs"};
  int argc = 3;
  agnocast::Context ctx;

  // Act
  ctx.init(argc, argv);

  // Assert: rcl_logging_multiple_output_handler must be installed (file logging preserved).
  // rcl_logging_configure_with_output_handler reads --disable-stdout-logs from parsed arguments
  // and omits the console sub-handler internally, so stdout is suppressed without replacing the
  // top-level handler with a noop.
  const auto current_handler = rcutils_logging_get_output_handler();
  EXPECT_EQ(current_handler, rcl_logging_multiple_output_handler);
  // Also verify the handler changed from the pre-init default, confirming configure took effect.
  EXPECT_NE(current_handler, original_handler_);
}

TEST_F(AgnocastContextStdoutLogsTest, no_flag_keeps_handler_unchanged)
{
  // Arrange
  const char * argv[] = {"program", "--ros-args", "--log-level", "info"};
  int argc = 4;
  agnocast::Context ctx;

  // Act
  ctx.init(argc, argv);

  // Assert: rcl_logging_multiple_output_handler must be in effect (stdout and file both active).
  const auto current_handler = rcutils_logging_get_output_handler();
  EXPECT_EQ(current_handler, rcl_logging_multiple_output_handler);
  EXPECT_NE(current_handler, original_handler_);
}

TEST_F(AgnocastContextStdoutLogsTest, flag_outside_ros_args_is_ignored)
{
  // Arrange: --disable-stdout-logs appears before --ros-args, so it is not a ROS argument
  const char * argv[] = {"program", "--disable-stdout-logs", "--ros-args", "--log-level", "info"};
  int argc = 5;
  agnocast::Context ctx;

  // Act
  ctx.init(argc, argv);

  // Assert: flag outside --ros-args scope must be ignored; both stdout and file handlers active.
  const auto current_handler = rcutils_logging_get_output_handler();
  EXPECT_EQ(current_handler, rcl_logging_multiple_output_handler);
  EXPECT_NE(current_handler, original_handler_);
}

TEST_F(AgnocastContextStdoutLogsTest, flag_after_double_dash_terminator_is_ignored)
{
  // Arrange: --disable-stdout-logs appears after -- (end of ROS args), so it is not a ROS argument
  const char * argv[] = {"program", "--ros-args", "--", "--disable-stdout-logs"};
  int argc = 4;
  agnocast::Context ctx;

  // Act
  ctx.init(argc, argv);

  // Assert: flag after -- must be ignored; both stdout and file handlers active.
  const auto current_handler = rcutils_logging_get_output_handler();
  EXPECT_EQ(current_handler, rcl_logging_multiple_output_handler);
  EXPECT_NE(current_handler, original_handler_);
}
