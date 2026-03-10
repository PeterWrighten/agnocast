#include "agnocast/node/agnocast_context.hpp"

#include <rcutils/logging.h>

#include <gtest/gtest.h>

// =========================================
// agnocast::Context --enable-rosout-logs tests
// =========================================

class AgnocastContextRosoutTest : public ::testing::Test
{
protected:
  void SetUp() override { original_handler_ = rcutils_logging_get_output_handler(); }

  void TearDown() override { rcutils_logging_set_output_handler(original_handler_); }

  rcutils_logging_output_handler_t original_handler_;
};

TEST_F(AgnocastContextRosoutTest, enable_rosout_logs_sets_flag)
{
  const char * argv[] = {"program", "--ros-args", "--enable-rosout-logs"};
  int argc = 3;
  agnocast::Context ctx;

  ctx.init(argc, argv);

  EXPECT_TRUE(ctx.is_rosout_enabled());
}

TEST_F(AgnocastContextRosoutTest, no_flag_rosout_disabled)
{
  const char * argv[] = {"program", "--ros-args", "--log-level", "info"};
  int argc = 4;
  agnocast::Context ctx;

  ctx.init(argc, argv);

  EXPECT_FALSE(ctx.is_rosout_enabled());
}

TEST_F(AgnocastContextRosoutTest, flag_outside_ros_args_is_ignored)
{
  const char * argv[] = {"program", "--enable-rosout-logs", "--ros-args", "--log-level", "info"};
  int argc = 5;
  agnocast::Context ctx;

  ctx.init(argc, argv);

  EXPECT_FALSE(ctx.is_rosout_enabled());
}

TEST_F(AgnocastContextRosoutTest, flag_after_double_dash_is_ignored)
{
  const char * argv[] = {"program", "--ros-args", "--", "--enable-rosout-logs"};
  int argc = 4;
  agnocast::Context ctx;

  ctx.init(argc, argv);

  EXPECT_FALSE(ctx.is_rosout_enabled());
}
