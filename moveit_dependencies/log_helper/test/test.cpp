//
// Created by fan on 2021/8/9.
//

#include "log_helper/log_helper.h"
#include <gtest/gtest.h>

TEST(LogHelper, format)
{
  auto str = format("ABC_%c_%s_%06d", 'D', "test", 12345);
  ASSERT_EQ(str, "ABC_D_test_012345");
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}