#include <gtest/gtest.h>


// Test template    

class SampleTest : public testing::Test {
  protected:
    virtual void SetUp() {
    }
    virtual void TearDown() {
    }
};
 
TEST_F(SampleTest, BasicTest00) {
  EXPECT_EQ(2, (1+1));
  EXPECT_EQ(5, (2+3));
}


