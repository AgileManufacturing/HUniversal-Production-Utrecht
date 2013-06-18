#include <gtest/gtest.h>
#include <gmock/gmock.h>

class MockObject{
public:
	MOCK_METHOD0(test, void());
};

TEST(SanityTest, True) {
  EXPECT_TRUE(true);
}

TEST(SanityTest, False) {
  EXPECT_FALSE(false);
}

TEST(SanityTest, Mock) {
	MockObject mock;
	EXPECT_CALL(mock, test()).Times(1);
	mock.test();
}
