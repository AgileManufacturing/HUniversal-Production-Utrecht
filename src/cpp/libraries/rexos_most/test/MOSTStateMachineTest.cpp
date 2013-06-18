#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "rexos_most/MOSTStateMachine.h"



using namespace rexos_most;
using ::testing::Return;

class MockMOSTStateMachine: public MOSTStateMachine {
public:
	MockMOSTStateMachine() :
			MOSTStateMachine(123) {
	}

	MOCK_METHOD0(transitionStart, bool());
	MOCK_METHOD0(transitionStop, bool());
	MOCK_METHOD0(transitionSetup, bool());
	MOCK_METHOD0(transitionShutdown, bool());
};

TEST(MOSTStateMachineTest, Startup) {
	MockMOSTStateMachine mock;

	EXPECT_EQ(mock.getCurrentModi(), MODI_NORMAL);
	EXPECT_EQ(mock.getCurrentState(), STATE_SAFE);
}

TEST(MOSTStateMachineTest, Setup) {
	MockMOSTStateMachine mock;

	EXPECT_CALL(mock, transitionSetup()).Times(1).WillOnce(Return(true));
	EXPECT_CALL(mock, transitionStart()).Times(0);
	EXPECT_CALL(mock, transitionStop()).Times(0);
	EXPECT_CALL(mock, transitionShutdown()).Times(0);

	EXPECT_EQ(mock.getCurrentState(), STATE_SAFE);

	EXPECT_TRUE(mock.changeState(STATE_STANDBY));

	EXPECT_EQ(mock.getCurrentState(), STATE_STANDBY);
}

TEST(MOSTStateMachineTest, Start) {
	MockMOSTStateMachine mock;

	EXPECT_CALL(mock, transitionSetup()).Times(1).WillOnce(Return(true));
	EXPECT_CALL(mock, transitionStart()).Times(1).WillOnce(Return(true));
	EXPECT_CALL(mock, transitionStop()).Times(0);
	EXPECT_CALL(mock, transitionShutdown()).Times(0);

	EXPECT_EQ(mock.getCurrentState(), STATE_SAFE);

	EXPECT_TRUE(mock.changeState(STATE_STANDBY));

	EXPECT_EQ(mock.getCurrentState(), STATE_STANDBY);

	EXPECT_TRUE(mock.changeState(STATE_NORMAL));

	EXPECT_EQ(mock.getCurrentState(), STATE_NORMAL);
}

TEST(MOSTStateMachineTest, Stop) {
	MockMOSTStateMachine mock;

	EXPECT_CALL(mock, transitionSetup()).Times(1).WillOnce(Return(true));
	EXPECT_CALL(mock, transitionStart()).Times(1).WillOnce(Return(true));
	EXPECT_CALL(mock, transitionStop()).Times(1).WillOnce(Return(true));
	EXPECT_CALL(mock, transitionShutdown()).Times(0);

	EXPECT_EQ(mock.getCurrentState(), STATE_SAFE);

	EXPECT_TRUE(mock.changeState(STATE_STANDBY));
	EXPECT_EQ(mock.getCurrentState(), STATE_STANDBY);

	EXPECT_TRUE(mock.changeState(STATE_NORMAL));
	EXPECT_EQ(mock.getCurrentState(), STATE_NORMAL);

	EXPECT_TRUE(mock.changeState(STATE_STANDBY));
	EXPECT_EQ(mock.getCurrentState(), STATE_STANDBY);
}

TEST(MOSTStateMachineTest, Shutdown) {
	MockMOSTStateMachine mock;

	EXPECT_CALL(mock, transitionSetup()).Times(1).WillOnce(Return(true));
	EXPECT_CALL(mock, transitionStart()).Times(1).WillOnce(Return(true));
	EXPECT_CALL(mock, transitionStop()).Times(1).WillOnce(Return(true));
	EXPECT_CALL(mock, transitionShutdown()).Times(1).WillOnce(Return(true));

	EXPECT_EQ(mock.getCurrentState(), STATE_SAFE);

	EXPECT_TRUE(mock.changeState(STATE_STANDBY));
	EXPECT_EQ(mock.getCurrentState(), STATE_STANDBY);

	EXPECT_TRUE(mock.changeState(STATE_NORMAL));
	EXPECT_EQ(mock.getCurrentState(), STATE_NORMAL);

	EXPECT_TRUE(mock.changeState(STATE_STANDBY));
	EXPECT_EQ(mock.getCurrentState(), STATE_STANDBY);

	EXPECT_TRUE(mock.changeState(STATE_SAFE));
	EXPECT_EQ(mock.getCurrentState(), STATE_SAFE);
}
