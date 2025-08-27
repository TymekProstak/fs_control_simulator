#include <gtest/gtest.h>
#include "../engine.h"

using namespace metzler_model;

class EngineTest : public ::testing::Test {
protected:
    engine_params params{2.0, 100.0, -50.0};
    engine_state initial_state{10.0};
    Engine engine{params, initial_state};
};

TEST_F(EngineTest, ConstructorInitializesStateAndParams) {
    EXPECT_DOUBLE_EQ(engine.get_torque(), 10.0);
    engine_params p = engine.get_params();
    EXPECT_DOUBLE_EQ(p.charactersistic_time, 2.0);
    EXPECT_DOUBLE_EQ(p.max_torque, 100.0);
    EXPECT_DOUBLE_EQ(p.min_torque, -50.0);
}

TEST_F(EngineTest, SetAndGetTorque) {
    engine.set_torque(42.0);
    EXPECT_DOUBLE_EQ(engine.get_torque(), 42.0);
}

TEST_F(EngineTest, SetAndGetState) {
    engine_state s{77.0};
    engine.set_state(s);
    EXPECT_DOUBLE_EQ(engine.get_state().torque, 77.0);
}

TEST_F(EngineTest, SetAndGetParams) {
    engine_params new_params{1.5, 80.0, -40.0};
    engine.set_params(new_params);
    engine_params p = engine.get_params();
    EXPECT_DOUBLE_EQ(p.charactersistic_time, 1.5);
    EXPECT_DOUBLE_EQ(p.max_torque, 80.0);
    EXPECT_DOUBLE_EQ(p.min_torque, -40.0);
}

TEST_F(EngineTest, ResetToDefault) {
    engine.set_torque(55.0);
    engine.reset();
    EXPECT_DOUBLE_EQ(engine.get_torque(), 0.0);
}

TEST_F(EngineTest, ResetToCustomState) {
    engine_state s{33.0};
    engine.reset(s);
    EXPECT_DOUBLE_EQ(engine.get_torque(), 33.0);
}