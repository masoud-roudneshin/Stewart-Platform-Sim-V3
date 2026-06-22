#include <gtest/gtest.h>
#include "utils/RingBuffer.h"
#include "utils/LoopTimer.h"
#include "math/Math.h"

// ─── RingBuffer Tests ────────────────────────────────────────

class RingBufferTest : public ::testing::Test
{
protected:
    RingBuffer<int, 8> buffer;  // small buffer for easy testing
};

TEST_F(RingBufferTest, EmptyOnConstruction)
{
    EXPECT_TRUE(buffer.empty());
    EXPECT_EQ(buffer.size(), 0u);
}

TEST_F(RingBufferTest, PushMakesNotEmpty)
{
    buffer.push(42);
    EXPECT_FALSE(buffer.empty());
    EXPECT_EQ(buffer.size(), 1u);
}

TEST_F(RingBufferTest, PopReturnsPushedValue)
{
    buffer.push(42);
    int value = 0;
    bool success = buffer.pop(value);
    EXPECT_TRUE(success);
    EXPECT_EQ(value, 42);
}

TEST_F(RingBufferTest, PopOnEmptyReturnsFalse)
{
    int value = 0;
    bool success = buffer.pop(value);
    EXPECT_FALSE(success);
}

TEST_F(RingBufferTest, FIFOOrder)
{
    // First in first out
    buffer.push(1);
    buffer.push(2);
    buffer.push(3);

    int v1, v2, v3;
    buffer.pop(v1);
    buffer.pop(v2);
    buffer.pop(v3);

    EXPECT_EQ(v1, 1);
    EXPECT_EQ(v2, 2);
    EXPECT_EQ(v3, 3);
}

TEST_F(RingBufferTest, PushReturnsFalseWhenFull)
{
    // Buffer size is 8 — fill it up (capacity is N-1 = 7)
    for (int i = 0; i < 7; i++)
        buffer.push(i);

    // Next push should fail — buffer full
    bool success = buffer.push(99);
    EXPECT_FALSE(success);
}

TEST_F(RingBufferTest, PushOverwriteAlwaysSucceeds)
{
    // Fill buffer completely
    for (int i = 0; i < 10; i++)
        buffer.push_overwrite(i);

    // Buffer should not be empty
    EXPECT_FALSE(buffer.empty());
}

TEST_F(RingBufferTest, PushOverwriteKeepsLatestValues)
{
    // Overwrite old values — should keep last 7
    for (int i = 0; i < 10; i++)
        buffer.push_overwrite(i);

    // Pop all values — should be recent ones
    int value;
    int last = -1;
    while (buffer.pop(value))
        last = value;

    // Last value should be 9
    EXPECT_EQ(last, 9);
}

TEST_F(RingBufferTest, SizeTracksCorrectly)
{
    EXPECT_EQ(buffer.size(), 0u);
    buffer.push(1);
    EXPECT_EQ(buffer.size(), 1u);
    buffer.push(2);
    EXPECT_EQ(buffer.size(), 2u);

    int v;
    buffer.pop(v);
    EXPECT_EQ(buffer.size(), 1u);
    buffer.pop(v);
    EXPECT_EQ(buffer.size(), 0u);
}

TEST_F(RingBufferTest, WorksWithStruct)
{
    struct Point { int x, y; };
    RingBuffer<Point, 4> point_buffer;

    point_buffer.push({ 1, 2 });
    point_buffer.push({ 3, 4 });

    Point p;
    point_buffer.pop(p);
    EXPECT_EQ(p.x, 1);
    EXPECT_EQ(p.y, 2);
}

// ─── LoopTimer Tests ─────────────────────────────────────────

class LoopTimerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        timer = std::make_unique<LoopTimer>(
            std::chrono::milliseconds(10));  // 100Hz loop
    }

    std::unique_ptr<LoopTimer> timer;
};

TEST_F(LoopTimerTest, StartsWithZeroIterations)
{
    timer->start();
    EXPECT_EQ(timer->get_missed_deadlines(), 0);
}

TEST_F(LoopTimerTest, JitterIsNonNegative)
{
    timer->start();
    real_t jitter = timer->wait_until();
    EXPECT_GE(jitter, 0.0);
}

TEST_F(LoopTimerTest, DtIsPositiveAfterFirstIteration)
{
    timer->start();
    timer->wait_until();
    EXPECT_GT(timer->get_dt_seconds(), 0.0);
}

TEST_F(LoopTimerTest, RunsMultipleIterations)
{
    timer->start();
    for (int i = 0; i < 5; i++)
        timer->wait_until();

    // After 5 iterations at 100Hz — dt should be ~10ms
    EXPECT_NEAR(timer->get_dt_seconds(), 0.01, 0.05);  // wide tolerance for OS
}