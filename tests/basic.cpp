#include    <signals/handler.hpp>
#include    <csignal>

#include    <gtest/gtest.h>

#include    <thread>
#include    <chrono>
#include    <future>

struct Counter
{
    void onSignal (int);

    std::size_t count {0};
    std::promise<int> sig;
};

void Counter::onSignal (int num)
{
    ++count;
    sig.set_value (num);
}

TEST (basic, triggerAndCatchSignal)
{
    Counter counter;
    std::future<int> result = counter.sig.get_future ();

    signals::Handler handler {
        std::bind (&Counter::onSignal, std::ref (counter), std::placeholders::_1)};

    handler.addSignal (SIGINT);
    std::raise (SIGINT);

    ASSERT_EQ (
            std::future_status::ready,
            result.wait_for (std::chrono::milliseconds {200}));

    EXPECT_EQ (counter.count, 1);
    EXPECT_EQ (result.get (), SIGINT);
}

