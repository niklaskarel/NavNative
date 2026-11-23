#include "nav/Logger/PositionLogger.h"

#include "nav/Logger/Log.h"

#include <gtest/gtest.h>

#include <sstream>
#include <vector>

using namespace std::chrono_literals;

// Verifies the position logger rate limit and reset behavior.
TEST(PositionLoggerTest, LogsFirstPositionAndThrottlesUntilThreeSeconds)
{
	std::ostringstream output;
	std::vector<std::string> displayed;
	{
		nav::logger::Log log(output);
		log.setDisplayHandler([&displayed](const nav::logger::Log::DisplayEntry& entry) {
			displayed.push_back(entry.lightHtml);
		});
		nav::logger::PositionLogger logger(log);
		const auto start = std::chrono::steady_clock::time_point{};

		logger.log({1, 2}, start);
		logger.log({3, 4}, start + 2999ms);
		logger.log({5, 6}, start + 3s);
		logger.reset();
		logger.log({7, 8}, start + 3001ms);
	}

	EXPECT_EQ(output.str(), "[INFO] True position: (1, 2) m\n[INFO] True position: (3, 4) m\n"
						   "[INFO] True position: (5, 6) m\n[INFO] True position: (7, 8) m\n");
	ASSERT_EQ(displayed.size(), 3);
	EXPECT_NE(displayed[0].find("True position: (1, 2) m"), std::string::npos);
	EXPECT_NE(displayed[1].find("True position: (5, 6) m"), std::string::npos);
	EXPECT_NE(displayed[2].find("True position: (7, 8) m"), std::string::npos);
}

// Verifies that logger shutdown writes queued entries with their severity.
TEST(LogTest, DrainsQueuedMessagesWithTheirSeverityOnShutdown)
{
	std::ostringstream output;
	{
		nav::logger::Log log(output);
		log.enqueue(nav::logger::Level::Warning, "Invalid fix");
		log.enqueue(nav::logger::Level::Error, "Worker stopped");
	}
	EXPECT_EQ(output.str(), "[WARN] Invalid fix\n[ERROR] Worker stopped\n");
}
