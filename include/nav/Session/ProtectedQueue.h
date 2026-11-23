#pragma once

#include "nav/Logger/Log.h"
#include "nav/Geometry/NavigationTypes.h"

#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>
#include <stop_token>
#include <utility>

namespace nav::session
{
	/// Bounded GNSS packet handoff. Pausing preserves pending packets.
	class ProtectedQueue
	{
	public:
		/// Creates the bounded GNSS handoff with input and output initially paused.
		explicit ProtectedQueue(std::size_t capacity)
			: m_capacity(capacity == 0 ? 1 : capacity)
		{
			if (capacity == 0)
				nav::logger::warning("GNSS queue capacity was zero; using one packet");
		}

		/// Waits until acquisition can reserve space to publish the next packet.
		bool waitToProduce(std::stop_token stop)
		{
			std::unique_lock lock(m_mutex);
			m_changed.wait(lock, stop, [this] { return m_mode == Mode::Open || m_mode == Mode::Stopped; });
			if (stop.stop_requested() || m_mode == Mode::Stopped)
				return false;
			m_producing = true;
			return true;
		}

		/// Publishes the reserved packet and drops the oldest pending packet if the queue is full.
		void push(nav::simulation::GnssPacket&& item)
		{
			{
				std::lock_guard lock(m_mutex);
				if (!m_producing)
					return;
				m_producing = false;
				if (m_mode != Mode::Stopped)
				{
					if (m_items.size() == m_capacity)
					{
						m_items.pop_front();
						++m_dropped;
					}
					m_items.push_back(std::move(item));
				}
			}
			m_changed.notify_all();
		}

		/// Waits for an available packet and marks it as in flight for the consumer.
		std::optional<nav::simulation::GnssPacket> waitAndPop(std::stop_token stop)
		{
			std::unique_lock lock(m_mutex);
			m_changed.wait(
				lock, stop, [this] { return m_mode == Mode::Stopped || (m_mode != Mode::Paused && !m_items.empty()); });
			if (stop.stop_requested() || m_mode == Mode::Stopped)
				return std::nullopt;
			std::optional<nav::simulation::GnssPacket> item{std::move(m_items.front())};
			m_items.pop_front();
			m_consuming = true;
			return item;
		}

		/// Releases the in-flight packet and reopens acquisition after the backlog drains.
		void complete()
		{
			{
				std::lock_guard lock(m_mutex);
				m_consuming = false;
				if (m_mode == Mode::Draining && m_items.empty())
					m_mode = Mode::Open;
			}
			m_changed.notify_all();
		}

		/// Blocks new queue work and waits for in-flight work to finish.
		void pause()
		{
			std::unique_lock lock(m_mutex);
			if (m_mode == Mode::Stopped)
				return;
			m_mode = Mode::Paused;
			m_changed.notify_all();
			// In-flight work may finish. No further work starts while paused.
			m_changed.wait(lock, [this] { return !m_producing && !m_consuming; });
		}

		/// Allows consumption to resume and holds new input until any backlog drains.
		void resume()
		{
			{
				std::lock_guard lock(m_mutex);
				if (m_mode != Mode::Paused)
					return;
				m_mode = m_items.empty() ? Mode::Open : Mode::Draining;
			}
			m_changed.notify_all();
		}

		/// Closes the queue, discards pending packets, and wakes waiting threads.
		void stop()
		{
			{
				std::lock_guard lock(m_mutex);
				m_mode = Mode::Stopped;
				m_producing = false;
				m_items.clear();
			}
			m_changed.notify_all();
		}

		/// Returns the number of buffered packets for inspection and tests.
		std::size_t pending() const
		{
			std::lock_guard lock(m_mutex);
			return m_items.size();
		}

		/// Lets the session detect that a worker closed the queue.
		bool isStopped() const
		{
			std::lock_guard lock(m_mutex);
			return m_mode == Mode::Stopped;
		}

		/// Returns how many packets were evicted when the queue reached capacity.
		std::size_t dropped() const
		{
			std::lock_guard lock(m_mutex);
			return m_dropped;
		}

	private:
		enum class Mode
		{
			Paused,
			Open,
			Draining,
			Stopped
		};
		const std::size_t m_capacity;                       // Maximum number of pending packets before the oldest is dropped.
		mutable std::mutex m_mutex;                         // Protects pending packets, queue mode, and in-flight flags.
		std::condition_variable_any m_changed;              // Wakes producer and consumer when packets or queue state change.
		std::deque<nav::simulation::GnssPacket> m_items;    // FIFO of receiver packets awaiting tracking.
		Mode m_mode = Mode::Paused;                         // Controls whether the queue accepts or delivers packets.
		bool m_producing = false;     // Tracks a reserved producer operation so pause can wait for publication.
		bool m_consuming = false;     // Tracks a dequeued packet so pause can wait for matching to finish.
		std::size_t m_dropped = 0;    // Counts packets evicted when the queue fills.
	};
}    // namespace nav::session
