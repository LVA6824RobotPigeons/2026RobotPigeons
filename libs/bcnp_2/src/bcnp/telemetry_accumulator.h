#pragma once

#include "bcnp/packet.h"
#include "bcnp/packet_storage.h"
#include "bcnp/static_vector.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <type_traits>
#include <vector>

namespace bcnp {

/**
 * @brief Configuration for telemetry accumulator.
 */
struct TelemetryAccumulatorConfig {
    /// Flush interval: send telemetry every N control loop ticks
    std::size_t flushIntervalTicks{2};
    
    /// Maximum messages to accumulate before forcing a flush
    std::size_t maxBufferedMessages{64};
};

namespace telemetry_detail {
template<typename T>
struct FixedStorageCapacity {
    static constexpr bool kIsFixed = false;
    static constexpr std::size_t kCapacity = 0;
};

template<typename T, std::size_t Capacity>
struct FixedStorageCapacity<StaticVector<T, Capacity>> {
    static constexpr bool kIsFixed = true;
    static constexpr std::size_t kCapacity = Capacity;
};
} // namespace telemetry_detail

/**
 * @brief Accumulates high-frequency telemetry data and batches into packets.
 * 
 * The accumulator collects sensor/state readings during the control loop and 
 * sends them as batched BCNP packets at a configurable rate. This avoids 
 * the overhead of a send() syscall per reading.
 * 
 * @tparam MsgType The message struct type (e.g., DrivetrainState, EncoderData)
 * @tparam Storage Container type (default: StaticVector<MsgType, 64>)
 * 
 * Usage (robot side):
 * @code{cpp}
 *   TelemetryAccumulator<DrivetrainState> drivetrainTelem;
 *   
 *   // In TeleopPeriodic (50Hz):
 *   drivetrainTelem.Record(DrivetrainState{
 *       .vxActual = drivetrain.GetVelocity(),
 *       .omegaActual = drivetrain.GetAngularVelocity(),
 *       .leftPos = drivetrain.GetLeftEncoder(),
 *       .rightPos = drivetrain.GetRightEncoder(),
 *       .timestampMs = static_cast<uint32_t>(Timer::GetFPGATimestamp() * 1000)
 *   });
 *   
 *   // At end of Periodic:
 *   drivetrainTelem.MaybeFlush(tcpAdapter);  // Sends every N ticks
 * @endcode
 */
template<typename MsgType, typename Storage = StaticVector<MsgType, 64>>
class TelemetryAccumulator {
public:
    using Clock = std::chrono::steady_clock;

    explicit TelemetryAccumulator(TelemetryAccumulatorConfig config = {})
        : m_config(config) {
        ClampConfigUnlocked();
        InitializeStorageUnlocked();
    }

    /**
     * @brief Record a telemetry reading.
     * 
     * Call this for each sensor/state update during the control loop.
     * If the buffer is full, the oldest reading is discarded (ring-buffer behavior).
     * 
     * @param msg The telemetry message to record
     * @return true if recorded successfully, false if buffer was full (still recorded, but overwrote)
     */
    bool Record(const MsgType& msg) {
        std::lock_guard<std::mutex> lock(m_mutex);

        if (m_ringCount >= m_config.maxBufferedMessages) {
            DropOldestUnlocked();
            ++m_metrics.bufferOverflows;
        }

        const std::size_t capacity = RingCapacityUnlocked();
        if (capacity == 0) {
            return false;
        }

        const std::size_t writeIndex = (m_ringHead + m_ringCount) % capacity;
        m_ringStorage[writeIndex] = msg;
        ++m_ringCount;
        ++m_metrics.messagesRecorded;
        return true;
    }

    /**
     * @brief Record multiple telemetry readings at once.
     * 
     * Useful for batching multiple encoder readings in a single call.
     */
    template<typename InputIt>
    void RecordBatch(InputIt first, InputIt last) {
        std::lock_guard<std::mutex> lock(m_mutex);
        for (auto it = first; it != last; ++it) {
            if (m_ringCount >= m_config.maxBufferedMessages) {
                DropOldestUnlocked();
                ++m_metrics.bufferOverflows;
            }

            const std::size_t capacity = RingCapacityUnlocked();
            if (capacity == 0) {
                break;
            }

            const std::size_t writeIndex = (m_ringHead + m_ringCount) % capacity;
            m_ringStorage[writeIndex] = *it;
            ++m_ringCount;
            ++m_metrics.messagesRecorded;
        }
    }

    /**
     * @brief Flush if interval has elapsed.
     * 
     * Call this at the end of each control loop iteration.
     * 
     * @param adapter The transport adapter to send through (must have SendBytes)
     * @return true if a packet was sent, false if interval not yet elapsed or buffer empty
     */
    template<typename Adapter>
    bool MaybeFlush(Adapter& adapter) {
        Storage localBuffer;
        std::size_t messageCount = 0;
        
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            
            ++m_tickCount;
            if (m_tickCount < m_config.flushIntervalTicks) {
                return false;
            }
            
            m_tickCount = 0;
            
            if (m_ringCount == 0) {
                return false;
            }

            messageCount = ExtractBufferedMessagesUnlocked(localBuffer);
        }
        // Lock released - now safe to do blocking I/O
        
        return SendBuffer(adapter, std::move(localBuffer), messageCount);
    }

    /**
     * @brief Force an immediate flush regardless of interval.
     * 
     * @param adapter The transport adapter to send through
     * @return true if a packet was sent, false if buffer was empty
     */
    template<typename Adapter>
    bool ForceFlush(Adapter& adapter) {
        Storage localBuffer;
        std::size_t messageCount = 0;
        
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_tickCount = 0;
            
            if (m_ringCount == 0) {
                return false;
            }

            messageCount = ExtractBufferedMessagesUnlocked(localBuffer);
        }
        // Lock released - now safe to do blocking I/O
        
        return SendBuffer(adapter, std::move(localBuffer), messageCount);
    }

    /**
     * @brief Get the number of buffered messages waiting to be sent.
     */
    std::size_t BufferedCount() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_ringCount;
    }

    /**
     * @brief Clear all buffered messages without sending.
     */
    void Clear() {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_ringHead = 0;
        m_ringCount = 0;
        m_tickCount = 0;
    }

    /**
     * @brief Metrics for diagnostics.
     */
    struct Metrics {
        uint64_t messagesRecorded{0};
        uint64_t messagesSent{0};
        uint64_t packetsSent{0};
        uint64_t bufferOverflows{0};
        uint64_t sendFailures{0};
    };

    Metrics GetMetrics() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_metrics;
    }

    void ResetMetrics() {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_metrics = {};
    }

    /**
     * @brief Update configuration.
     */
    void SetConfig(const TelemetryAccumulatorConfig& config) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_config = config;
        ClampConfigUnlocked();
        ResizeStorageUnlocked();
        while (m_ringCount > m_config.maxBufferedMessages) {
            DropOldestUnlocked();
            ++m_metrics.bufferOverflows;
        }
        m_tickCount = 0;
    }

private:
    /**
     * @brief Send a buffer of messages (called without holding m_mutex).
     * 
     * This helper is called after the buffer has been swapped out of the
     * accumulator, allowing blocking I/O without blocking Record() calls.
     */
    template<typename Adapter>
    bool SendBuffer(Adapter& adapter, Storage&& buffer, std::size_t messageCount) {
        // Build packet from extracted buffer
        TypedPacket<MsgType, Storage> packet;
        packet.messages = std::move(buffer);

        {
            // Serialize encode/send so concurrent flushes cannot race m_wireBuffer.
            std::lock_guard<std::mutex> sendLock(m_sendMutex);
            m_wireBuffer.clear();
            if (!EncodeTypedPacket(packet, m_wireBuffer)) {
                std::lock_guard<std::mutex> lock(m_mutex);
                ++m_metrics.sendFailures;
                return false;
            }

            // Send (potentially blocking syscall - but we don't hold m_mutex).
            if (!adapter.SendBytes(m_wireBuffer.data(), m_wireBuffer.size())) {
                std::lock_guard<std::mutex> lock(m_mutex);
                ++m_metrics.sendFailures;
                return false;
            }
        }

        // Update metrics
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_metrics.messagesSent += messageCount;
            ++m_metrics.packetsSent;
        }
        return true;
    }

    TelemetryAccumulatorConfig m_config;
    Storage m_ringStorage{};
    std::size_t m_ringHead{0};
    std::size_t m_ringCount{0};
    std::vector<uint8_t> m_wireBuffer;  // Persistent buffer, capacity retained across flushes
    std::size_t m_tickCount{0};
    Metrics m_metrics{};
    mutable std::mutex m_mutex;
    std::mutex m_sendMutex;

    void DropOldestUnlocked() {
        if (m_ringCount == 0) {
            return;
        }

        m_ringHead = (m_ringHead + 1) % RingCapacityUnlocked();
        --m_ringCount;
    }

    std::size_t RingCapacityUnlocked() const {
        if constexpr (telemetry_detail::FixedStorageCapacity<Storage>::kIsFixed) {
            return telemetry_detail::FixedStorageCapacity<Storage>::kCapacity;
        } else {
            return m_ringStorage.size();
        }
    }

    void InitializeStorageUnlocked() {
        ResizeStorageUnlocked();
        m_ringHead = 0;
        m_ringCount = 0;
    }

    void ResizeStorageUnlocked() {
        const std::size_t targetCapacity = m_config.maxBufferedMessages;
        if constexpr (telemetry_detail::FixedStorageCapacity<Storage>::kIsFixed) {
            constexpr std::size_t kCapacity =
                telemetry_detail::FixedStorageCapacity<Storage>::kCapacity;
            if (m_ringStorage.size() != kCapacity) {
                m_ringStorage.resize(kCapacity);
            }
            (void)targetCapacity;
            return;
        } else {
            const std::size_t oldCapacity = m_ringStorage.size();
            if (oldCapacity == targetCapacity) {
                return;
            }

            Storage rebuilt;
            rebuilt.resize(targetCapacity);

            if (oldCapacity == 0 || m_ringCount == 0) {
                m_ringStorage = std::move(rebuilt);
                m_ringHead = 0;
                m_ringCount = 0;
                return;
            }

            const std::size_t keptCount = std::min(m_ringCount, targetCapacity);
            const std::size_t droppedCount = m_ringCount - keptCount;
            const std::size_t start = (m_ringHead + droppedCount) % oldCapacity;

            for (std::size_t i = 0; i < keptCount; ++i) {
                const std::size_t src = (start + i) % oldCapacity;
                rebuilt[i] = std::move(m_ringStorage[src]);
            }

            m_ringStorage = std::move(rebuilt);
            m_ringHead = 0;
            m_ringCount = keptCount;
            m_metrics.bufferOverflows += droppedCount;
        }
    }

    std::size_t ExtractBufferedMessagesUnlocked(Storage& out) {
        out.clear();
        out.reserve(m_ringCount);

        const std::size_t capacity = RingCapacityUnlocked();
        const std::size_t messageCount = m_ringCount;
        for (std::size_t i = 0; i < messageCount; ++i) {
            const std::size_t src = (m_ringHead + i) % capacity;
            out.push_back(m_ringStorage[src]);
        }

        m_ringHead = 0;
        m_ringCount = 0;
        return messageCount;
    }

    void ClampConfigUnlocked() {
        if (m_config.flushIntervalTicks == 0) {
            m_config.flushIntervalTicks = 1;
        }
        if (m_config.maxBufferedMessages == 0) {
            m_config.maxBufferedMessages = 1;
        }

        if constexpr (telemetry_detail::FixedStorageCapacity<Storage>::kIsFixed) {
            constexpr std::size_t kCapacity =
                telemetry_detail::FixedStorageCapacity<Storage>::kCapacity;
            if (m_config.maxBufferedMessages > kCapacity) {
                m_config.maxBufferedMessages = kCapacity;
            }
        }
    }
};

/**
 * @brief Convenience alias for heap-allocated accumulator (large batches).
 */
template<typename MsgType>
using DynamicTelemetryAccumulator = TelemetryAccumulator<MsgType, std::vector<MsgType>>;

/**
 * @brief Convenience alias for stack-allocated accumulator (real-time, default).
 */
template<typename MsgType, std::size_t Capacity = 64>
using StaticTelemetryAccumulator = TelemetryAccumulator<MsgType, StaticVector<MsgType, Capacity>>;

} // namespace bcnp
