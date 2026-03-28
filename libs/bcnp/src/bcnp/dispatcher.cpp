/**
 * @file dispatcher.cpp
 * @brief Implementation of the BCNP packet dispatcher.
 * 
 * Routes parsed packets to registered message type handlers and manages
 * connection state tracking. Thread-safe for concurrent PushBytes() calls.
 */

#include "bcnp/dispatcher.h"

#include <utility>

namespace bcnp {

/**
 * @brief Construct a dispatcher with the given configuration.
 * 
 * Creates internal StreamParser with callbacks wired to HandlePacket/HandleError.
 * 
 * @param config Dispatcher configuration (buffer size, timeouts)
 */
PacketDispatcher::PacketDispatcher(DispatcherConfig config)
    : m_config(config),
      m_parser(
          [this](const PacketView& packet) { HandlePacket(packet); },
          [this](const StreamParser::ErrorInfo& error) { HandleError(error); },
          m_config.parserBufferSize) {
    m_pendingPackets.reserve(16);
    m_pendingErrors.reserve(16);
}

/**
 * @brief Push raw bytes for parsing and dispatch.
 * 
 * Thread-safe: acquires mutex before accessing parser.
 * Parsed packets are dispatched to registered handlers synchronously.
 * 
 * @param data Pointer to incoming byte data
 * @param length Number of bytes to process
 */
void PacketDispatcher::PushBytes(const uint8_t* data, std::size_t length) {
    std::vector<std::pair<PacketHandler, PendingPacket>> packetsToDispatch;
    std::vector<std::pair<ErrorHandler, StreamParser::ErrorInfo>> errorsToDispatch;

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_parser.Push(data, length);

        packetsToDispatch.reserve(m_pendingPackets.size());
        const auto now = Clock::now();
        for (auto& pending : m_pendingPackets) {
            m_lastRx = now;
            const auto handlerIndex = static_cast<uint16_t>(pending.header.messageType);
            if (handlerIndex < m_handlers.size() && m_handlers[handlerIndex]) {
                packetsToDispatch.emplace_back(m_handlers[handlerIndex], std::move(pending));
            }
        }
        m_pendingPackets.clear();

        errorsToDispatch.reserve(m_pendingErrors.size());
        for (const auto& pendingError : m_pendingErrors) {
            ++m_parseErrors;
            if (m_errorHandler) {
                errorsToDispatch.emplace_back(m_errorHandler, pendingError);
            }
        }
        m_pendingErrors.clear();
    }

    for (auto& dispatch : packetsToDispatch) {
        PacketView packetView;
        packetView.header = dispatch.second.header;
        packetView.payload = crab::Slice<const uint8_t>(
            dispatch.second.payload.data(),
            dispatch.second.payload.size());
        dispatch.first(packetView);
    }

    for (auto& dispatchError : errorsToDispatch) {
        dispatchError.first(dispatchError.second);
    }
}

/**
 * @brief Register a handler for a specific message type.
 * 
 * @param typeId Message type ID to handle
 * @param handler Callback to invoke when packets of this type arrive
 */
void PacketDispatcher::RegisterHandler(MessageTypeId typeId, PacketHandler handler) {
    std::lock_guard<std::mutex> lock(m_mutex);
    const auto handlerIndex = static_cast<uint16_t>(typeId);
    if (handlerIndex >= m_handlers.size()) {
        return;
    }
    m_handlers[handlerIndex] = std::move(handler);
}

/**
 * @brief Remove a previously registered handler.
 * @param typeId Message type ID to stop handling
 */
void PacketDispatcher::UnregisterHandler(MessageTypeId typeId) {
    std::lock_guard<std::mutex> lock(m_mutex);
    const auto handlerIndex = static_cast<uint16_t>(typeId);
    if (handlerIndex >= m_handlers.size()) {
        return;
    }
    m_handlers[handlerIndex] = {};
}

void PacketDispatcher::SetWireSizeLookup(StreamParser::WireSizeLookup lookup) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_parser.SetWireSizeLookup(std::move(lookup));
}

/**
 * @brief Set the error callback for parse failures.
 * @param handler Callback to invoke on stream parsing errors
 */
void PacketDispatcher::SetErrorHandler(ErrorHandler handler) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_errorHandler = std::move(handler);
}

/**
 * @brief Check if the connection is active.
 * 
 * Returns true if a packet was received within the configured timeout period.
 * 
 * @param now Current time point for comparison
 * @return true if recently received packets, false if timed out
 */
bool PacketDispatcher::IsConnected(Clock::time_point now) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_lastRx == Clock::time_point::min()) {
        return false;
    }
    return (now - m_lastRx) <= m_config.connectionTimeout;
}

/**
 * @brief Get the timestamp of the last received packet.
 * @return Time point of last successful packet reception
 */
PacketDispatcher::Clock::time_point PacketDispatcher::LastReceiveTime() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_lastRx;
}

/**
 * @brief Get the total number of parse errors encountered.
 * @return Cumulative count of parse errors since creation
 */
uint64_t PacketDispatcher::ParseErrorCount() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_parseErrors;
}

/**
 * @brief Internal handler for successfully parsed packets.
 * 
 * Updates last receive time and dispatches to registered handler if one exists.
 * Unknown message types are silently ignored (no handler registered).
 * 
 * @param packet The validated packet view
 */
void PacketDispatcher::HandlePacket(const PacketView& packet) {
    PendingPacket pending;
    pending.header = packet.header;
    pending.payload.assign(packet.payload.data(), packet.payload.data() + packet.payload.size());
    m_pendingPackets.push_back(std::move(pending));
}

/**
 * @brief Internal handler for parse errors.
 * 
 * Increments error counter and invokes user error callback if set.
 * 
 * @param error Error information from the stream parser
 */
void PacketDispatcher::HandleError(const StreamParser::ErrorInfo& error) {
    m_pendingErrors.push_back(error);
}

} // namespace bcnp

