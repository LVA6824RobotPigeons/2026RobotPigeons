/**
 * @file bcnp_jni.cpp
 * @brief JNI implementation for BCNP Java bindings.
 * 
 * Bridges the Java BcnpJNI class to the C++ bcnp_core library.
 * Uses DirectByteBuffers for zero-copy data sharing.
 */

#include <jni.h>
#include <cstdint>
#include <cstring>
#include <deque>

#include "bcnp/packet.h"

namespace {

/**
 * Map C++ PacketError ordinals to Java BcnpResult error constants.
 * Java constants: OK=0, INCOMPLETE=1, UNSUPPORTED_VERSION=2,
 * CHECKSUM_MISMATCH=3, UNKNOWN_MESSAGE_TYPE=4,
 * PAYLOAD_TOO_LARGE=5, BUFFER_TOO_SMALL=6.
 */
int MapPacketErrorToJava(bcnp::PacketError error) {
    switch (error) {
        case bcnp::PacketError::None:               return 0; // OK
        case bcnp::PacketError::TooSmall:            return 6; // BUFFER_TOO_SMALL
        case bcnp::PacketError::UnsupportedVersion:  return 2; // UNSUPPORTED_VERSION
        case bcnp::PacketError::TooManyMessages:     return 5; // PAYLOAD_TOO_LARGE
        case bcnp::PacketError::Truncated:           return 1; // INCOMPLETE
        case bcnp::PacketError::InvalidFloat:        return 5; // PAYLOAD_TOO_LARGE (closest match)
        case bcnp::PacketError::ChecksumMismatch:    return 3; // CHECKSUM_MISMATCH
        case bcnp::PacketError::UnknownMessageType:  return 4; // UNKNOWN_MESSAGE_TYPE
        case bcnp::PacketError::HandshakeRequired:   return 1; // INCOMPLETE (closest match)
        case bcnp::PacketError::SchemaMismatch:      return 2; // UNSUPPORTED_VERSION (closest match)
        default:                                     return -1;
    }
}

} // namespace

// JNI function naming: Java_com_bcnp_BcnpJNI_<methodName>

extern "C" {

/**
 * Helper to get direct buffer address and validate.
 */
static uint8_t* GetBufferAddress(JNIEnv* env, jobject buffer, jint offset) {
    if (!buffer) return nullptr;
    void* addr = env->GetDirectBufferAddress(buffer);
    if (!addr) return nullptr;
    return static_cast<uint8_t*>(addr) + offset;
}

/**
 * Decode a packet from a DirectByteBuffer.
 */
JNIEXPORT jboolean JNICALL Java_com_bcnp_BcnpJNI_decodePacket(
    JNIEnv* env, jclass cls,
    jobject buffer, jint offset, jint length,
    jobject result, jobject payloadSlice)
{
    uint8_t* data = GetBufferAddress(env, buffer, offset);
    if (!data || length <= 0) {
        return JNI_FALSE;
    }
    
    // Decode using C++ library
    bcnp::DecodeViewResult decodeResult = bcnp::DecodePacketView(data, static_cast<std::size_t>(length));
    
    // Get result class and methods
    jclass resultClass = env->GetObjectClass(result);
    jmethodID setOkMethod = env->GetMethodID(resultClass, "setOk", "(III)V");
    jmethodID setErrorMethod = env->GetMethodID(resultClass, "setError", "(II)V");
    
    if (decodeResult.error != bcnp::PacketError::None || decodeResult.view.is_none()) {
        // Set error in result object using explicit mapping (not raw ordinals)
        int errorCode = MapPacketErrorToJava(decodeResult.error);
        env->CallVoidMethod(result, setErrorMethod, errorCode, static_cast<jint>(decodeResult.bytesConsumed));
        return JNI_FALSE;
    }
    
    // Success - populate result and payload slice
    const auto& view = decodeResult.view.unwrap();
    env->CallVoidMethod(result, setOkMethod,
        static_cast<jint>(decodeResult.bytesConsumed),
        static_cast<jint>(view.header.messageType),
        static_cast<jint>(view.header.messageCount));
    
    // Populate payload slice
    jclass sliceClass = env->GetObjectClass(payloadSlice);
    jmethodID wrapMethod = env->GetMethodID(sliceClass, "wrap", "(Ljava/nio/ByteBuffer;II)V");
    
    // Calculate payload offset within the original buffer
    jint payloadOffset = offset + bcnp::kHeaderSizeV3;
    jint payloadLength = static_cast<jint>(view.payload.size());
    
    env->CallVoidMethod(payloadSlice, wrapMethod, buffer, payloadOffset, payloadLength);
    
    return JNI_TRUE;
}

/**
 * Encode a packet into a DirectByteBuffer.
 */
JNIEXPORT jint JNICALL Java_com_bcnp_BcnpJNI_encodePacket(
    JNIEnv* env, jclass cls,
    jobject buffer, jint offset, jint maxLength,
    jint messageType, jint flags,
    jobject payload, jint payloadOffset, jint payloadLength,
    jint messageCount)
{
    uint8_t* dest = GetBufferAddress(env, buffer, offset);
    if (!dest || maxLength <= 0) {
        return -1;
    }
    
    const uint8_t* payloadData = GetBufferAddress(env, payload, payloadOffset);
    if (!payloadData && payloadLength > 0) {
        return -1;
    }
    
    // Calculate expected packet size
    std::size_t headerSize = bcnp::kHeaderSizeV3;
    std::size_t crcSize = bcnp::kChecksumSize;
    std::size_t totalSize = headerSize + static_cast<std::size_t>(payloadLength) + crcSize;
    
    if (totalSize > static_cast<std::size_t>(maxLength)) {
        return -2; // Buffer too small
    }
    
    // Build header
    bcnp::PacketHeader header{};
    header.messageType = static_cast<bcnp::MessageTypeId>(messageType);
    header.messageCount = static_cast<uint16_t>(messageCount);
    header.flags = static_cast<uint8_t>(flags);
    
    // Write header (V3 format)
    dest[0] = bcnp::kProtocolMajor;
    dest[1] = bcnp::kProtocolMinor;
    dest[2] = header.flags;
    bcnp::detail::StoreU16(static_cast<uint16_t>(header.messageType), &dest[3]);
    bcnp::detail::StoreU16(header.messageCount, &dest[5]);
    
    // Copy payload
    if (payloadLength > 0) {
        std::memcpy(dest + headerSize, payloadData, static_cast<std::size_t>(payloadLength));
    }
    
    // Compute and write CRC
    std::size_t dataLen = headerSize + static_cast<std::size_t>(payloadLength);
    uint32_t crc = bcnp::ComputeCrc32(dest, dataLen);
    bcnp::detail::StoreU32(crc, &dest[dataLen]);
    
    return static_cast<jint>(totalSize);
}

/**
 * Compute CRC32 checksum.
 */
JNIEXPORT jint JNICALL Java_com_bcnp_BcnpJNI_computeCrc32(
    JNIEnv* env, jclass cls,
    jobject buffer, jint offset, jint length)
{
    uint8_t* data = GetBufferAddress(env, buffer, offset);
    if (!data || length <= 0) {
        return 0;
    }
    return static_cast<jint>(bcnp::ComputeCrc32(data, static_cast<std::size_t>(length)));
}

/**
 * Get wire size for a message type.
 */
JNIEXPORT jint JNICALL Java_com_bcnp_BcnpJNI_getMessageWireSize(
    JNIEnv* env, jclass cls, jint messageTypeId)
{
    auto info = bcnp::GetMessageInfo(static_cast<bcnp::MessageTypeId>(messageTypeId));
    return info ? static_cast<jint>(info->wireSize) : 0;
}

// StreamParser handle storage (simplified, in production use proper handle management)
#include "bcnp/stream_parser.h"

/**
 * Stores a deep copy of a decoded packet, avoiding dangling pointers
 * into the StreamParser's internal ring buffer.
 */
struct StoredPacket {
    bcnp::PacketHeader header;
    std::vector<uint8_t> payloadData;  // Stable backing storage

    StoredPacket(const bcnp::PacketView& view)
        : header(view.header)
        , payloadData(view.payload.data(), view.payload.data() + view.payload.size()) {}
};

struct JavaStreamParser {
    std::deque<StoredPacket> pendingPackets;
    bcnp::StreamParser* parser;
    
    JavaStreamParser(int capacity) {
        parser = new bcnp::StreamParser(
            [this](const bcnp::PacketView& pkt) {
                // Deep-copy the packet to avoid dangling references
                pendingPackets.emplace_back(pkt);
            },
            [](const bcnp::StreamParser::ErrorInfo&) {},
            static_cast<std::size_t>(capacity));
    }
    
    ~JavaStreamParser() {
        delete parser;
    }
};

JNIEXPORT jlong JNICALL Java_com_bcnp_BcnpJNI_createStreamParser(
    JNIEnv* env, jclass cls, jint bufferCapacity)
{
    auto* jsp = new JavaStreamParser(bufferCapacity);
    return reinterpret_cast<jlong>(jsp);
}

JNIEXPORT void JNICALL Java_com_bcnp_BcnpJNI_destroyStreamParser(
    JNIEnv* env, jclass cls, jlong handle)
{
    if (handle) {
        delete reinterpret_cast<JavaStreamParser*>(handle);
    }
}

JNIEXPORT void JNICALL Java_com_bcnp_BcnpJNI_streamParserPush(
    JNIEnv* env, jclass cls,
    jlong handle, jobject buffer, jint offset, jint length)
{
    if (!handle) return;
    uint8_t* data = GetBufferAddress(env, buffer, offset);
    if (!data || length <= 0) return;
    
    auto* jsp = reinterpret_cast<JavaStreamParser*>(handle);
    jsp->parser->Push(data, static_cast<std::size_t>(length));
}

JNIEXPORT jboolean JNICALL Java_com_bcnp_BcnpJNI_streamParserPop(
    JNIEnv* env, jclass cls,
    jlong handle, jobject result, jobject payloadSlice)
{
    if (!handle) return JNI_FALSE;
    auto* jsp = reinterpret_cast<JavaStreamParser*>(handle);
    
    if (jsp->pendingPackets.empty()) {
        return JNI_FALSE;
    }
    
    const auto& stored = jsp->pendingPackets.front();
    
    // Populate result
    jclass resultClass = env->GetObjectClass(result);
    jmethodID setOkMethod = env->GetMethodID(resultClass, "setOk", "(III)V");
    jclass sliceClass = env->GetObjectClass(payloadSlice);
    jmethodID wrapMethod = env->GetMethodID(sliceClass, "wrap", "(Ljava/nio/ByteBuffer;II)V");
    jclass byteBufferClass = env->FindClass("java/nio/ByteBuffer");
    jmethodID allocateDirectMethod = env->GetStaticMethodID(
        byteBufferClass, "allocateDirect", "(I)Ljava/nio/ByteBuffer;");
    
    // Note: bytesConsumed is not meaningful here, set to payload size
    env->CallVoidMethod(result, setOkMethod,
        static_cast<jint>(stored.payloadData.size()),
        static_cast<jint>(stored.header.messageType),
        static_cast<jint>(stored.header.messageCount));
    
    jobject payloadBuffer = env->CallStaticObjectMethod(
        byteBufferClass,
        allocateDirectMethod,
        static_cast<jint>(stored.payloadData.size()));
    if (!payloadBuffer || env->ExceptionCheck()) {
        env->ExceptionClear();
        return JNI_FALSE;
    }

    auto* payloadDest = static_cast<uint8_t*>(env->GetDirectBufferAddress(payloadBuffer));
    if (payloadDest && !stored.payloadData.empty()) {
        std::memcpy(payloadDest, stored.payloadData.data(), stored.payloadData.size());
    }
    env->CallVoidMethod(payloadSlice, wrapMethod, payloadBuffer, 0,
                        static_cast<jint>(stored.payloadData.size()));
    
    jsp->pendingPackets.pop_front();
    return JNI_TRUE;
}

} // extern "C"
