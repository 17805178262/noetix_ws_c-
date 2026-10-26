#pragma once
/**
 * base_reader.hpp — Abstract motion capture reader interface
 *
 * Concrete readers (OptiTrack, Pico, XSens, ...) inherit this.
 * Each reader is responsible for:
 *   1. Connecting to its data source
 *   2. Converting device-specific bone names to the canonical names
 *      expected by the IK config JSON
 *   3. Pushing RawFrames into the shared FrameQueue
 */

#include "../include/gmr/frame_queue.hpp"
#include <string>

namespace gmr {

class BaseReader {
public:
    explicit BaseReader(FrameQueue& queue) : queue_(queue) {}
    virtual ~BaseReader() = default;

    // Connect to the data source. Throws on failure.
    virtual void connect() = 0;

    // Disconnect and release resources.
    virtual void disconnect() = 0;

    // Human-readable name for logging ("OptiTrack", "Pico", ...)
    virtual std::string name() const = 0;

    // True if currently connected and receiving data.
    virtual bool isConnected() const = 0;

protected:
    FrameQueue& queue_;
};

} // namespace gmr
