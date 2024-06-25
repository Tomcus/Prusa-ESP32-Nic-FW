#pragma once

#include <array>
#include <cstdint>
#include <span>

#include "messages.hpp"
#include "checksum.hpp"

namespace esp {

struct RxParserBase {
    RxParserBase() = default;
    using Input = std::span<const uint8_t>;
    static constexpr size_t SMALL_BUFFER_SIZE = 64;

    void set_intron(std::span<const uint8_t, INTRON_SIZE> new_intron);

    void process_data(Input data);

    void reset();

protected:
    enum class State : uint8_t {
        Intron,
        Header,
        Data,
        ThrowAwayData,
    };

    [[nodiscard]] bool wait_for_intron();
    [[nodiscard]] bool wait_for_buffer();
    [[nodiscard]] bool wait_for_data();
    [[nodiscard]] bool validate_length_with_type() const;
    void on_parsed();

    virtual void process_scan_ap_count() = 0;
    virtual void process_scan_ap_info() = 0;
    virtual void process_invalid_message() = 0;
    virtual void process_esp_device_info() = 0;
    virtual bool start_packet() = 0;
    virtual void reset_packet() = 0;
    virtual void update_packet(std::span<const uint8_t>) = 0;
    virtual void process_packet() = 0;

    MessagePrelude msg;
    Fletcher16 fl16;
    Input::iterator curr, end;
    std::array<std::uint8_t, SMALL_BUFFER_SIZE> small_buffer;
    std::span<uint8_t> buffer;
    std::span<const uint8_t> intron;
    uint16_t read;
    State state;
    bool checksum_valid;
};

} // namespace esp
