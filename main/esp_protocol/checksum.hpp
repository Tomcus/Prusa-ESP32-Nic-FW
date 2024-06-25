#pragma once
#include <cstdint>
#include <optional>

#if __has_include("span")
    #include <span>
#elif __has_include("gsl/span")
    #include <gsl/span>
using gsl::span;
#else
    #error "Esp protocol requires some type of span (eather std or gsl)"
#endif

namespace esp {

#if __has_include("span")
using ChecksumInput = std::span<const uint8_t>;
#elif __has_include("gsl/span")
using ChecksumInput = gsl::span<const uint8_t>;
#else
    #error "Esp protocol requires some type of span (eather std or gsl)"
#endif

constexpr uint32_t FLETCHER_16_MODULO = 0xffff;
constexpr uint32_t FLETCHER_16_MAX_BULK_SIZE = 360 * 2; // *2 because we are taking bytes as input

/*
 * Implementation of Fletcher16 algorithm
 *
 * The number in the name is based on RFC 1146, where the number references the number of bits in the input piece.
 * Some sources (like wikipedia) are using the size of the output as the number. So make sure when googling that
 * you are looking for the right algorithm (been there, done that).
 *
 * If we found that the algorithm isn't good enough, then there is alternative in the Adler algorithm, that can
 * have better results, but should be similar: https://www.zlib.net/maxino06_fletcher-adler.pdf
 */
struct Fletcher16 {
    constexpr Fletcher16()
        : c0(0)
        , c1(0)
        , overflow_byte(std::nullopt) {}

    constexpr void update(ChecksumInput data) {
        if (data.size() == 0) {
            return;
        }
        size_t initial_offset = 0;
        if (overflow_byte.has_value()) {
            initial_offset = 1;
            uint16_t block = *overflow_byte;
            block |= data[0] << 8;
            overflow_byte.reset();

            c0 = (c0 + block) % FLETCHER_16_MODULO;
            c1 = (c1 + c0) % FLETCHER_16_MODULO;
        }

        size_t len = data.size() - initial_offset;
        size_t iterations = 0;

        while (len > 0) {
            size_t blocklen = len;
            if (blocklen >= FLETCHER_16_MAX_BULK_SIZE) {
                blocklen = FLETCHER_16_MAX_BULK_SIZE;
            }
            const auto base_offset = iterations * FLETCHER_16_MAX_BULK_SIZE + initial_offset;
            for (size_t i = 0; i < blocklen; i += 2) {
                if (i + 1 != len) {
                    // TODO: Both pritner and esps are LE, but if we ever use BE machine,
                    // we should make sure that the order is correct based on the architecture
                    // (the whole espif.cpp requires for both devices to be LE)
                    uint16_t block = data[base_offset + i];
                    block |= data[base_offset + i + 1] << 8;
                    c0 += block;
                    c1 += c0;
                } else {
                    overflow_byte = data[i + base_offset];
                }
            }
            ++iterations;
            len -= blocklen;
            c0 %= FLETCHER_16_MODULO;
            c1 %= FLETCHER_16_MODULO;
        }
    }

    [[nodiscard]] constexpr uint32_t get() const {
        uint32_t tmp_c0 = c0;
        uint32_t tmp_c1 = c1;
        if (overflow_byte.has_value()) {
            uint16_t block = *overflow_byte;
            tmp_c0 = (tmp_c0 + block) % FLETCHER_16_MODULO;
            tmp_c1 = (tmp_c1 + tmp_c0) % FLETCHER_16_MODULO;
        }
        return tmp_c0 | (tmp_c1 << 16);
    }

    void reset() {
        c0 = 0;
        c1 = 0;
        overflow_byte = std::nullopt;
    }

protected:
    uint32_t c0, c1;
    std::optional<uint8_t> overflow_byte;
};

constexpr uint32_t fletcher16(ChecksumInput data) {
    Fletcher16 fl16 {};
    fl16.update(data);
    return fl16.get();
}

} // namespace esp
