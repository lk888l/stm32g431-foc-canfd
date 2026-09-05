#pragma once
#include "roller/core/storage.hpp"
#include <algorithm>
#include <bit>
#include <cstring>

namespace roller {
class ParameterPages {
  public:
    static constexpr std::size_t page_size = 2048;
    virtual std::span<const std::byte, page_size> page(unsigned index) const noexcept = 0;
    virtual bool erase(unsigned index) noexcept = 0;
    virtual bool program_word(unsigned index, std::size_t offset, std::uint64_t value) noexcept = 0;

  protected:
    ~ParameterPages() = default;
};

// Two-slot journal: the active page is never erased, and the commit word is
// programmed last. CRC covers metadata as well as payload. No dynamic storage.
class ParameterJournal final : public ParameterStore {
  public:
    static constexpr std::size_t max_payload = 2000;
    explicit ParameterJournal(ParameterPages& pages) noexcept : pages_(pages) {}
    bool read(std::span<std::byte> data) noexcept override {
        const auto a = inspect(0, data.size()), b = inspect(1, data.size());
        const auto index = b.valid && (!a.valid || newer(b.header.sequence, a.header.sequence)) ? 1U : 0U;
        if (!(index ? b : a).valid) {
            return false;
        }
        const auto bytes = pages_.page(index).subspan(sizeof(Header), data.size());
        std::copy(bytes.begin(), bytes.end(), data.begin());
        return true;
    }
    bool write(std::span<const std::byte> data) noexcept override {
        if (data.empty() || data.size() > max_payload) {
            return false;
        }
        const auto a = inspect(0, data.size()), b = inspect(1, data.size());
        const bool latest_b = b.valid && (!a.valid || newer(b.header.sequence, a.header.sequence));
        const auto target = latest_b ? 0U : 1U;
        Header header{static_cast<std::uint32_t>(data.size()),
                      (latest_b ? b.header.sequence : (a.valid ? a.header.sequence : 0)) + 1U, 1, 0};
        header.crc = checksum(header, data);
        if (!pages_.erase(target) || !program(target, 0, std::as_bytes(std::span{&header, 1})) ||
            !program(target, sizeof(Header), data)) {
            return false;
        }
        if (checksum(header, pages_.page(target).subspan(sizeof(Header), data.size())) != header.crc) {
            return false;
        }
        if (!pages_.program_word(target, ParameterPages::page_size - 8, commit)) {
            return false;
        }
        return inspect(target, data.size()).valid;
    }

  private:
    struct Header {
        std::uint32_t length, sequence, version, crc;
    };
    struct Record {
        bool valid{};
        Header header{};
    };
    static_assert(sizeof(Header) == 16 && std::endian::native == std::endian::little);
    static constexpr std::uint64_t commit = 0x524F4C4C45523031ULL;
    static bool newer(std::uint32_t a, std::uint32_t b) noexcept {
        return a != b && (a - b) < 0x80000000U;
    }
    static std::uint32_t checksum(const Header& header, std::span<const std::byte> payload) noexcept {
        return crc32(payload, crc32(std::as_bytes(std::span{&header, 1}).first(offsetof(Header, crc))));
    }
    Record inspect(unsigned index, std::size_t size) const noexcept {
        Record record{};
        if (size == 0 || size > max_payload) {
            return record;
        }
        const auto bytes = pages_.page(index);
        std::uint64_t marker{};
        std::memcpy(&marker, bytes.data() + ParameterPages::page_size - 8, 8);
        if (marker != commit) {
            return record;
        }
        std::memcpy(&record.header, bytes.data(), sizeof(Header));
        if (record.header.length != size || record.header.version != 1) {
            return record;
        }
        record.valid = checksum(record.header, bytes.subspan(sizeof(Header), size)) == record.header.crc;
        return record;
    }
    bool program(unsigned index, std::size_t offset, std::span<const std::byte> data) noexcept {
        for (std::size_t i = 0; i < data.size(); i += 8) {
            std::uint64_t word = 0xFFFFFFFFFFFFFFFFULL;
            std::memcpy(&word, data.data() + i, std::min<std::size_t>(8, data.size() - i));
            if (!pages_.program_word(index, offset + i, word)) {
                return false;
            }
        }
        return true;
    }
    ParameterPages& pages_;
};
} // namespace roller
