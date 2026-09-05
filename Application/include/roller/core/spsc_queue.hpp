#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <type_traits>

namespace roller {
// Single producer/single consumer ONLY. Publication is release/acquire; payload
// slots are never concurrently read and written. No seqlock over non-atomic data.
template <class T, std::uint32_t Capacity> class SpscQueue {
    static_assert(Capacity > 1 && (Capacity & (Capacity - 1)) == 0);
    static_assert(std::is_trivially_copyable_v<T>);
    static_assert(std::atomic<std::uint32_t>::is_always_lock_free);

  public:
    [[nodiscard]] bool push(const T& value) noexcept {
        const auto head = head_.load(std::memory_order_relaxed);
        if (head - tail_.load(std::memory_order_acquire) == Capacity) {
            return false;
        }
        values_[head & (Capacity - 1)] = value;
        head_.store(head + 1, std::memory_order_release);
        return true;
    }
    [[nodiscard]] bool pop(T& value) noexcept {
        const auto tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) {
            return false;
        }
        value = values_[tail & (Capacity - 1)];
        tail_.store(tail + 1, std::memory_order_release);
        return true;
    }
    [[nodiscard]] bool latest(T& value) noexcept {
        bool found = false;
        // Bound ISR work even if the producer runs concurrently on a host CPU.
        for (std::uint32_t i = 0; i < Capacity && pop(value); ++i) {
            found = true;
        }
        return found;
    }

  private:
    std::array<T, Capacity> values_{};
    std::atomic<std::uint32_t> head_{}, tail_{};
};
} // namespace roller
