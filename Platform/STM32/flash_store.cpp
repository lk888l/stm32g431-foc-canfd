#include "board.hpp"
#include "main.h"
#include "roller/core/parameter_journal.hpp"

namespace roller::platform {
namespace {
class Stm32Pages final : public ParameterPages {
  public:
    std::span<const std::byte, page_size> page(unsigned index) const noexcept override {
        return std::span<const std::byte, page_size>{reinterpret_cast<const std::byte*>(address(index)),
                                                     page_size};
    }
    bool erase(unsigned index) noexcept override {
        FLASH_EraseInitTypeDef request{};
        request.TypeErase = FLASH_TYPEERASE_PAGES;
        request.Banks = FLASH_BANK_1;
        request.Page = 62 + index;
        request.NbPages = 1;
        std::uint32_t failed = 0;
        return HAL_FLASHEx_Erase(&request, &failed) == HAL_OK;
    }
    bool program_word(unsigned index, std::size_t offset, std::uint64_t word) noexcept override {
        return HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                 static_cast<std::uint32_t>(address(index) + offset), word) == HAL_OK;
    }

  private:
    static std::uintptr_t address(unsigned index) noexcept {
        return 0x0801F000U + index * page_size;
    }
};
Stm32Pages pages;
ParameterJournal journal{pages};
} // namespace
bool FlashStore::read(std::span<std::byte> data) noexcept {
    return journal.read(data);
}
bool FlashStore::write(std::span<const std::byte> data) noexcept {
    if (data.empty() || data.size() > ParameterJournal::max_payload || (GPIOB->ODR & PWM_ENABLE_Pin)) {
        return false;
    }
    const bool resume = board_.sampling();
    board_.stop_sampling();
    // G431 is single bank: ADC/PWM are paused while flash fetches may stall.
    bool success = HAL_FLASH_Unlock() == HAL_OK;
    if (success) {
        success = journal.write(data);
    }
    (void)HAL_FLASH_Lock();
    if (resume && !board_.start_sampling()) {
        success = false;
    }
    return success;
}
} // namespace roller::platform
