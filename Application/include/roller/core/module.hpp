#pragma once
#include <array>
#include <cstddef>
#include <expected>
#include <string_view>

namespace roller {
enum class Error { invalid_state, duplicate, capacity, hardware, configuration, timeout, cleanup };
using Result = std::expected<void, Error>;

// Template Method: all modules share exactly one lifecycle implementation.
class Module {
  public:
    enum class State { stopped, started, cleanup_failed };
    Module(const Module&) = delete;
    Module& operator=(const Module&) = delete;
    [[nodiscard]] virtual std::string_view name() const noexcept = 0;
    [[nodiscard]] Result start() noexcept {
        if (state_ == State::started) {
            return {};
        }
        if (state_ != State::stopped) {
            return std::unexpected(Error::invalid_state);
        }
        auto result = on_start();
        if (result) {
            state_ = State::started;
        } else if (!on_stop()) {
            state_ = State::cleanup_failed;
            return std::unexpected(Error::cleanup);
        }
        return result;
    }
    [[nodiscard]] Result stop() noexcept {
        if (state_ == State::stopped) {
            return {};
        }
        auto result = on_stop();
        state_ = result ? State::stopped : State::cleanup_failed;
        return result;
    }
    [[nodiscard]] State state() const noexcept {
        return state_;
    }

  protected:
    Module() = default;
    ~Module() = default; // Non-owning interface: deletion through Module* is prohibited.
    virtual Result on_start() noexcept = 0;
    virtual Result on_stop() noexcept = 0;

  private:
    State state_{State::stopped};
};

// Non-owning registry; module storage must outlive the manager and all interrupts.
template <std::size_t Capacity = 4> class ModuleManager {
  public:
    [[nodiscard]] Result add(Module& module) noexcept {
        if (running_ || faulted_) {
            return std::unexpected(Error::invalid_state);
        }
        if (size_ == Capacity) {
            return std::unexpected(Error::capacity);
        }
        for (std::size_t i = 0; i < size_; ++i) {
            if (modules_[i]->name() == module.name()) {
                return std::unexpected(Error::duplicate);
            }
        }
        modules_[size_++] = &module;
        return {};
    }
    [[nodiscard]] Result start() noexcept {
        if (faulted_) {
            return std::unexpected(Error::invalid_state);
        }
        if (running_) {
            return {};
        }
        for (std::size_t i = 0; i < size_; ++i) {
            if (auto result = modules_[i]->start(); !result) {
                // A failed cleanup may still depend on earlier modules. Keep them alive.
                if (modules_[i]->state() == Module::State::cleanup_failed) {
                    faulted_ = true;
                    return std::unexpected(Error::cleanup);
                }
                while (i > 0) {
                    if (!modules_[--i]->stop()) {
                        faulted_ = true;
                        return std::unexpected(Error::cleanup);
                    }
                }
                return result;
            }
        }
        running_ = true;
        return {};
    }
    [[nodiscard]] Result stop() noexcept {
        for (auto i = size_; i > 0; --i) {
            if (!modules_[i - 1]->stop()) {
                faulted_ = true;
                return std::unexpected(Error::cleanup);
            }
        }
        running_ = faulted_ = false;
        return {};
    }

  private:
    std::array<Module*, Capacity> modules_{};
    std::size_t size_{};
    bool running_{}, faulted_{};
};
} // namespace roller
