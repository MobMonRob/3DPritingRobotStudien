#pragma once

#include <string>
#include <variant>

namespace medusa::kronos {

/// Lightweight result type that holds either a success value (T) or an
/// error message (E, defaults to std::string).
/// Avoids exception overhead across module boundaries.
template <typename T, typename E = std::string>
class Result {
public:
    /// Creates a successful result.
    [[nodiscard]] static Result ok(T value) {
        return Result{std::in_place_index<0>, std::move(value)};
    }

    /// Creates an error result.
    [[nodiscard]] static Result err(E error) {
        return Result{std::in_place_index<1>, std::move(error)};
    }

    [[nodiscard]] bool is_ok()  const noexcept { return data_.index() == 0; }
    [[nodiscard]] bool is_err() const noexcept { return data_.index() == 1; }

    /// Success value – valid only when is_ok() == true.
    [[nodiscard]] const T& value() const&  { return std::get<0>(data_); }
    [[nodiscard]] T&       value() &       { return std::get<0>(data_); }
    [[nodiscard]] T        value() &&      { return std::get<0>(std::move(data_)); }

    /// Error message – valid only when is_err() == true.
    [[nodiscard]] const E& error() const&  { return std::get<1>(data_); }
    [[nodiscard]] E&       error() &       { return std::get<1>(data_); }

private:
    template <std::size_t I, typename... Args>
    explicit Result(std::in_place_index_t<I> idx, Args&&... args)
        : data_{idx, std::forward<Args>(args)...} {}

    std::variant<T, E> data_;
};

} // namespace medusa::kronos
