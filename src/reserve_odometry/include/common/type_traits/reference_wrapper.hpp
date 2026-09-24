#ifndef REFERENCE_WRAPPER_HPP
#define REFERENCE_WRAPPER_HPP

#include <functional>

namespace sys {
namespace type_traits {
namespace detail
{
template<class T> constexpr T& FUN(T& t) noexcept { return t; }
template<class T> void FUN(T&&) = delete;
}

template<class T>
class reference_wrapper
{
public:
    // types
    using type = T;

    // construct/copy/destroy
    template<class U, class = decltype(
                          detail::FUN<T>(std::declval<U>()),
                          std::enable_if_t<!std::is_same_v<reference_wrapper, std::remove_cvref_t<U>>>()
                          )>
    constexpr reference_wrapper(U&& u)
        noexcept(noexcept(detail::FUN<T>(std::forward<U>(u))))
        : _ptr(std::addressof(detail::FUN<T>(std::forward<U>(u)))) {}

    reference_wrapper(const reference_wrapper&) noexcept = default;

    // assignment
    reference_wrapper& operator=(const reference_wrapper& x) noexcept = default;

    bool operator < (const reference_wrapper& x) {
        return this->get().um_signal_clone().um_position()[2] < x.get().um_signal_clone().um_position()[2];
    }

    bool operator == (const reference_wrapper& x) {
        return this->get().um_signal_clone().um_position()[2] == x.get().um_signal_clone().um_position()[2];
    }

    // access
    constexpr operator T& () const noexcept { return *_ptr; }
    constexpr T& get() const noexcept { return *_ptr; }

    template<class... ArgTypes>
    constexpr std::invoke_result_t<T&, ArgTypes...>
    operator() (ArgTypes&&... args ) const
        noexcept(std::is_nothrow_invocable_v<T&, ArgTypes...>)
    {
        return std::invoke(get(), std::forward<ArgTypes>(args)...);
    }

private:
    T* _ptr;
};

template <class T>
bool operator < (const reference_wrapper<T>& rf_1_, const reference_wrapper<T>& rf_2_) {
    return rf_1_.get().um_signal_clone().um_position()[2] < rf_2_.get().um_signal_clone().um_position()[2];
}

template <class T>
bool operator == (const reference_wrapper<T>& rf_1_, const reference_wrapper<T>& rf_2_) {
    return rf_1_.get().um_signal_clone().um_position()[2] == rf_2_.get().um_signal_clone().um_position()[2];
}

// deduction guides
template<class T>
reference_wrapper(T&) -> reference_wrapper<T>;
}
}

#endif
