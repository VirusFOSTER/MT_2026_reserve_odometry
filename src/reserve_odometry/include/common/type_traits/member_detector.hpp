#ifndef SYSTEM_MEMBER_DETECTOR_H
#define SYSTEM_MEMBER_DETECTOR_H

#define GENERATE_REQUIRED_MEMBER_CLASS(field)                                                       \
    template <class T, class Enable = void>                                                         \
    class RequireField_##field {                                                                    \
    public:                                                                                         \
    static constexpr bool _result = false;                                                          \
    };                                                                                              \
                                                                                                    \
    template <class T>                                                                              \
    class RequireField_##field <T,                                                                  \
                    typename boost::enable_if <                                                     \
                    boost::is_class<T>                                                              \
                    >::type> {                                                                      \
    using has_field = char[2];                                                                      \
    using hasnt_field = char[1];                                                                    \
                                                                                                    \
    struct Fallback { int field; };                                                                 \
                                                                                                    \
    struct Derived : T, Fallback {  };                                                              \
                                                                                                    \
    template <class U>                                                                              \
    static has_field& check(U*);                                                                    \
                                                                                                    \
    template <class U>                                                                              \
    static hasnt_field& check(decltype(U::field)*);                                                 \
                                                                                                    \
    public:                                                                                         \
    static constexpr bool _result =                                                                 \
    sizeof(check<Derived>(nullptr)) == sizeof(has_field);                                           \
};                                                                                                  \
                                                                                                    \
    template <class T>                                                                              \
    struct has_field_##field                                                                        \
    : public boost::integral_constant<bool,RequireField_##field<T>::_result> {                      \
};

#endif
