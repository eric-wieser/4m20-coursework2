/*
The purpose of this file is to provide the <type_traits> header that is part of C++
Arduino does not include it. This code is copied directly from C++ documentation online
*/

namespace std {
  // from http://en.cppreference.com/w/cpp/types/integral_constant#Possible_implementation
  template<class T, T v>
  struct integral_constant {
    static constexpr T value = v;
    typedef T value_type;
    typedef integral_constant type; // using injected-class-name
    constexpr operator value_type() const noexcept { return value; }
    constexpr value_type operator()() const noexcept { return value; } //since c++14
  };

  typedef std::integral_constant<bool, true> true_type;
  typedef std::integral_constant<bool, false> false_type;

  // from http://en.cppreference.com/w/cpp/types/enable_if#Possible_implementation
  template<bool B, class T = void>
  struct enable_if {};

  template<class T>
  struct enable_if<true, T> { typedef T type; };

  // from http://en.cppreference.com/w/cpp/types/remove_cv#Possible_implementation
  template< class T > struct remove_const          { typedef T type; };
  template< class T > struct remove_const<const T> { typedef T type; };

  template< class T > struct remove_volatile             { typedef T type; };
  template< class T > struct remove_volatile<volatile T> { typedef T type; };

  template< class T >
  struct remove_cv {
    typedef typename std::remove_volatile<typename std::remove_const<T>::type>::type type;
  };

  // from http://en.cppreference.com/w/cpp/types/add_cv#Possible_implementation
  template< class T> struct add_const { typedef const T type; };

  template< class T> struct add_volatile { typedef volatile T type; };

  template< class T >
  struct add_cv {
    typedef typename std::add_volatile<typename std::add_const<T>::type>::type type;
  };


  // from http://en.cppreference.com/w/cpp/types/remove_pointer#Possible_implementation
  template< class T > struct remove_pointer                    {typedef T type;};
  template< class T > struct remove_pointer<T*>                {typedef T type;};
  template< class T > struct remove_pointer<T* const>          {typedef T type;};
  template< class T > struct remove_pointer<T* volatile>       {typedef T type;};
  template< class T > struct remove_pointer<T* const volatile> {typedef T type;};

  // from http://en.cppreference.com/w/cpp/types/is_same#Possible_implementation
  template<class T, class U>
  struct is_same : std::false_type {};

  template<class T>
  struct is_same<T, T> : std::true_type {};
}
