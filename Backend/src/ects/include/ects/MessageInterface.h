#pragma once

#include <functional>

namespace ects {

template<typename internal_t>
struct server_traits;

template<typename internal_t>
struct client_traits;

#ifdef concepts
    template<typename T>
    concept from_ros_message =
            requires { typename T::from_ros_t; } &&
            requires(T::from_ros_t r) {
                { T::from_ros(r) } -> std::same_as<T>;
            };

    template<typename T>
    concept to_ros_message =
            requires { typename T::to_ros_t; } &&
            requires(T t) {
                { T::to_ros(t) } -> std::same_as<typename T::to_ros_t>;
            };


    template<typename T>
    concept server_messages =
            from_ros_message<typename server_traits<T>::request_from_ros_t> &&
            to_ros_message<typename server_traits<T>::response_to_ros_t> &&
            requires {
                typename server_traits<T>::ros_t;
            };

    template<typename T>
    concept client_messages =
            to_ros_message<typename T::request_to_ros_t> &&
            from_ros_message < typename T::response_from_ros_t> &&
            requires {
                typename T::ros_t;
            };
#else

#define from_ros_message typename
#define to_ros_message typename
#define server_messages typename
#define client_messages typename


#endif

}
