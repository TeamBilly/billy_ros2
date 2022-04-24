#ifndef BILLY_DESCRIPTION__VISIBILITY_CONTROL_H_
#define BILLY_DESCRIPTION__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define BILLY_DESCRIPTION_EXPORT __attribute__ ((dllexport))
    #define BILLY_DESCRIPTION_IMPORT __attribute__ ((dllimport))
  #else
    #define BILLY_DESCRIPTION_EXPORT __declspec(dllexport)
    #define BILLY_DESCRIPTION_IMPORT __declspec(dllimport)
  #endif
  #ifdef BILLY_DESCRIPTION_BUILDING_LIBRARY
    #define BILLY_DESCRIPTION_PUBLIC BILLY_DESCRIPTION_EXPORT
  #else
    #define BILLY_DESCRIPTION_PUBLIC BILLY_DESCRIPTION_IMPORT
  #endif
  #define BILLY_DESCRIPTION_PUBLIC_TYPE BILLY_DESCRIPTION_PUBLIC
  #define BILLY_DESCRIPTION_LOCAL
#else
  #define BILLY_DESCRIPTION_EXPORT __attribute__ ((visibility("default")))
  #define BILLY_DESCRIPTION_IMPORT
  #if __GNUC__ >= 4
    #define BILLY_DESCRIPTION_PUBLIC __attribute__ ((visibility("default")))
    #define BILLY_DESCRIPTION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define BILLY_DESCRIPTION_PUBLIC
    #define BILLY_DESCRIPTION_LOCAL
  #endif
  #define BILLY_DESCRIPTION_PUBLIC_TYPE
#endif

#endif  // BILLY_DESCRIPTION__VISIBILITY_CONTROL_H_
