#ifndef SERIAL_API_H
#define SERIAL_API_H

#if defined(_WIN32)
#  if !defined(LIBSERIAL_DYNAMIC)
#    define LIBSERIAL_API
#  else
#    if defined(LIBSERIAL_EXPORTS)
#      define LIBSERIAL_API __declspec(dllexport)
#    else
#      define LIBSERIAL_API __declspec(dllimport)
#    endif
#  endif
#else
#  define LIBSERIAL_API
#endif

#endif //SERIAL_API_H