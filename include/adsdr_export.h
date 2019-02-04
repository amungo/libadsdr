#ifndef __ADSDR_EXPORT_H__
#define __ADSDR_EXPORT_H__


#if defined __GNUC__
#  if __GNUC__ >= 4
#    define __SDR_EXPORT   __attribute__((visibility("default")))
#    define __SDR_IMPORT   __attribute__((visibility("default")))
#  else
#    define __SDR_EXPORT
#    define __SDR_IMPORT
#  endif
#elif _MSC_VER
#  define __SDR_EXPORT     __declspec(dllexport)
#  define __SDR_IMPORT     __declspec(dllimport)
#else
#  define __SDR_EXPORT
#  define __SDR_IMPORT
#endif

#ifndef adsdr_STATIC
#	ifdef adsdr_EXPORTS
#	define ADSDR_API __SDR_EXPORT
#	else
#	define ADSDR_API __SDR_IMPORT
#	endif
#else
#define ADSDR_API
#endif



#endif // __ADSDR_EXPORT_H__
