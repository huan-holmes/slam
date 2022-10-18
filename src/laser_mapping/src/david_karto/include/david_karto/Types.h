//*****************************************************************************
// Copyright (c) 2022 MOI SmartRobotTechnology Co.,Ltd.
//*****************************************************************************

#ifndef DAVID_KARTO_TYPES_H
#define DAVID_KARTO_TYPES_H

#include <cstddef>

/**
 * Karto type definition
 * Ensures platform independent types for windows, linux and mac
 */
#if defined(_MSC_VER)

typedef signed __int8 kt_int8s;
typedef unsigned __int8 kt_int8u;

typedef signed __int16 kt_int16s;
typedef unsigned __int16 kt_int16u;

typedef signed __int32 kt_int32s;
typedef unsigned __int32 kt_int32u;

typedef signed __int64 kt_int64s;
typedef unsigned __int64 kt_int64u;

#else

#include <stdint.h>

typedef int8_t kt_int8s;
typedef uint8_t kt_int8u;

typedef int16_t kt_int16s;
typedef uint16_t kt_int16u;

typedef int32_t kt_int32s;
typedef uint32_t kt_int32u;

#if defined(__LP64__)
typedef signed long kt_int64s;
typedef unsigned long kt_int64u;
#else
typedef signed long long kt_int64s;
typedef unsigned long long kt_int64u;
#endif

#endif

typedef bool kt_bool;
typedef char kt_char;
typedef float kt_float;
typedef double kt_double;

#endif  // OPEN_KARTO_TYPES_H
