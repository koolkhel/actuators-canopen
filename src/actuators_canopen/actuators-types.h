#pragma once

#include <stdint.h>

typedef unsigned char BYTE;
typedef unsigned short int WORD;
typedef unsigned int DWORD;
typedef short int SWORD;
typedef float FLOAT;

#ifndef CTASSERT /* Allow lint to override */
#define CTASSERT(x) _CTASSERT(x, __LINE__)
#define _CTASSERT(x, y) __CTASSERT(x, y)
#define __CTASSERT(x, y) typedef char __assert ## y[(x) ? 1 : -1]
#endif

#define PACKED __attribute__ ((__packed__))

/// Byte swap unsigned short
#define FLIP_ENDIAN_WORD(val) ((((val) << 8) & 0xFF00) | (((val) >> 8) & 0xFF))

#ifdef DEBUG
#define TRACE_ENTRY() fprintf(stderr, "entering %s\n", __func__)
#define TRACE_EXIT() fprintf(stderr, "exiting %s\n", __func__)
#else
#define TRACE_ENTRY() do {} while (0)
#define TRACE_EXIT() do {} while (0)
#endif
