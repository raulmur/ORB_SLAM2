/*M///////////////////////////////////////////////////////////////////////////////////////
 //
 //  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
 //
 //  By downloading, copying, installing or using the software you agree to this license.
 //  If you do not agree to this license, do not download, install,
 //  copy or use the software.
 //
 //
 //                           License Agreement
 //                For Open Source Computer Vision Library
 //
 // Copyright (C) 2014, Mohammad Norouzi, Ali Punjani, David J. Fleet,
 // all rights reserved.
 // Third party copyrights are property of their respective owners.
 //
 // Redistribution and use in source and binary forms, with or without modification,
 // are permitted provided that the following conditions are met:
 //
 //   * Redistribution's of source code must retain the above copyright notice,
 //     this list of conditions and the following disclaimer.
 //
 //   * Redistribution's in binary form must reproduce the above copyright notice,
 //     this list of conditions and the following disclaimer in the documentation
 //     and/or other materials provided with the distribution.
 //
 //   * The name of the copyright holders may not be used to endorse or promote products
 //     derived from this software without specific prior written permission.
 //
 // This software is provided by the copyright holders and contributors "as is" and
 // any express or implied warranties, including, but not limited to, the implied
 // warranties of merchantability and fitness for a particular purpose are disclaimed.
 // In no event shall the Intel Corporation or contributors be liable for any direct,
 // indirect, incidental, special, exemplary, or consequential damages
 // (including, but not limited to, procurement of substitute goods or services;
 // loss of use, data, or profits; or business interruption) however caused
 // and on any theory of liability, whether in contract, strict liability,
 // or tort (including negligence or otherwise) arising in any way out of
 // the use of this software, even if advised of the possibility of such damage.
 //
 //M*/

#ifndef __OPENCV_BITOPTS_HPP
#define __OPENCV_BITOPTS_HPP

#include "precomp_custom.hpp"

#ifdef _MSC_VER
# include <intrin.h>
# define popcnt __popcnt
# pragma warning( disable : 4267 )
#else
# define popcnt __builtin_popcount

#endif

/* LUT */
const int lookup[] =
{
    0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8
};

namespace cv
{
namespace line_descriptor
{
/*matching function */
inline int match( UINT8*P, UINT8*Q, int codelb )
{
    int i, output = 0;
    for( i = 0; i <= codelb - 16; i += 16 )
    {
        output += popcnt( *(UINT32*) (P+i) ^ *(UINT32*) (Q+i) ) +
                  popcnt( *(UINT32*) (P+i+4) ^ *(UINT32*) (Q+i+4) ) +
                  popcnt( *(UINT32*) (P+i+8) ^ *(UINT32*) (Q+i+8) ) +
                  popcnt( *(UINT32*) (P+i+12) ^ *(UINT32*) (Q+i+12) );
    }
    for( ; i < codelb; i++ )
        output += lookup[P[i] ^ Q[i]];
    return output;
}

/* splitting function (b <= 64) */
inline void split( UINT64 *chunks, UINT8 *code, int m, int mplus, int b )
{
  UINT64 temp = 0x0;
  int nbits = 0;
  int nbyte = 0;
  UINT64 mask = (b == 64) ? 0xFFFFFFFFFFFFFFFFull : ( ( UINT64_1 << b ) - UINT64_1 );

  for ( int i = 0; i < m; i++ )
  {
    while ( nbits < b )
    {
      temp |= ( (UINT64) code[nbyte++] << nbits );
      nbits += 8;
    }

    chunks[i] = temp & mask;
    temp = b == 64 ? 0x0 : temp >> b;
    nbits -= b;

    if( i == mplus - 1 )
    {
      b--; /* b <= 63 */
      mask = ( ( UINT64_1 << b ) - UINT64_1 );
    }
  }
}

/* generates the next binary code (in alphabetical order) with the
 same number of ones as the input x. Taken from
 http://www.geeksforgeeks.org/archives/10375 */
inline UINT64 next_set_of_n_elements( UINT64 x )
{
  UINT64 smallest, ripple, new_smallest;

  smallest = x & -(signed) x;
  ripple = x + smallest;
  new_smallest = x ^ ripple;
  new_smallest = new_smallest / smallest;
  new_smallest >>= 2;
  return ripple | new_smallest;
}

/* print code */
inline void print_code( UINT64 tmp, int b )
{
  for ( long long int j = ( b - 1 ); j >= 0; j-- )
  {
    printf( "%llu", (long long int) tmp / (UINT64) ( 1 << j ) );
    tmp = tmp - ( tmp / (UINT64) ( 1 << j ) ) * (UINT64) ( 1 << j );
  }

  printf( "\n" );
}

inline UINT64 choose( int n, int r )
{
  UINT64 nchooser = 1;
  for ( int k = 0; k < r; k++ )
  {
    nchooser *= n - k;
    nchooser /= k + 1;
  }

  return nchooser;
}
}
}

#endif
