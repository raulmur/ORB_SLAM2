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

#ifndef __OPENCV_BITARRAY_HPP
#define __OPENCV_BITARRAY_HPP

#ifdef _MSC_VER
#pragma warning( disable : 4267 )
#endif

#include "types_custom.hpp"
#include <stdio.h>
#include <math.h>
#include <string.h>

/* class defining a sequence of bits */
class bitarray
{

 public:
  /* pointer to bits sequence and sequence's length */
  UINT32 *arr;
  UINT32 length;

  /* constructor setting default values */
  bitarray()
  {
    arr = NULL;
    length = 0;
  }

  /* constructor setting sequence's length */
  bitarray( UINT64 _bits )
  {
    init( _bits );
  }

  /* initializer of private fields */
  void init( UINT64 _bits )
  {
    length = (UINT32) ceil( _bits / 32.00 );
    arr = new UINT32[length];
    erase();
  }

  /* destructor */
  ~bitarray()
  {
    if( arr )
      delete[] arr;
  }

  inline void flip( UINT64 index )
  {
    arr[index >> 5] ^= ( (UINT32) 0x01 ) << ( index % 32 );
  }

  inline void set( UINT64 index )
  {
    arr[index >> 5] |= ( (UINT32) 0x01 ) << ( index % 32 );
  }

  inline UINT8 get( UINT64 index )
  {
    return ( arr[index >> 5] & ( ( (UINT32) 0x01 ) << ( index % 32 ) ) ) != 0;
  }

  /* reserve menory for an UINT32 */
  inline void erase()
  {
    memset( arr, 0, sizeof(UINT32) * length );
  }

};

#endif
