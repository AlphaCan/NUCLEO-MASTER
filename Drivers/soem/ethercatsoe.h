/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

/** \file
 * \brief
 * Headerfile for ethercatsoe.c
 */

#ifndef _ethercatsoe_
#define _ethercatsoe_

#pragma anon_unions

#include <stdio.h>
#include <string.h>
#include "osal.h"
#include "oshw.h"
#include "ethercattype.h"
#include "ethercatbase.h"
#include "ethercatmain.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define NEX_SOE_DATASTATE_B   0x01
#define NEX_SOE_NAME_B        0x02
#define NEX_SOE_ATTRIBUTE_B   0x04
#define NEX_SOE_UNIT_B        0x08
#define NEX_SOE_MIN_B         0x10
#define NEX_SOE_MAX_B         0x20
#define NEX_SOE_VALUE_B       0x40
#define NEX_SOE_DEFAULT_B     0x80

#define NEX_SOE_MAXNAME       60
#define NEX_SOE_MAXMAPPING    64

#define NEX_IDN_MDTCONFIG     24
#define NEX_IDN_ATCONFIG      16

/** SoE name structure */

_Alignment typedef struct 
{
   /** current length in bytes of list */
   uint16     currentlength;
   /** maximum length in bytes of list */
   uint16     maxlength;
   char       name[NEX_SOE_MAXNAME];
} nex_SoEnamet;


/** SoE list structure */

_Alignment typedef struct 
{
   /** current length in bytes of list */
   uint16     currentlength;
   /** maximum length in bytes of list */
   uint16     maxlength;
_Alignment union
   {
      uint8   byte[8];
      uint16  word[4];
      uint32  dword[2];
      uint64  lword[1];
   };
} nex_SoElistt;


/** SoE IDN mapping structure */

_Alignment typedef struct 
{
   /** current length in bytes of list */
   uint16     currentlength;
   /** maximum length in bytes of list */
   uint16     maxlength;
   uint16     idn[NEX_SOE_MAXMAPPING];
} nex_SoEmappingt;


#define NEX_SOE_LENGTH_1         0x00
#define NEX_SOE_LENGTH_2         0x01
#define NEX_SOE_LENGTH_4         0x02
#define NEX_SOE_LENGTH_8         0x03
#define NEX_SOE_TYPE_BINARY      0x00
#define NEX_SOE_TYPE_UINT        0x01
#define NEX_SOE_TYPE_INT         0x02
#define NEX_SOE_TYPE_HEX         0x03
#define NEX_SOE_TYPE_STRING      0x04
#define NEX_SOE_TYPE_IDN         0x05
#define NEX_SOE_TYPE_FLOAT       0x06
#define NEX_SOE_TYPE_PARAMETER   0x07

/** SoE attribute structure */

_Alignment typedef struct 
{
   /** evaluation factor for display purposes */
   uint32     evafactor   :16;
   /** length of IDN element(s) */
   uint32     length      :2;
   /** IDN is list */
   uint32     list        :1;
   /** IDN is command */
   uint32     command     :1;
   /** datatype */
   uint32     datatype    :3;
   uint32     reserved1   :1;
   /** decimals to display if float datatype */
   uint32     decimals    :4;
   /** write protected in pre-op */
   uint32     wppreop     :1;
   /** write protected in safe-op */
   uint32     wpsafeop    :1;
   /** write protected in op */
   uint32     wpop        :1;
   uint32     reserved2   :1;
} nex_SoEattributet;


#ifdef NEX_VER1
int nex_SoEread(uint16 slave, uint8 driveNo, uint8 elementflags, uint16 idn, int *psize, void *p, int timeout);
int nex_SoEwrite(uint16 slave, uint8 driveNo, uint8 elementflags, uint16 idn, int psize, void *p, int timeout);
int nex_readIDNmap(uint16 slave, int *Osize, int *Isize);
#endif

int nexx_SoEread(nexx_contextt *context, uint16 slave, uint8 driveNo, uint8 elementflags, uint16 idn, int *psize, void *p, int timeout);
int nexx_SoEwrite(nexx_contextt *context, uint16 slave, uint8 driveNo, uint8 elementflags, uint16 idn, int psize, void *p, int timeout);
int nexx_readIDNmap(nexx_contextt *context, uint16 slave, int *Osize, int *Isize);

#ifdef __cplusplus
}
#endif

#endif

