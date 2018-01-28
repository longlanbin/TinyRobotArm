#ifndef __NEX_TYPE_H
#define __NEX_TYPE_H

/*****************************************************************
Copyright (c) NEXCOM International Co., Ltd. All Rights Reserved. 
variable type define. For cross compile
*******************************************************************/
typedef int               BOOL_T;  // 0:False, else:True
typedef unsigned char     U8_T;    // 0 ~ 255
typedef unsigned short    U16_T;   // 0 ~ 65535
typedef unsigned int      U32_T;   // 0 ~ 4294967295
typedef unsigned __int64  U64_T;   // 0 ~ 18446744073709551615

typedef char              I8_T;    // -128 ~ 127
typedef short             I16_T;   // -32768 ~ 32767
typedef int               I32_T;   // -2147483648 ~ 2147483647
typedef __int64           I64_T;   // -9223372036854775808 ~ 9223372036854775807

typedef float             F32_T;   // Single precision floating point
typedef double            F64_T;   // double precision floating point

typedef I32_T             RTN_ERR; // Return code. Return error code. see errore_code define file.
   

#define I32_T_MAX  ( 2147483647)
#define I32_T_MIN  (-2147483648)

#endif

