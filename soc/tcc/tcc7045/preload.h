/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright 2024 Hounjoung Rim <hounjoung@tsnlab.com>
 */

#ifndef MCU_BSP_PRELOAD_HEADER
#define MCU_BSP_PRELOAD_HEADER

/***************************************************************************************************
*
*
*
*
* @param    none
*
* @return   none
*
* Notes
*
***************************************************************************************************
*/
void PRELOAD_loadOnRam
(
    void
);


/*
***************************************************************************************************
*
*
*
*
* @param
*
* @return
*
* Notes
*
***************************************************************************************************
*/

#if 0
void PRELOAD_JOB
(
    void
)__attribute__ ((section (".codeonsram")));
#endif

#endif  // MCU_BSP_PRELOAD_HEADER

