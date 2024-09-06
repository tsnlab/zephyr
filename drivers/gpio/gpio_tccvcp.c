/*
 * Copyright (c) 2024 Hounjoung Rim <hounjoung@tsnlab.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT tcc_tccvcp_gpio

#include <zephyr/arch/common/sys_bitops.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>

/*
******************************************************************************
*                             GLOBAL DEFINES
******************************************************************************
*/

#define GPIO_D(fmt,args...)
#define GPIO_E(fmt,args...)

typedef unsigned char                   boolean;    /* for use with TRUE/FALSE        */

#ifndef FALSE
#define FALSE                           (0U)
#endif

#ifndef TRUE
#define TRUE                            (1U)
#endif

#define SAL_WriteReg(v, a)              (*((volatile uint32_t *)(a)) = (uint32_t)(v))
#define SAL_ReadReg(a)                  (*(volatile uint32_t *)(a))

#define SAL_ArraySize(x)                (sizeof(x)/sizeof((x)[0]))

/*
 * Return Codes
 */
typedef enum SALRetCode
{
    SAL_RET_SUCCESS                     = 0,
    SAL_RET_FAILED                      = 1

} SALRetCode_t;


#define GIC_PPI_START                   (16UL)
#define GIC_SPI_START                   (32UL)

#define GIC_EXT0                        (GIC_SPI_START + 71UL)
#define GIC_EXT9                        (GIC_SPI_START + 80UL)

#define GIC_EINT_START_INT              (GIC_EXT0)
#define GIC_EINT_END_INT                (GIC_EXT9)


#define MCU_BSP_PMIO_BASE               (0xA0F28800UL)
#define MCU_BSP_GPIO_BASE               (0xA0F22000UL)

/*
 * gpio cfg structures
 *   [31:14]: reserved
 *   [11:10]: input buffer
 *   [9]    : direction
 *   [8:6]  : driver strength (0~3)
 *   [5:4]  : pull up/down
 *   [3:0]  : function selection (0~15)
 */

#define GPIO_INPUTBUF_SHIFT             (10)
#define GPIO_INPUTBUF_MASK              (0x3UL)
#define GPIO_INPUTBUF_EN                ((2UL | 1UL) << (uint32_t)GPIO_INPUTBUF_SHIFT)
#define GPIO_INPUTBUF_DIS               ((2UL | 0UL) << (uint32_t)GPIO_INPUTBUF_SHIFT)

#define GPIO_OUTPUT_SHIFT               (9)
#define GPIO_OUTPUT_TCC                 (1UL << (uint32_t)GPIO_OUTPUT_SHIFT)
#define GPIO_INPUT_TCC                  (0UL << (uint32_t)GPIO_OUTPUT_SHIFT)

#define GPIO_DS_SHIFT                   (6)
#define GPIO_DS_MASK                    (0x7UL)
#define GPIO_DS(x)                      ((((x) & (uint32_t)GPIO_DS_MASK) | 0x4UL) << (uint32_t)GPIO_DS_SHIFT)

#define GPIO_PULL_SHIFT                 (4)
#define GPIO_PULL_MASK                  (0x3UL)
#define GPIO_NOPULL                     (0UL << (uint32_t)GPIO_PULL_SHIFT)
#define GPIO_PULLUP                     (1UL << (uint32_t)GPIO_PULL_SHIFT)
#define GPIO_PULLDN                     (2UL << (uint32_t)GPIO_PULL_SHIFT)

#define GPIO_FUNC_MASK                  (0xFUL)
#define GPIO_FUNC(x)                    ((x) & (uint32_t)GPIO_FUNC_MASK)

#define GPIO_MFIO_CFG_CH_SEL0           (0)
#define GPIO_MFIO_CFG_PERI_SEL0         (4)
#define GPIO_MFIO_CFG_CH_SEL1           (8)
#define GPIO_MFIO_CFG_PERI_SEL1         (12)
#define GPIO_MFIO_CFG_CH_SEL2           (16)
#define GPIO_MFIO_CFG_PERI_SEL2         (20)
#define GPIO_MFIO_DISABLE               (0)
#define GPIO_MFIO_SPI2                  (1)
#define GPIO_MFIO_UART3                 (2)
#define GPIO_MFIO_I2C3                  (3)
#define GPIO_MFIO_SPI3                  (1)
#define GPIO_MFIO_UART4                 (2)
#define GPIO_MFIO_I2C4                  (3)
#define GPIO_MFIO_SPI4                  (1)
#define GPIO_MFIO_UART5                 (2)
#define GPIO_MFIO_I2C5                  (3)
#define GPIO_MFIO_CH0                   (0)
#define GPIO_MFIO_CH1                   (1)
#define GPIO_MFIO_CH2                   (2)
#define GPIO_MFIO_CH3                   (3)
#define GPIO_PERICH_SEL_UARTSEL_0       (0)
#define GPIO_PERICH_SEL_UARTSEL_1       (1)
#define GPIO_PERICH_SEL_UARTSEL_2       (2)
#define GPIO_PERICH_SEL_I2CSEL_0        (3)
#define GPIO_PERICH_SEL_I2CSEL_1        (4)
#define GPIO_PERICH_SEL_I2CSEL_2        (5)
#define GPIO_PERICH_SEL_SPISEL_0        (6)
#define GPIO_PERICH_SEL_SPISEL_1        (7)
#define GPIO_PERICH_SEL_I2SSEL_0        (8)
#define GPIO_PERICH_SEL_PWMSEL_0        (10)
#define GPIO_PERICH_SEL_PWMSEL_1        (12)
#define GPIO_PERICH_SEL_PWMSEL_2        (14)
#define GPIO_PERICH_SEL_PWMSEL_3        (16)
#define GPIO_PERICH_SEL_PWMSEL_4        (18)
#define GPIO_PERICH_SEL_PWMSEL_5        (20)
#define GPIO_PERICH_SEL_PWMSEL_6        (22)
#define GPIO_PERICH_SEL_PWMSEL_7        (24)
#define GPIO_PERICH_SEL_PWMSEL_8        (26)
#define GPIO_PERICH_CH0                 (0)
#define GPIO_PERICH_CH1                 (1)
#define GPIO_PERICH_CH2                 (2)
#define GPIO_PERICH_CH3                 (3)

/*
 * gpio port & pin structures
 *   [31:10]: reserved
 *   [9:5] : port (A,B,C,...)
 *   [4:0] : pin number (0~31)
 */

#define GPIO_PIN_MASK                   (0x1FUL)
#define GPIO_PIN_NUM_MASK               (0x3FUL) // original 1FUL , avoid code sonar warning


#define GPIO_PORT_SHIFT                 (5)
#define GPIO_PORT_MASK                  ((uint32_t)0x1F << (uint32_t)GPIO_PORT_SHIFT)

//(n<<GPIO_PORT_SHIFT)                  n = ofset/0x40
#define GPIO_PORT_A                     ((uint32_t)0 << (uint32_t)GPIO_PORT_SHIFT)        // offset: 0x000
#define GPIO_PORT_B                     ((uint32_t)1 << (uint32_t)GPIO_PORT_SHIFT)        // offset: 0x040
#define GPIO_PORT_C                     ((uint32_t)2 << (uint32_t)GPIO_PORT_SHIFT)        // offset: 0x080
#define GPIO_PORT_K                     ((uint32_t)3<<(uint32_t)GPIO_PORT_SHIFT)        // offset: 0x0c0

#define GPIO_GPA(x)                     (GPIO_PORT_A | ((x) & (uint32_t)0x1F))
#define GPIO_GPB(x)                     (GPIO_PORT_B | ((x) & (uint32_t)0x1F))
#define GPIO_GPC(x)                     (GPIO_PORT_C | ((x) & (uint32_t)0x1F))
#define GPIO_GPK(x)                     (GPIO_PORT_K | ((x) & (uint32_t)0x1F))


#define GPIO_GP_MAX                     GPIO_GPK((uint32_t)0x1f)


/*
******************************************************************************
*                             LOCAL DEFINES
******************************************************************************
*/

#define GPIO_PMGPIO_BASE                (MCU_BSP_PMIO_BASE)

#define GPIO_REG_BASE(x)                (MCU_BSP_GPIO_BASE + ((((x) & GPIO_PORT_MASK) >> (uint32_t)GPIO_PORT_SHIFT) * 0x40UL))


#define GPIO_IS_GPIOK(x)                (boolean)((((x) & GPIO_PORT_MASK) == GPIO_PORT_K) ? 1 : 0)

#define GPIO_REG_DATA(x)                (GPIO_REG_BASE(x) + 0x00UL)
#define GPIO_REG_OUTEN(x)               (GPIO_REG_BASE(x) + 0x04UL)
#define GPIO_REG_DATA_OR(x)             (GPIO_REG_BASE(x) + 0x08UL)
#define GPIO_REG_DATA_BIC(x)            (GPIO_REG_BASE(x) + 0x0CUL)
#define GPIO_REG_PULLEN(x)              (GPIO_IS_GPIOK(x) ? (GPIO_PMGPIO_BASE + 0x10UL) : (GPIO_REG_BASE(x) + 0x1CUL))
#define GPIO_REG_PULLSEL(x)             (GPIO_IS_GPIOK(x) ? (GPIO_PMGPIO_BASE + 0x14UL) : (GPIO_REG_BASE(x) + 0x20UL))
#define GPIO_REG_CD(x,pin)              ((GPIO_IS_GPIOK(x) ? (GPIO_PMGPIO_BASE + 0x18UL) : (GPIO_REG_BASE(x) + 0x14UL)) + (0x4UL * ((pin) / (uint32_t)16)))
#define GPIO_REG_IEN(x)                 (GPIO_IS_GPIOK(x) ? (GPIO_PMGPIO_BASE + 0x0CUL) :( GPIO_REG_BASE(x) + 0x24UL))
//#define GPIO_REG_IS(x)                  (GPIO_REG_BASE(x) + 0x28UL)
//#define GPIO_REG_SR(x)                  (GPIO_REG_BASE(x) + 0x2CUL)
#define GPIO_REG_FN(x,pin)              ((GPIO_REG_BASE(x) + 0x30UL) + (0x4UL * ((pin)/(uint32_t)8)))
#define GPIO_MFIO_CFG                   (MCU_BSP_GPIO_BASE + (0x2B4UL))
#define GPIO_PERICH_SEL                 (MCU_BSP_GPIO_BASE + (0x2B8UL))

#define GPIO_PMGPIO_SEL                 (GPIO_PMGPIO_BASE + 0x8UL)

#define GPIO_LIST_NUM                   (6)

/*
******************************************************************************
*                             VARIABLES
******************************************************************************
*/

int mfio_ch_cfg_flag[3] = { 0, };


/*
******************************************************************************
*                             FUNCTION PROTOTYPES
******************************************************************************
*/


#if 1 // POOKY 20240906 gpio/tcc70xx/gpio.c

static void GPIO_SetRegister(uint32_t addr, uint32_t bit, uint32_t enable)
{
    uint32_t base_val;
    uint32_t set_val;

    base_val    = SAL_ReadReg(addr);
    set_val     = 0UL;

    if(enable == 1UL)
    {
        set_val     = (base_val | bit);
    }
    else if(enable == 0UL)
    {
        set_val     = (base_val & ~bit);
    }
    else
    {
        // Do nothing.
    }

    SAL_WriteReg(set_val, addr);
}

/*
***************************************************************************************************
*                                          GPIO_Config
*
* @param    [In] uiPort     :   Gpio port index, GPIO_GPX(X)
* @param    [In] uiConfig   :   Gpio configuration options
* @return   SAL_RET_SUCCESS or SAL_RET_FAILED
*
* Notes
*
***************************************************************************************************
*/

SALRetCode_t GPIO_Config
(
    uint32_t                              uiPort,
    uint32_t                              uiConfig
)
{
    uint32_t pin;
    uint32_t bit;
    uint32_t func;
    uint32_t pull;
    uint32_t ds;
    uint32_t ien;
    uint32_t base_val;
    uint32_t comp_val;
    uint32_t set_val;
    uint32_t reg_fn;
    uint32_t pullen_addr;
    uint32_t pullsel_addr;
    uint32_t cd_addr;
    uint32_t outen_addr;
    uint32_t ien_addr;
	SALRetCode_t ret;

    ret     = SAL_RET_SUCCESS;
    pin     = uiPort & (uint32_t)GPIO_PIN_MASK;
    bit     = (uint32_t)1 << pin;
    func    = uiConfig & (uint32_t)GPIO_FUNC_MASK;
    pull    = uiConfig & ((uint32_t)GPIO_PULL_MASK << (uint32_t)GPIO_PULL_SHIFT);
    ds      = uiConfig & ((uint32_t)GPIO_DS_MASK << (uint32_t)GPIO_DS_SHIFT);
    ien     = uiConfig & ((uint32_t)GPIO_INPUTBUF_MASK << (uint32_t)GPIO_INPUTBUF_SHIFT);


    /* function */
    reg_fn      = GPIO_REG_FN(uiPort,pin);
    base_val    = SAL_ReadReg(reg_fn) & (~((uint32_t)0xF<<((pin%(uint32_t)8)*(uint32_t)4)));
    set_val     = base_val | (func<<((pin%(uint32_t)8)*(uint32_t)4));
    SAL_WriteReg(set_val, reg_fn);
    /* configuration check */
    comp_val    = SAL_ReadReg(reg_fn);

    if(comp_val != set_val)
    {
        ret = SAL_RET_FAILED;
    }
    else
    {
	    /* pull-up/down */
	    if (pull == GPIO_PULLUP)
	    {
	        if(GPIO_IS_GPIOK(uiPort))
	        {
	            pullen_addr = (GPIO_PMGPIO_BASE + 0x10UL);
	        }
	        else
	        {
	            pullen_addr = (GPIO_REG_BASE(uiPort) + 0x1CUL);
	        }

	        GPIO_SetRegister(pullen_addr, bit, (uint32_t)TRUE);

	        if(GPIO_IS_GPIOK(uiPort))
	        {
	            pullsel_addr = (GPIO_PMGPIO_BASE + 0x14UL);
	        }
	        else
	        {
	            pullsel_addr = (GPIO_REG_BASE(uiPort) + 0x20UL);
	        }

            GPIO_SetRegister(pullsel_addr, bit, (uint32_t)TRUE);
        }
        else if (pull == GPIO_PULLDN)
        {
	        if(GPIO_IS_GPIOK(uiPort))
            {
                pullen_addr = (GPIO_PMGPIO_BASE + 0x10UL);
            }
            else
            {
                pullen_addr = (GPIO_REG_BASE(uiPort) + 0x1CUL);
            }

            GPIO_SetRegister(pullen_addr, bit, (uint32_t)TRUE);

	        if(GPIO_IS_GPIOK(uiPort))
	        {
	            pullsel_addr = (GPIO_PMGPIO_BASE + 0x14UL);
	        }
	        else
	        {
	            pullsel_addr = (GPIO_REG_BASE(uiPort) + 0x20UL);
	        }

            GPIO_SetRegister(pullsel_addr, bit, (uint32_t)FALSE);
        }
        else
        {
        	if(GPIO_IS_GPIOK(uiPort))
            {
                pullen_addr = (GPIO_PMGPIO_BASE + 0x10UL);
            }
            else
            {
                pullen_addr = (GPIO_REG_BASE(uiPort) + 0x1CUL);
            }

            GPIO_SetRegister(pullen_addr, bit, (uint32_t)FALSE);
        }

	    /* drive strength */
	    if (ds != 0UL)
	    {
	        if(GPIO_IS_GPIOK(uiPort))
	        {
	            cd_addr = (GPIO_PMGPIO_BASE + 0x18UL) + (0x4UL * ((pin) / (uint32_t)16));
	        }
	        else
	        {
	            cd_addr = (GPIO_REG_BASE(uiPort) + 0x14UL) + (0x4UL * ((pin) / (uint32_t)16));
	        }

	        ds          = ds >> (uint32_t)GPIO_DS_SHIFT;
	        base_val    = SAL_ReadReg(cd_addr) & ~((uint32_t)3 << ((pin % (uint32_t)16) * (uint32_t)2));
	        set_val     = base_val | ((ds & (uint32_t)0x3) << ((pin % (uint32_t)16) * (uint32_t)2));
            SAL_WriteReg(set_val, cd_addr);
        }

        /* direction */
        if ((uiConfig&GPIO_OUTPUT_TCC) != 0UL)
        {
            outen_addr  = GPIO_REG_OUTEN(uiPort);

            GPIO_SetRegister(outen_addr, bit, (uint32_t)TRUE);
        }
        else
        {
            outen_addr  = GPIO_REG_OUTEN(uiPort);

            GPIO_SetRegister(outen_addr, bit, (uint32_t)FALSE);
        }

        /* input buffer enable */
        if (ien == GPIO_INPUTBUF_EN)
        {
    	    if(GPIO_IS_GPIOK(uiPort))
            {
                ien_addr = (GPIO_PMGPIO_BASE + 0x0CUL);
            }
            else
            {
                ien_addr = (GPIO_REG_BASE(uiPort) + 0x24UL);
            }

            GPIO_SetRegister(ien_addr, bit, (uint32_t)TRUE);
        }
        else if (ien == GPIO_INPUTBUF_DIS)
        {
	        if(GPIO_IS_GPIOK(uiPort))
            {
                ien_addr = (GPIO_PMGPIO_BASE + 0x0CUL);
            }
            else
            {
                ien_addr = (GPIO_REG_BASE(uiPort) + 0x24UL);
            }

            GPIO_SetRegister(ien_addr, bit, (uint32_t)FALSE);
        }
        else //QAC
        {
            ; // no statement
        }

    }

    return ret;
}

/*
***************************************************************************************************
*                                          GPIO_Get
*
* @param    [In] uiPort     :   Gpio port index, GPIO_GPX(X)
* @return   Gpio data value (0:Low or 1:High)
*
* Notes
*
***************************************************************************************************
*/

uint8_t GPIO_Get
(
    uint32_t                              uiPort
)
{
    uint32_t reg_data;
    uint32_t value;
    uint8_t  ret;

    reg_data    = GPIO_REG_DATA(uiPort);
    value       = (SAL_ReadReg(reg_data) & ((uint32_t)1 << (uiPort & GPIO_PIN_MASK)));

    if (value != 0UL)
    {
        ret = (uint8_t)1;
    }
    else
    {
        ret = (uint8_t)0;
    }

    return ret;
}


/*
***************************************************************************************************
*                                          GPIO_Set
* @param    [In] uiPort     :   Gpio port index, GPIO_GPX(X)
* @param    [In] uiData     :   Gpio data value, (0 or 1)
* @return   SAL_RET_SUCCESS or SAL_RET_FAILED
*
* Notes
*
***************************************************************************************************
*/

SALRetCode_t GPIO_Set
(
    uint32_t                              uiPort,
    uint32_t                              uiData
)
{
    uint32_t bit;
    uint32_t data_or;
    uint32_t data_bic;
    SALRetCode_t ret;

    ret = SAL_RET_SUCCESS;

    bit = (uint32_t)1 << (uiPort & GPIO_PIN_MASK);

    if (uiData > 1UL)
    {
        ret = SAL_RET_FAILED;
    }
    else
    {
        /* set data */
        if (uiData!=0UL)
        {
            data_or = GPIO_REG_DATA_OR(uiPort);
            SAL_WriteReg(bit, data_or);
        }
        else
        {
            data_bic = GPIO_REG_DATA_BIC(uiPort);
            SAL_WriteReg(bit, data_bic);
        }
    }

    return ret;
}


/*
***************************************************************************************************
*                                          GPIO_ToNum
*
* @param    [In] uiPort     :   Gpio port index, GPIO_GPX(X)
* @return
*
* Notes
*
***************************************************************************************************
*/
uint32_t GPIO_ToNum
(
    uint32_t                              uiPort
)
{
    uint32_t pin;
    uint32_t ret;
    uint32_t port;

    pin = uiPort & GPIO_PIN_NUM_MASK;
    ret = 0UL;
    port = uiPort & GPIO_PORT_MASK;

    switch (port)
    {
        case GPIO_PORT_A:
        {
            ret = pin;
            break;
        }

        case GPIO_PORT_B:
        {
            ret = (0x20UL + pin);
            break;
        }
        case GPIO_PORT_C:
        {
            ret = (0x3dUL + pin);
            break;
        }
        case GPIO_PORT_K:
        {
        	ret = (0x86UL + pin);

            break;
        }

        default:
        {
            GPIO_E("\n Can't find GPIO Port \n");
            break;
        }
    }

    return ret;
}

/*
***************************************************************************************************
*                                          GPIO_PerichSel
*
* @param    [In] uiPerichSel    : Gpio peri select index
* @param    [In] uiCh           : Gpio peri select channel index
* @return   SAL_RET_SUCCESS or SAL_RET_FAILED
*
* Notes
*
***************************************************************************************************
*/
SALRetCode_t GPIO_PerichSel
(
    uint32_t                              uiPerichSel,
    uint32_t                              uiCh
)
{
    uint32_t peri_sel_addr;
    uint32_t clear_bit;
    uint32_t set_bit;
    uint32_t base_val;
    uint32_t comp_val;

    peri_sel_addr = GPIO_PERICH_SEL;
    base_val = SAL_ReadReg(peri_sel_addr);

    if(uiPerichSel < GPIO_PERICH_SEL_I2SSEL_0)
    {
        if(uiCh < 2)
        {
            //clear bit
            clear_bit = base_val & ~((0x1UL) << uiPerichSel);
            SAL_WriteReg(clear_bit, peri_sel_addr);
            //set bit
            base_val = SAL_ReadReg(peri_sel_addr);
            set_bit = base_val | ((uiCh & 0x1UL) << uiPerichSel);
            SAL_WriteReg(set_bit,peri_sel_addr);
            comp_val    = SAL_ReadReg(peri_sel_addr);

            if(comp_val != set_bit)
            {
                GPIO_E("GPIO PerichSel 1bit error \n");
                return SAL_RET_FAILED;
            }
        }
        else
        {
            GPIO_E("GPIO PerichSel ch 1bit error \n");
            return SAL_RET_FAILED;
        }
    }
    else
    {
        if(uiCh < 4)
        {
            //clear bit
            clear_bit = base_val & ~((0x3UL) << uiPerichSel);
            SAL_WriteReg(clear_bit, peri_sel_addr);
            //set bit
            base_val = SAL_ReadReg(peri_sel_addr);
            set_bit = base_val | ((uiCh & 0x3UL) << uiPerichSel);
            SAL_WriteReg(set_bit,peri_sel_addr);
            comp_val    = SAL_ReadReg(peri_sel_addr);

            if(comp_val != set_bit)
            {
                GPIO_E("GPIO PerichSel 2bit error \n");
                return SAL_RET_FAILED;
            }
        }
        else
        {
            GPIO_E("GPIO PerichSel ch 2bit error \n");
            return SAL_RET_FAILED;
        }
    }

    return SAL_RET_SUCCESS;
}

/*
***************************************************************************************************
*                                          GPIO_MfioCfg
*
* @param    [In] uiPeriSel      :   MFIO peri select index
* @param    [In] uiPeriType     :   MFIO peri select type (Disable/GPSB/UART/I2C)
* @param    [In] uiChSel        :   MFIO channel select index
* @param    [In] uiChNum        :   MFIO channel select value
* @return   SAL_RET_SUCCESS or SAL_RET_FAILED
*
* Notes
*
***************************************************************************************************
*/
SALRetCode_t GPIO_MfioCfg
(
    uint32_t                              uiPeriSel,
    uint32_t                              uiPeriType,
    uint32_t                              uiChSel,
    uint32_t                              uiChNum
)
{
    uint32_t base_val;
    uint32_t set_val;
    uint32_t clear_bit;
    uint32_t comp_val;

    if(uiPeriSel == GPIO_MFIO_CFG_PERI_SEL0)
    {
        if(uiChSel == GPIO_MFIO_CFG_CH_SEL0)
        {
            if(mfio_ch_cfg_flag[0] == 0)
            {
                //clear bit
                base_val = SAL_ReadReg(GPIO_MFIO_CFG);
                clear_bit = base_val & ~((0x3UL) << (uint32_t)GPIO_MFIO_CFG_CH_SEL0)
                                     & ~((0x3UL) << (uint32_t)GPIO_MFIO_CFG_PERI_SEL0);
                SAL_WriteReg(clear_bit, GPIO_MFIO_CFG);

                base_val = SAL_ReadReg(GPIO_MFIO_CFG);
                set_val = base_val  | ((uiChNum & 0x3UL) << (uint32_t)GPIO_MFIO_CFG_CH_SEL0)
                                    | ((uiPeriType & 0x3UL) << (uint32_t)GPIO_MFIO_CFG_PERI_SEL0);
                SAL_WriteReg(set_val, GPIO_MFIO_CFG);
                comp_val    = SAL_ReadReg(GPIO_MFIO_CFG);

                if(comp_val != set_val)
                {
                    return SAL_RET_FAILED;
                }
                mfio_ch_cfg_flag[0] = 1;
            }
            else
            {
                GPIO_E("MFID 0 ch%d already set!!\n",uiChNum);
                return SAL_RET_FAILED;
            }

        }
        else
        {
            GPIO_E("match perisel0, chsel0 \n");
            return SAL_RET_FAILED;
        }
    }
    else if(uiPeriSel == GPIO_MFIO_CFG_PERI_SEL1)
    {
        if(uiChSel == GPIO_MFIO_CFG_CH_SEL1)
        {
            if(mfio_ch_cfg_flag[1] == 0)
            {
                //clear bit
                base_val = SAL_ReadReg(GPIO_MFIO_CFG);
                clear_bit = base_val & ~((0x3UL) << (uint32_t)GPIO_MFIO_CFG_CH_SEL1)
                                     & ~((0x3UL) << (uint32_t)GPIO_MFIO_CFG_PERI_SEL1);
                SAL_WriteReg(clear_bit, GPIO_MFIO_CFG);

                base_val = SAL_ReadReg(GPIO_MFIO_CFG);
                set_val = base_val  | ((uiChNum & 0x3UL) << (uint32_t)GPIO_MFIO_CFG_CH_SEL1)
                                    | ((uiPeriType & 0x3UL) << (uint32_t)GPIO_MFIO_CFG_PERI_SEL1);
                SAL_WriteReg(set_val, GPIO_MFIO_CFG);
                comp_val    = SAL_ReadReg(GPIO_MFIO_CFG);

                if(comp_val != set_val)
                {
                    return SAL_RET_FAILED;
                }
                mfio_ch_cfg_flag[1] = 1;
            }
            else
            {
                GPIO_E("MFID 01 ch%d already set!!\n",uiChNum);
                return SAL_RET_FAILED;
            }
        }
        else
        {
            GPIO_E("match perisel1, chsel1 \n");
            return SAL_RET_FAILED;
        }

    }
    else if(uiPeriSel == GPIO_MFIO_CFG_PERI_SEL2)
    {
        if(uiChSel == GPIO_MFIO_CFG_CH_SEL2)
        {
            if(mfio_ch_cfg_flag[2] == 0)
            {
                //clear bit
                base_val = SAL_ReadReg(GPIO_MFIO_CFG);
                clear_bit = base_val & ~((0x3UL) << (uint32_t)GPIO_MFIO_CFG_CH_SEL2)
                                     & ~((0x3UL) << (uint32_t)GPIO_MFIO_CFG_PERI_SEL2);
                SAL_WriteReg(clear_bit, GPIO_MFIO_CFG);

                base_val = SAL_ReadReg(GPIO_MFIO_CFG);
                set_val = base_val  | ((uiChNum & 0x3UL) << (uint32_t)GPIO_MFIO_CFG_CH_SEL2)
                                    | ((uiPeriType & 0x3UL) << (uint32_t)GPIO_MFIO_CFG_PERI_SEL2);
                SAL_WriteReg(set_val, GPIO_MFIO_CFG);
                comp_val    = SAL_ReadReg(GPIO_MFIO_CFG);

                if(comp_val != set_val)
                {
                    return SAL_RET_FAILED;
                }
                mfio_ch_cfg_flag[2] = 1;
            }
            else
            {
                GPIO_E("MFID 02 ch%d already set!!\n",uiChNum);
                return SAL_RET_FAILED;
            }
        }
        else
        {
            GPIO_E("match perisel2, chsel2 \n");
            return SAL_RET_FAILED;
        }
    }
    else
    {
        GPIO_E("check perisel \n");
        return SAL_RET_FAILED;
    }

    return SAL_RET_SUCCESS;
}

/**************************************************************************************************
*                                       GPIO_IntExtSet
*
* Configure external interrupt source select.
*
* @param    [In] uiIntId        : Index of Interrupt Source id.
* @param    [In] uiGpio         : Gpio port index, GPIO_GPX(X)
* @return   SAL_RET_SUCCESS or SAL_RET_FAILED
*
* Notes
*
**************************************************************************************************/
SALRetCode_t GPIO_IntExtSet
(
    uint32_t                              uiIntId,
    uint32_t                              uiGpio
)
{
    SALRetCode_t  ucRet;
    uint32_t        uiEintSel;
    uint32_t        uiBitField;
    uint32_t        uiExtIntGpioIdx;
    uint32_t        uiEIntSel;
    uint32_t        uiEIntSelMask;

    static const uint32_t ExtIntr[] =
    {
        GPIO_GPA(0UL),   GPIO_GPA(1UL),   GPIO_GPA(2UL),   GPIO_GPA(3UL),   GPIO_GPA(4UL),   GPIO_GPA(5UL),   GPIO_GPA(6UL),   GPIO_GPA(7UL),
        GPIO_GPA(8UL),   GPIO_GPA(9UL),   GPIO_GPA(10UL),  GPIO_GPA(11UL),  GPIO_GPA(12UL),  GPIO_GPA(13UL),  GPIO_GPA(14UL),  GPIO_GPA(15UL),
        GPIO_GPA(16UL),  GPIO_GPA(17UL),  GPIO_GPA(18UL),  GPIO_GPA(18UL),  GPIO_GPA(20UL),  GPIO_GPA(21UL),  GPIO_GPA(22UL),  GPIO_GPA(23UL),
        GPIO_GPA(24UL),  GPIO_GPA(25UL),  GPIO_GPA(26UL),  GPIO_GPA(27UL),  GPIO_GPA(28UL),  GPIO_GPA(29UL),  GPIO_GPA(30UL),

        GPIO_GPB(0UL),   GPIO_GPB(1UL),   GPIO_GPB(2UL),   GPIO_GPB(3UL),   GPIO_GPB(4UL),   GPIO_GPB(5UL),   GPIO_GPB(6UL),   GPIO_GPB(7UL),
        GPIO_GPB(8UL),   GPIO_GPB(9UL),   GPIO_GPB(10UL),  GPIO_GPB(11UL),  GPIO_GPB(12UL),  GPIO_GPB(13UL),  GPIO_GPB(14UL),  GPIO_GPB(15UL),
        GPIO_GPB(16UL),  GPIO_GPB(17UL),  GPIO_GPB(18UL),  GPIO_GPB(19UL),  GPIO_GPB(20UL),  GPIO_GPB(21UL),  GPIO_GPB(22UL),  GPIO_GPB(23UL),
        GPIO_GPB(24UL),  GPIO_GPB(25UL),  GPIO_GPB(26UL),  GPIO_GPB(27UL),  GPIO_GPB(28UL),

        GPIO_GPC(0UL),   GPIO_GPC(1UL),   GPIO_GPC(2UL),   GPIO_GPC(3UL) ,  GPIO_GPC(4UL),   GPIO_GPC(5UL),   GPIO_GPC(6UL),   GPIO_GPC(7UL),
        GPIO_GPC(8UL),   GPIO_GPC(9UL),   GPIO_GPC(10UL),  GPIO_GPC(11UL),  GPIO_GPC(12UL),  GPIO_GPC(13UL),  GPIO_GPC(14UL),  GPIO_GPC(15UL),
        GPIO_GPC(16UL),  GPIO_GPC(17UL),  GPIO_GPC(18UL),  GPIO_GPC(19UL),  GPIO_GPC(20UL),  GPIO_GPC(21UL),  GPIO_GPC(22UL),  GPIO_GPC(23UL),
        GPIO_GPC(24UL),  GPIO_GPC(25UL),  GPIO_GPC(26UL),  GPIO_GPC(27UL),

        GPIO_GPK(0UL),   GPIO_GPK(1UL),   GPIO_GPK(2UL),   GPIO_GPK(3UL) ,  GPIO_GPK(4UL),   GPIO_GPK(5UL),   GPIO_GPK(6UL),   GPIO_GPK(7UL),
        GPIO_GPK(8UL),   GPIO_GPK(9UL),   GPIO_GPK(10UL),  GPIO_GPK(11UL),  GPIO_GPK(12UL),  GPIO_GPK(13UL),  GPIO_GPK(14UL),  GPIO_GPK(15UL),
        GPIO_GPK(16UL),  GPIO_GPK(17UL),
    };

    ucRet           = (SALRetCode_t)SAL_RET_SUCCESS;
    uiEintSel       = ((MCU_BSP_GPIO_BASE + 0x280UL) + (4UL*((uiIntId-(uint32_t)GIC_EXT0)/4UL))); /* EINT_SEL0,2,3 */
    uiBitField      = 0;
    uiExtIntGpioIdx     = 0;
    uiEIntSel       = 0;
    uiEIntSelMask   = 0;

    if ((uiIntId < (uint32_t)GIC_EINT_START_INT) || (uiIntId > (uint32_t)GIC_EINT_END_INT))
    {
        ucRet = (SALRetCode_t)SAL_RET_FAILED;
    }
    else
    {

        for ( ; uiExtIntGpioIdx < SAL_ArraySize(ExtIntr) ; uiExtIntGpioIdx++)
        {
            if (ExtIntr[uiExtIntGpioIdx] == uiGpio)
            {
                break;
            }
        }

        if (uiExtIntGpioIdx >= SAL_ArraySize(ExtIntr))
        {
            ucRet = (SALRetCode_t)SAL_RET_FAILED;
        }
        else
        {
            uiBitField      = (uint32_t)(8UL * ((uiIntId - (uint32_t)GIC_EXT0) % 4UL));
            uiEIntSelMask   = ((uint32_t)0x7FUL << uiBitField);

            uiEIntSel = (uint32_t)SAL_ReadReg(uiEintSel);
            uiEIntSel = (uint32_t)((uiEIntSel & ~uiEIntSelMask) | (uiExtIntGpioIdx << uiBitField));
            SAL_WriteReg(uiEIntSel, uiEintSel);
        }
    }

    return ucRet;
}

#endif // POOKY 20240906 gpio/tcc70xx/gpio.c








#define GIO_DATA  0x04
#define GIO_IODIR 0x08

#define DEV_CFG(dev)  ((const struct gpio_tccvcp_config *)(dev)->config)
#define DEV_DATA(dev) ((struct gpio_tccvcp_data *)(dev)->data)

struct gpio_tccvcp_config {
	struct gpio_driver_config common;

	DEVICE_MMIO_NAMED_ROM(reg_base);
	mem_addr_t offset;
};

struct gpio_tccvcp_data {
	struct gpio_driver_data common;

	DEVICE_MMIO_NAMED_RAM(reg_base);
	mem_addr_t base;
};

static int gpio_tccvcp_pin_configure(const struct device *port, gpio_pin_t pin, gpio_flags_t flags)
{
	struct gpio_tccvcp_data *data = port->data;

	if (flags & (GPIO_SINGLE_ENDED | GPIO_PULL_UP | GPIO_PULL_DOWN)) {
		return -ENOTSUP;
	}

	if (flags & GPIO_INPUT_TCC) {
		sys_set_bit(data->base + GIO_IODIR, pin);
	} else if (flags & GPIO_OUTPUT_TCC) {
		sys_clear_bit(data->base + GIO_IODIR, pin);

		if (flags & GPIO_OUTPUT_INIT_HIGH) {
			sys_set_bit(data->base + GIO_DATA, pin);
		} else if (flags & GPIO_OUTPUT_INIT_LOW) {
			sys_clear_bit(data->base + GIO_DATA, pin);
		}
	}

	return 0;
}

static int gpio_tccvcp_port_get_raw(const struct device *port, gpio_port_value_t *value)
{
	struct gpio_tccvcp_data *data = port->data;

	*value = sys_read32(data->base + GIO_DATA);

	return 0;
}

static int gpio_tccvcp_port_set_masked_raw(const struct device *port, gpio_port_pins_t mask,
					    gpio_port_value_t value)
{
	struct gpio_tccvcp_data *data = port->data;

	sys_clear_bits(data->base + GIO_DATA, mask);
	sys_set_bits(data->base + GIO_DATA, (value & mask));

	return 0;
}

static int gpio_tccvcp_port_set_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	struct gpio_tccvcp_data *data = port->data;

	sys_set_bits(data->base + GIO_DATA, pins);

	return 0;
}

static int gpio_tccvcp_port_clear_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	struct gpio_tccvcp_data *data = port->data;

	sys_clear_bits(data->base + GIO_DATA, pins);

	return 0;
}

static int gpio_tccvcp_port_toggle_bits(const struct device *port, gpio_port_pins_t pins)
{
	struct gpio_tccvcp_data *data = port->data;
	uint32_t reg_data;

	reg_data = sys_read32(data->base + GIO_DATA);
	reg_data ^= pins;
	sys_write32(reg_data, data->base + GIO_DATA);

	return 0;
}

static const struct gpio_driver_api gpio_tccvcp_api = {
	.pin_configure = gpio_tccvcp_pin_configure,
	.port_get_raw = gpio_tccvcp_port_get_raw,
	.port_set_masked_raw = gpio_tccvcp_port_set_masked_raw,
	.port_set_bits_raw = gpio_tccvcp_port_set_bits_raw,
	.port_clear_bits_raw = gpio_tccvcp_port_clear_bits_raw,
	.port_toggle_bits = gpio_tccvcp_port_toggle_bits,
};

int gpio_tccvcp_init(const struct device *port)
{
	const struct gpio_tccvcp_config *config = port->config;
	struct gpio_tccvcp_data *data = port->data;

	DEVICE_MMIO_NAMED_MAP(port, reg_base, K_MEM_CACHE_NONE);
	data->base = DEVICE_MMIO_NAMED_GET(port, reg_base) + config->offset;

	return 0;
}


#define GPIO_TCCVCP_INIT(n)                                                           \
	static struct gpio_tccvcp_data gpio_tccvcp_data_##n;                              \
                                                                                      \
	static const struct gpio_tccvcp_config gpio_tccvcp_cfg_##n = {                    \
		.common = {.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(0)},              \
		DEVICE_MMIO_NAMED_ROM_INIT(reg_base, DT_INST_PARENT(n)),                      \
		.offset = DT_INST_REG_ADDR(n),                                                \
	};                                                                                \
                                                                                      \
	DEVICE_DT_INST_DEFINE(n, gpio_tccvcp_init, NULL, &gpio_tccvcp_data_##n,           \
			      &gpio_tccvcp_cfg_##n, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,      \
			      &gpio_tccvcp_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_TCCVCP_INIT)
