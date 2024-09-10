#ifndef _PLL_H_
#define _PLL_H_

#define USE_SET_PLL_DIRECT
#define MICOM_MAIN_PLL_SEL  2   // Select MICOM dedicated pll

//#define MICOM_MAIN_PLL_VAL    0x00028005  // 614.4MHz (for T-soound)

#define VEHICLE_DEMO_PLL_933  // PWM turnned based on this base clock

#ifdef USE_SET_PLL_DIRECT

#ifdef VEHICLE_DEMO_PLL_933
#define MICOM_MAIN_PLL_VAL 0x00014dc4   // 933MHz
#else
#define MICOM_MAIN_PLL_VAL 0x00014a43   // 1188MHz
#endif

#else

#ifdef VEHICLE_DEMO_PLL_933
#define MICOM_MAIN_PLL_VAL (933000000)  // 933MHz
#else
#define MICOM_MAIN_PLL_VAL (1188000000) // 1188MHz
#endif

#endif // USE_SET_PLL_DIRECT

#endif

