.global vfp_init

vfp_init:

/* FPU_D16 Start */

/* Globally enable VFP */
    mrc p15, #0, r0, c1, c0, #2     @ r0 = Access Control Register
    orr r0, r0, #(0xf << 20)        @ enable full access for p10,11
    mcr p15, #0, r0, c1, c0, #2     @ Access Control Register = r0

    /* enable fpu access  */
    vmrs    r1, c8
    orr     r1, r1, #(1<<30)
    vmsr    c8, r1

    /* clear the floating point register*/
    mov     r1,#0
    vmov    d0,r1,r1
    vmov    d1,r1,r1
    vmov    d2,r1,r1
    vmov    d3,r1,r1
    vmov    d4,r1,r1
    vmov    d5,r1,r1
    vmov    d6,r1,r1
    vmov    d7,r1,r1
    vmov    d8,r1,r1
    vmov    d9,r1,r1
    vmov    d10,r1,r1
    vmov    d11,r1,r1
    vmov    d12,r1,r1
    vmov    d13,r1,r1
    vmov    d14,r1,r1
    vmov    d15,r1,r1
/* FPU_D16 End */

    bx  lr

    .end



