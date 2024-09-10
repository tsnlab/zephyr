
.global ARM_DSB


ARM_DSB:
        DSB
        ISB
        BX      LR
        
	.end
