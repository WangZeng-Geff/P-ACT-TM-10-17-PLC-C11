;*******************************************************************************
; file       : startup_p131.s
; description: P131 Device Startup File
; author     : hqsun
; data       : 07/12/2016
; (C) COPYRIGHT 2016 Shanghai Eastsoft IC Co., Ltd.
;*******************************************************************************

;Stack Configuration------------------------------------------------------------
Stack_Size      EQU     0x00000004
                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp
;-------------------------------------------------------------------------------

;Heap Configuration-------------------------------------------------------------
Heap_Size       EQU     0x00000000

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB

; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
__Vectors
                DCD     __initial_sp                    ; 0,  load top of stack
                DCD     Reset_Handler                   ; 1,  reset handler
                DCD     NMI_Handler                     ; 2,  nmi handler
                DCD     HardFault_Handler               ; 3,  hard fault handler
                DCD     MemManage_Handler               ; 4,  MPU Fault Handler
                DCD     BusFault_Handler                ; 5,  Bus Fault Handler
                DCD     UsageFault_Handler              ; 6,  Usage Fault Handler
                DCD     0x00000000                      ; 7,  crc32_1
                DCD     0x00000000                      ; 8,  len
                DCD     0x00000000                      ; 9,  crc32_2
                DCD     sec_main                        ; 10, Reserved
                DCD     SVC_Handler                     ; 11, SVCall Handler
                DCD     DebugMon_Handler                ; 12, Debug Monitor Handler
                DCD     0                               ; 13, Reserved
                DCD     PendSV_Handler                  ; 14, pendsv handler
                DCD     SysTick_Handler                 ; 15, systick handler
                ;External Interrupts
                DCD     0                               ; 62, irq46 Reserved
                DCD     0                               ; 63, irq47 Reserved
                DCD     0                               ; 64, irq48 Reserved
                DCD     0                               ; 65, irq49 Reserved
         AREA    |.text|, CODE, READONLY

				EXPORT __ENTRY_INIT__
__ENTRY_INIT__
RW_INIT
				IMPORT ||Image$$SRAM$$RW$$Base||
				IMPORT ||Image$$SRAM$$RW$$Limit||
				IMPORT ||Image$$SRAM$$ZI$$Base||
				IMPORT ||Image$$SRAM$$ZI$$Limit||
				IMPORT ||Image$$SROM$$RO$$Base||
				IMPORT ||Image$$SROM$$RO$$Limit||
				LDR		R0, =||Image$$SROM$$RO$$Limit||
				LDR		R1, =||Image$$SRAM$$RW$$Base||
				LDR		R2,	=||Image$$SRAM$$RW$$Limit||
COPY_RW
				LDR		R3, [R0],  #4
				STR		R3, [R1],  #4
				CMP		R1, R2
				BLT		COPY_RW
				
ZI_INIT
				LDR		R0, =||Image$$SRAM$$ZI$$Base||
				LDR		R1,	=||Image$$SRAM$$ZI$$Limit||
CLR_ZI
				MOV		R2, #0x00
				STR		R2, [R0],  #4
				CMP		R0, R1
				BLT		CLR_ZI
				
				BX LR

				IMPORT __scatterload_copy
				IMPORT __scatterload_zeroinit
Proc_init
				PUSH    {LR}
				LDR		R0, =||Image$$SROM$$RO$$Limit||
				LDR		R1, =||Image$$SRAM$$RW$$Base||
				LDR		R2,	=||Image$$SRAM$$RW$$Limit||
				LDR     R3, =||Image$$SRAM$$RW$$Base||
				SUB		R2, R3
				BL.W    __scatterload_copy
				
				LDR		R0, =||Image$$SRAM$$RW$$Limit||
				LDR		R1, =||Image$$SRAM$$ZI$$Base||
				LDR		R2,	=||Image$$SRAM$$ZI$$Limit||
				LDR     R3, =||Image$$SRAM$$ZI$$Base||
				SUB	    R2, R3
				BL.W    __scatterload_zeroinit
				
				POP	{LR}
				BX LR
				IMPORT __scatterload
;Reset Handler----------------------------------------------
Reset_Handler   PROC
                EXPORT  Reset_Handler               [WEAK]

				;BL __ENTRY_INIT__
				IMPORT   sec_main
				
				PUSH	{R0-R1, LR}	
				;BL __ENTRY_INIT__
				BL.W	Proc_init		
				POP {R0-R1}
				BL		sec_main
				POP     {LR}
				BX      LR
                ENDP

; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler\
                PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler         [WEAK]
                B       .
                ENDP

HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B         .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler        [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler        [WEAK]
                B       .
                ENDP
SVC_Handler\
                PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler          [WEAK]
                B       .
                ENDP
PendSV_Handler\
                PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler\
                PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP
				EXPORT main
main\
				PROC
				NOP
				ENDP
                ALIGN

; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap
__user_initial_stackheap

                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR

                ALIGN

                ENDIF


                END
