	AREA text, CODE, READONLY
	EXPORT asm_math

; function asm_math
; inputs:
; -------
; R0: input array of FIR values
; R1: output array containing RMS, max/min value/index
; R2: input array length

; R3: counter value i
; R4: RMS value

; S0: intermediate RMS register
; S1: max value
; S2: min value
; S3: input array length
; S4: RMS value
; S5: max index
; S6: min index

asm_math

	PUSH{R4, LR}
	
	LDR R3, =0 ;initialize int i to 0
	LDR R4, =0 ;initalize rms value to 0
	
	VLDR.f32 S1, [R0] ;initialize max value output
	VLDR.f32 S2, [R0] ;initialize min value output
	FMSR S3, R2 ;save length of array into FP register
	FSITOS S3, S3 ;convert int to FP
	
loop
	
	ADD R4, R0, R3, LSL #2 ; access inputVector[i] by shifting 2^i bytes on base address
	VLDR.f32 S0, [R4]
	;RMS calculation
	VFMA.f32 S4, S0, S0 ;square input value and add to sum
	
	CMP R3, #1 ;compare i with 1 to determine if it is in first index
	ADD R3, #1
	BLT loop   ;if in first index, branch to loop
	;only reach here if after first iteration
	
	SUB R3, #1 	;de-increment the i value by 1
	
	VCMP.f32 S0, S2 ;compare the current index with the min
	VMRS APSR_nzcv, FPSCR; ;check FP flag for branching
	BLLT min_loop	
	
	VCMP.f32 S0, S1 ;compare the current index with the max
	VMRS APSR_nzcv, FPSCR ;check FP flag for branching
	BLGT max_loop
	
	ADD R3, #1
	CMP R3, R2 ;compare i to the length of the input array
	BLT loop ;if i is less than length, re-loop

	VDIV.f32 S4, S4, S3 ;divide sum of squares by the length of array
	VSQRT.f32 S4, S4
	
	;store in output array
	VSTR.f32 S4, [R1, #0]
	VSTR.f32 S1, [R1, #4]
	VSTR.f32 S5, [R1, #8]
	VSTR.f32 S2, [R1, #12]
	VSTR.f32 S6, [R1, #16]
	POP{R4, LR}
	BX LR
	
max_loop
	VMOV.f32 S1, S0 ;save max value into register
	FMSR S5, R3 ;save max index into register
	FSITOS S5, S5 ;convert int to FP
	BX LR
	
min_loop
	VMOV.f32 S2, S0 ;save min value into register
	FMSR S6, R3 ;save min index into register
	FSITOS S6, S6 ;convert int to FP
	BX LR
	
	END