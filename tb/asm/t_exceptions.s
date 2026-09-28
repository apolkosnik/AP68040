; AP040 milestone C exception, interrupt and control register self test
; assembled with vasmm68k_mot -Fbin -m68040
;
; testbench protocol:
;   word write to $F100 = failing test number
;   word write to $F102 = $BAD0 on failure, $600D when all tests passed
;   word write to $F110 = request interrupt level (0 releases)
;   byte write to $F120 must carry FC=1 (MOVES with DFC=1 check in the TB)
;
; frame layout under test (MC68040):
;   format $0: SR@0, PC@2, fmt/vec@6            (8 bytes)
;   format $1: same, throwaway                  (8 bytes)
;   format $2: + address@8                      (12 bytes)

FAILREG	equ	$F100
DONEREG	equ	$F102
IPLREG	equ	$F110
IPLPULSE	equ	$F14C	; raise IPL, then withdraw it N cycles later
IPLDLY	equ	$F148	; raise IPL N cycles from now
IPLSTEP	equ	$F150	; raise IPL, then DOWNGRADE it N cycles later
FBERRCTL equ	$F154	; one-shot bus error on a FETCH at the written address
FCREG	equ	$F120
BERRCTL equ	$F142
IRQEXCCTL equ	$F144
IPLCAP	equ	$F160	; bench capability word: bit 0 = coarse IPL delivery
			; (IPLREG, IPLDLY arriving eventually), bit 1 = cycle-
			; fine injectors (IPLDLY exact, IPLPULSE, IPLSTEP,
			; IRQEXCCTL), bit 2 = bus-error injection (BERRCTL,
			; FBERRCTL), bit 3 = the cache_allow window models
			; production (cache_allow_all=0), bit 4 = the real
			; fastchip/rtg block is instantiated.  Gated tests are
			; bypassed, never faked.

hfa		equ	$3674	; fault address seen by h_buserr
hfpc		equ	$3678	; and its stacked PC
hid		equ	$3670	; which handler called hfail (X2.3a diagnostic)
cnt_trap0	equ	$3600
cnt_ill		equ	$3602
cnt_aline	equ	$3604
cnt_fline	equ	$3606
cnt_chk		equ	$3608
cnt_divz	equ	$360A
cnt_trapv	equ	$360C
cnt_trapcc	equ	$360E
cnt_int2	equ	$3610
cnt_int5	equ	$3612
cnt_nmi		equ	$3614
cnt_mflag	equ	$3616
cnt_priv	equ	$3618
cnt_trapu	equ	$361A
cnt_fmt		equ	$361C
cnt_addr	equ	$361E
cnt_trace	equ	$3620
cnt_buserr	equ	$3622
cnt_int3	equ	$3624
cnt_fberr	equ	$3626
resume		equ	$3630
exp_addr	equ	$3634
exp_pc		equ	$3638
addr_sflag	equ	$363C
irq_early	equ	$363E
irq_guard	equ	$3640
buserr_fc	equ	$3642
fberr_fa	equ	$3654
irq_exc	equ	$3644
trap_guard	equ	$3646
exp_sr		equ	$3648
exp_srv		equ	$364A
tw_sr		equ	$364C
got_fsp		equ	$3650
irq_guard2	equ	$3652
int2_pc		equ	$3658	; level-2 frame PC captured by h_int2
trace_pc	equ	$365C	; first trace frame PC since the last clear
order_hit	equ	$3660	; trace-vs-IRQ order test saw a same-boundary hit
hold2		equ	$3664	; h_int2 leaves the line ASSERTED for N entries
kick5		equ	$3666	; h_int2 raises level 5 mid-handler once
nest52		equ	$3668	; h_int5 hands the line back to level 2 once

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

chkl	macro
	cmp.l	#\2,\1
	beq.s	ok\@
	failt	\3
ok\@:
	endm

chkccr	macro
	move.w	ccr,d6
	andi.w	#$1F,d6
	cmp.w	#\1,d6
	beq.s	ok\@
	failt	\2
ok\@:
	endm

chkcnt	macro			; \1 = counter address, \2 = expected, \3 = test
	move.w	(\1).l,d6
	cmp.w	#\2,d6
	beq.s	ok\@
	failt	\3
ok\@:
	endm

	org	0
	dc.l	$3400		; ISP
	dc.l	start
	dc.l	h_buserr	; 2 bus error
	dc.l	h_addr		; 3 address error
	dc.l	h_ill		; 4 illegal
	dc.l	h_divz		; 5 divide by zero
	dc.l	h_chk		; 6 CHK
	dc.l	h_trapv		; 7 TRAPV/TRAPcc
	dc.l	h_priv		; 8 privilege violation
	dc.l	h_trace		; 9 trace
	dc.l	h_aline		; 10 A-line
	dc.l	h_fline		; 11 F-line
	dc.l	unexp,unexp	; 12,13
	dc.l	h_fmt		; 14 format error
	dc.l	unexp		; 15
	rept	8
	dc.l	unexp		; 16-23
	endr
	dc.l	unexp,unexp	; 24 spurious, 25 level 1
	dc.l	h_int2		; 26 level 2 autovector
	dc.l	h_int3		; 27 level 3 autovector
	dc.l	unexp		; 28
	dc.l	h_int5		; 29 level 5 autovector
	dc.l	unexp		; 30
	dc.l	h_nmi		; 31 level 7 autovector
	dc.l	h_trap0		; 32 TRAP #0
	dc.l	h_trap1		; 33 TRAP #1
	rept	14
	dc.l	unexp		; 34-47
	endr
	rept	208
	dc.l	unexp		; 48-255
	endr

	org	$400
start:
	lea	(cnt_trap0).l,a0
	moveq	#17,d0
clrloop:
	clr.w	(a0)+
	dbra	d0,clrloop

;----------------------------------------------------------------- TRAP #0
	trap	#0
	chkcnt	cnt_trap0,1,1

;----------------------------------------------------------------- illegal / BKPT
	dc.w	$4AFC		; ILLEGAL
	chkcnt	cnt_ill,1,2
	dc.w	$4848		; BKPT #0: illegal on this implementation
	chkcnt	cnt_ill,2,3

;----------------------------------------------------------------- A/F-line
	dc.w	$A123
	chkcnt	cnt_aline,1,4
	dc.w	$F0FF		; 030 PMMU opcode: F-line on 040
	chkcnt	cnt_fline,1,5

;----------------------------------------------------------------- FSAVE/FRESTORE
; FPU-less state frame model: FSAVE writes a 4-byte NULL frame (version
; byte $00) and FRESTORE consumes it; neither takes the F-line trap
	move.l	#$DEADBEEF,($3530).l
	lea	($3534).l,a0
	dc.w	$F320		; fsave -(a0)
	cmp.l	#$3530,a0
	beq.s	fsv1
	failt	60
fsv1:
	tst.l	($3530).l	; NULL frame is all zero
	beq.s	fsv2
	failt	61
fsv2:
	dc.w	$F358		; frestore (a0)+
	cmp.l	#$3534,a0
	beq.s	fsv3
	failt	62
fsv3:
	chkcnt	cnt_fline,1,63	; F-line count unchanged by fsave/frestore

;----------------------------------------------------------------- CHK
	move.l	#5,d0
	chk.w	#3,d0		; out of bounds high
	chkcnt	cnt_chk,1,6
	move.l	#-1,d0
	chk.w	#100,d0		; negative
	chkcnt	cnt_chk,2,7
	move.l	#50,d0
	chk.w	#100,d0		; in bounds: no trap
	chkcnt	cnt_chk,2,8
	move.w	#10,($3520).l	; CHK2 bounds pair {10, 20}
	move.w	#20,($3522).l
	moveq	#15,d0
	chk2.w	($3520).l,d0	; inside: no trap
	chkcnt	cnt_chk,2,55
	moveq	#30,d0
	chk2.w	($3520).l,d0	; outside: vector 6, format $2
	chkcnt	cnt_chk,3,56

;----------------------------------------------------------------- divide by zero
	move.l	#7,d0
	divu.w	#0,d0
	chkcnt	cnt_divz,1,9

	; 68040 long divide-by-zero clears C but preserves X/N/Z/V.
	; Use all CCR bits set so a stale C is observable after RTE.
	moveq	#0,d1
	move.l	#$142,d0
	move.w	#$1F,ccr
	divul.l	d1,d2:d0
	chkccr	$1E,64
	chkcnt	cnt_divz,2,65

	move.l	#$142,d0
	move.w	#$1F,ccr
	divu.w	#0,d0
	chkccr	$1E,66
	chkcnt	cnt_divz,3,67

;----------------------------------------------------------------- TRAPV / TRAPcc
	move.w	#$02,ccr
	trapv
	chkcnt	cnt_trapv,1,10
	move.w	#$00,ccr
	trapv
	chkcnt	cnt_trapv,1,11
	move.w	#$04,ccr	; Z=1
	trapeq
	chkcnt	cnt_trapv,2,12
	trapne
	chkcnt	cnt_trapv,2,13
	trapeq.w #$1234		; with operand word
	chkcnt	cnt_trapv,3,14

;----------------------------------------------------------------- trace
	move.w	#$A000,sr	; T1 + S: trace every instruction
	nop			; traced; the handler clears the stacked T bits
	chkcnt	cnt_trace,1,57
	move.w	#$6000,sr	; T0 + S: trace only changes of flow
	nop			; NOP synchronizes the 040 pipeline and is T0-traced
	chkcnt	cnt_trace,2,58
	move.w	#$6000,sr
	moveq	#1,d0		; ordinary straight-line instruction: no trace
	chkcnt	cnt_trace,2,97
	move.w	#$2700,sr	; MOVE to SR is itself a T0 synchronization point
	chkcnt	cnt_trace,3,98
	move.w	#$6000,sr
	bra	t0flow		; taken branch: traced (word form: zero displacement)
t0flow:
	chkcnt	cnt_trace,4,59
	move.w	#$6000,sr
	stop	#$2700		; a traced STOP never enters stopped state
	chkcnt	cnt_trace,5,99
	; MOVEC to a control register is a T0 change of flow; reading one
	; back is NOT.  gencpu marks only i_MOVE2C ($4E7B) with
	; trace_t0_68040_only(), and cputest Basic/MOVEC2 expects the trace
	; after the $4E7B in its sequence rather than the $4E7A before it.
	move.w	#$6000,sr
	movec	vbr,d0		; $4E7A: reads a control register, no trace
	nop			; ...so the trace comes from this NOP instead
	chkcnt	cnt_trace,6,100
	move.w	#$2700,sr

;----------------------------------------------------------------- MOVEC matrix
	moveq	#0,d0
	move.l	#$100,d0
	movec	d0,vbr
	movec	vbr,d1
	chkl	d1,$100,15
	moveq	#0,d0
	movec	d0,vbr		; vectors back at zero

	move.l	#$FFFFFFFF,d0
	movec	d0,sfc
	movec	sfc,d1
	chkl	d1,7,16
	movec	d0,dfc
	movec	dfc,d1
	chkl	d1,7,17

	movec	d0,cacr
	movec	cacr,d1
	chkl	d1,$80008000,18
	moveq	#0,d0
	movec	d0,cacr

	move.l	#$FFFF4000,d0	; P bit only: E stays off here
	movec	d0,tc
	movec	tc,d1
	chkl	d1,$00004000,19
	moveq	#0,d0
	movec	d0,tc

	move.l	#$FFFFFFFF,d0
	movec	d0,itt0
	movec	itt0,d1
	chkl	d1,$FFFFE364,20
	movec	d0,dtt1
	movec	dtt1,d1
	chkl	d1,$FFFFE364,21
	moveq	#0,d0
	movec	d0,itt0
	movec	d0,dtt1

	move.l	#$FFFFFFFF,d0
	movec	d0,urp
	movec	urp,d1
	chkl	d1,$FFFFFE00,22
	movec	d0,srp
	movec	srp,d1
	chkl	d1,$FFFFFE00,23

	move.l	#$12345678,d0
	movec	d0,mmusr
	movec	mmusr,d1
	chkl	d1,$12345678,24

	move.l	#$3C00,d0
	movec	d0,usp
	movec	usp,d1
	chkl	d1,$3C00,25

	move.l	#$3800,d0
	movec	d0,msp
	movec	msp,d1
	chkl	d1,$3800,26

	movec	isp,d0		; active stack: must equal SP
	cmp.l	sp,d0
	beq.s	t27ok
	failt	27
t27ok:

;----------------------------------------------------------------- MOVE USP
	movea.l	#$3D40,a0	; differ from USP value left by MOVEC test above
	move.l	#100,d0
	divu.w	#3,d0		; let the fetch queue get ahead of decode
	move	a0,usp
	move	usp,a1
	cmpa.l	#$3D40,a1
	beq.s	t28ok
	failt	28
t28ok:

;----------------------------------------------------------------- MOVES
	moveq	#1,d0
	movec	d0,sfc
	movec	d0,dfc
	lea	(FCREG).l,a0
	move.b	#$5A,d1
	moves.b	d1,(a0)		; TB verifies FC=1 on this write
	moves.b	(a0),d2
	and.l	#$FF,d2
	chkl	d2,$5A,29

;----------------------------------------------------------------- PTEST/PFLUSH/CINV/CPUSH decode
; PTEST walks the translation tables even with TC.E clear (WinUAE's PTEST
; has no enable check), so point the root at a zeroed descriptor: the probe
; reports a plain nonresident MMUSR rather than a transparent shortcut.
	move.l	#$5800,d0
	movec	d0,urp
	movec	d0,srp
	clr.l	($5800).l	; root entry 0: invalid
	lea	($5000).l,a0
	ptestr	(a0)
	movec	mmusr,d0
	chkl	d0,0,30		; nonresident: R clear
	pflusha
	cinva	bc
	cpushl	dc,(a0)

;----------------------------------------------------------------- user mode round trip
	movea.l	#$3C00,a0
	movec	a0,usp
	move.w	#$0000,-(sp)	; format/vector
	pea	user_code(pc)
	move.w	#$0000,-(sp)	; user SR
	rte

user_code:
	move.l	#$CAFEBABE,-(sp)	; goes to the user stack
	trap	#1
	move.w	#$2700,sr	; privilege violation in user mode
	failt	31		; never reached

super_cont:
	chkcnt	cnt_trapu,1,32
	chkcnt	cnt_priv,1,33

;----------------------------------------------------------------- format error
	lea	fmt_cont(pc),a0
	move.l	a0,(resume).l
	move.w	#$B010,-(sp)	; format $B frame: invalid
	pea	fmt_cont(pc)
	move.w	#$2700,-(sp)
	rte			; must take a format error
fmt_cont:
	addq.l	#8,sp		; discard the fake frame
	chkcnt	cnt_fmt,1,34

	; A full MC68040 does not recognize the LC/EC format-$4 frame.
	lea	fmt4_cont(pc),a0
	move.l	a0,(resume).l
	clr.l	-(sp)		; frame bytes 12..15
	clr.l	-(sp)		; frame bytes 8..11
	move.w	#$4010,-(sp)	; format $4, arbitrary vector offset
	pea	fmt4_cont(pc)
	move.w	#$2700,-(sp)
	rte
fmt4_cont:
	lea	16(sp),sp	; invalid frame remains intact below format-error frame
	chkcnt	cnt_fmt,2,87

	; FRESTORE must reject a state frame with an incompatible version byte.
	; Vector 14 stacks the FRESTORE instruction PC in a format-$0 frame.
	move.l	#$42000000,($3540).l
	lea	frestore_bad_cont(pc),a0
	move.l	a0,(resume).l
	frestore	($3540).l
frestore_bad_cont:
	chkcnt	cnt_fmt,3,101

;----------------------------------------------------------------- address error
	lea	addr_cont(pc),a0
	move.l	a0,(resume).l
	move.l	#$400,(exp_addr).l
	lea	odd_jmp+2(pc),a0	; frame PC is the word after the opcode:
	move.l	a0,(exp_pc).l		; gencpu's i_JMP incpc(2) before the fault
	clr.w	(addr_sflag).l
odd_jmp:
	jmp	($0401).l	; odd target
addr_cont:
	chkcnt	cnt_addr,1,35

	; The 68040 validates a Bcc target even when the condition is false.
	; BNE.B +1 below is not taken (Z=1), but its calculated target is odd.
	lea	bcc_nt_cont(pc),a0
	move.l	a0,(resume).l
	lea	bcc_nt+2(pc),a0	; odd target with A0 cleared
	move.l	a0,(exp_addr).l
	lea	bcc_nt(pc),a0
	move.l	a0,(exp_pc).l
	move.w	#$04,ccr
bcc_nt:
	dc.w	$6601		; bne.b +1
bcc_nt_cont:
	chkcnt	cnt_addr,2,88

	; RTE to an odd user-mode PC with tracing enabled stacks the restored
	; user/T1 SR exactly.  The handler verifies it before forcing a safe
	; supervisor continuation.
	lea	rte_odd_cont(pc),a0
	move.l	a0,(resume).l
	move.l	#$400,(exp_addr).l
	lea	rte_odd(pc),a0		; the frame names the RTE itself
	move.l	a0,(exp_pc).l
	move.w	#1,(exp_srv).l
	move.w	#$8000,(exp_sr).l
	move.w	#$0000,-(sp)
	move.l	#$00000401,-(sp)
	move.w	#$8000,-(sp)	; user + T1
rte_odd:
	rte
rte_odd_cont:
	chkcnt	cnt_addr,3,89

	; A TAKEN branch to an odd target stacks the same masked address as the
	; not-taken case above: cputest ae Bcc.B/0001 runs bra.b to $4205002F
	; and expects the format-$2 address field to read $4205002E.
	lea	bra_odd_cont(pc),a0
	move.l	a0,(resume).l
	lea	bra_odd+2(pc),a0	; the odd target with A0 cleared
	move.l	a0,(exp_addr).l
	lea	bra_odd(pc),a0
	move.l	a0,(exp_pc).l
bra_odd:
	dc.w	$6001		; bra.b +1: target bra_odd+3 is odd
bra_odd_cont:
	chkcnt	cnt_addr,4,95

;----------------------------------------------------------------- interrupts
	move.w	#$2000,sr	; supervisor, mask 0
	move.w	#2,(IPLREG).l
	bsr	wait_int2_1
	chkcnt	cnt_int2,1,36

	; A request already asserted at an instruction boundary must be taken
	; AT that boundary: the next instruction does not execute first, and
	; the stacked PC names it.  cputest irq/all checks exactly this and it
	; failed on hardware -- exception 25 stacked the address AFTER the
	; tested instruction (43900004) where the reference expects 43900000.
	; The request is asserted while MASKED and left to settle, so nothing
	; here depends on synchroniser latency: the only question is which
	; boundary takes it once the mask drops.  It must be the very next
	; one -- the instruction after the SR write does not execute first.
	move.w	#$2700,sr	; mask 7: the request cannot be taken yet
	move.w	#2,(IPLREG).l	; assert level 2 and let it settle
	move.w	#40,d1
irq_bnd_settle:
	dbra	d1,irq_bnd_settle
	move.l	#0,(int2_pc).l
	move.w	#$2000,sr	; mask 0: taken at THIS boundary
irq_bnd_op0:
	nop
irq_bnd_op:
	moveq	#1,d0		; must NOT run before the interrupt is taken
irq_bnd_cont:
	move.w	#0,(IPLREG).l
	move.l	(int2_pc).l,d0
	chkl	d0,irq_bnd_op0,150
	subq.w	#1,(cnt_int2).l	; keep the absolute counts below intact

	; The mask can also be lowered by RTE -- which is how cputest enters
	; every test -- and the pending request must be taken at THAT
	; boundary, with the target instruction not yet executed.  Hardware
	; irq/all stacked the address after the tested instruction, so the
	; RTE path is tested separately from the MOVE-to-SR path above.
	move.w	#$2700,sr	; mask 7: level 2 cannot be taken yet
	move.w	#2,(IPLREG).l	; assert it and let it settle
	move.w	#40,d1
irq_rte_settle:
	dbra	d1,irq_rte_settle
	move.l	#0,(int2_pc).l
	move.w	#$0000,-(sp)	; format $0 frame
	pea	irq_rte_target
	move.w	#$2000,-(sp)	; SR with mask 0
	rte			; lowers the mask AND redirects
irq_rte_target:
	moveq	#1,d0		; must NOT run before the interrupt is taken
irq_rte_cont:
	move.w	#0,(IPLREG).l
	move.l	(int2_pc).l,d0
	chkl	d0,irq_rte_target,151
	subq.w	#1,(cnt_int2).l	; keep the absolute counts below intact

	; masked interrupt stays pending
	move.w	#$2700,sr
	move.w	#5,(IPLREG).l
	move.w	#100,d0
mdelay:
	dbra	d0,mdelay
	chkcnt	cnt_int5,0,37	; must not have fired
	move.w	#$2400,sr	; open mask to 4: level 5 fires
	bsr	wait_int5_1
	chkcnt	cnt_int5,1,38

	; STOP wakes on pending interrupt after mask drop
	move.w	#$2700,sr
	move.w	#2,(IPLREG).l
	stop	#$2000
	chkcnt	cnt_int2,2,39

	; NMI is edge sensitive and pierces the mask
	move.w	#$2700,sr
	move.w	#7,(IPLREG).l
	bsr	wait_nmi_1
	chkcnt	cnt_nmi,1,40
	move.w	#7,(IPLREG).l
	bsr	wait_nmi_2
	chkcnt	cnt_nmi,2,41

;----------------------------------------------------------------- M bit throwaway
	move.w	#$2000,sr
	move.l	#$3800,d0
	movec	d0,msp
	movec	isp,d5		; remember ISP
	ori.w	#$1000,sr	; switch to master stack
	move.w	#2,(IPLREG).l
	bsr	wait_int2_3
	; back here with M restored from the master stack frame
	move.w	sr,d0
	andi.w	#$1000,d0
	beq	mfail
	chkcnt	cnt_mflag,1,42	; exactly one throwaway frame seen
	movec	msp,d0
	chkl	d0,$3800,43	; master stack fully unwound
	andi.w	#$EFFF,sr	; back to interrupt stack
	movec	isp,d0
	cmp.l	d0,d5
	beq.s	t44ok
	failt	44
t44ok:

	; A level already pending under mask 7 when RTE restores mask 0 IS
	; taken at that boundary: the instruction at the restored PC does not
	; execute first.  This test previously required the opposite, from
	; gencpu's `ipl_fetched = 10` on RTE -- but that machinery is emitted
	; only for the cycle-exact 68000/68020 paths (using_ce / isce020()).
	; The 68040 model has no such deferral: it checks interrupts in
	; do_specialties once RTE has restored SR and PC, so the request is
	; serviced with the RTE target stacked.  cputest irq/all agrees --
	; its ANDSR.B round EXPECTS frame PC $43900000, the tested
	; instruction's own address, and hardware reported $43900004 until
	; ap040_core sampled interrupts on the RTE path.
	move.w	#$2700,sr
	clr.w	(irq_guard).l
	clr.w	(irq_guard2).l
	move.w	#1,(irq_early).l
	move.w	#2,(IPLREG).l
	moveq	#20,d0
rte_irq_wait:
	dbra	d0,rte_irq_wait
	move.w	#$0000,-(sp)
	pea	rte_irq_target(pc)
	move.w	#$2000,-(sp)
	rte
rte_irq_target:
	move.w	#1,(irq_guard).l	; both run only AFTER the handler returns
	move.w	#1,(irq_guard2).l
	chkcnt	cnt_int2,4,90
	tst.w	(irq_early).l
	beq.s	rte_irq_ok
	failt	91
rte_irq_ok:
	move.w	#$2700,sr

	move.w	(IPLCAP).l,d0	; tests 93-105/136/137 need cycle-fine
	btst	#1,d0		; injectors; benches without them advertise
	beq	fine_inj_done	; it and the block is bypassed, not faked

	; If an interrupt becomes pending while another exception is being
	; processed, the 68040 stacks the interrupt and vectors to it before it
	; executes the first instruction of the original exception handler.
	move.w	#$2000,sr		; IPL2 is unmasked in the interrupted context
	clr.w	(trap_guard).l
	move.w	#1,(irq_exc).l
	move.w	#1,(IRQEXCCTL).l	; TB raises IPL2 after TRAP #0 starts
	trap	#0
	chkcnt	cnt_trap0,2,93
	chkcnt	cnt_int2,5,94
	tst.w	(irq_exc).l
	beq.s	exc_irq_done
	failt	95
exc_irq_done:
	tst.w	(trap_guard).l
	bne.s	exc_irq_guarded
	failt	96
exc_irq_guarded:
	; Repeat in the narrower vector-read/handler-refill window. The TB holds
	; the fetched handler opcode until IPL synchronization completes. The
	; core must discard it and take IPL2 before executing h_trap0's guard.
	move.w	#$2000,sr
	clr.w	(trap_guard).l
	move.w	#1,(irq_exc).l
	move.w	#2,(IRQEXCCTL).l
	trap	#0
	chkcnt	cnt_trap0,3,102
	chkcnt	cnt_int2,6,103
	tst.w	(irq_exc).l
	beq.s	fetch_irq_done
	failt	104
fetch_irq_done:
	tst.w	(trap_guard).l
	bne.s	fetch_irq_guarded
	failt	105
fetch_irq_guarded:

	; A device that drops its request before the CPU acknowledges it must
	; not produce an interrupt.  The 68040 requires IPL to be held until
	; acknowledged; the internal mask-qualified hold exists so a later
	; MOVE to SR raising the mask cannot lose a pending request, and it
	; must not outlive the request itself -- otherwise the level fires as
	; a phantom interrupt at the next instruction boundary, long after
	; the device let go.
	; Sweep several withdrawal widths; the testbench itself fails the run
	; if any interrupt is accepted after the pins have gone idle, so this
	; does not depend on where the code happens to land in a cycle.
	move.w	#$2000,sr		; mask 0, so the level qualifies at once
	moveq	#1,d5
irq_withdraw_loop:
	move.w	d5,d6
	lsl.w	#8,d6
	ori.w	#2,d6			; d6 = width:level
	move.w	d6,(IPLPULSE).l
	nop
	nop
	nop
	nop
	addq.w	#1,d5
	cmp.w	#6,d5
	bls.s	irq_withdraw_loop
	move.w	#0,(IPLREG).l
	move.w	#$2700,sr

	; The other half of the same rule: a request that DID qualify keeps
	; its claim even though the very next instruction raises the mask.
	; The 68040 sets IPEND when the level beats the mask, and an
	; interrupt whose IPEND is set is taken at the next instruction
	; boundary regardless of a mask raised in the meantime.  Time the
	; request to arrive inside the MOVE to SR that masks it.
	move.w	#$2000,sr		; mask 0 while the request arrives
	move.w	(cnt_int2).l,d5
	; The request has to reach the registered level while the mask is
	; still 0.  With the fetch queue on its own request channel (P171)
	; MOVE-to-SR no longer waits for its immediate, so the NOP keeps the
	; mask raise far enough out: four cycles lands inside MOVE-to-SR on
	; both the shared-port and the split-channel cores (3-5 pass on
	; P171, 3-7 on the shared port; 6+ arrive after the mask on P171).
	move.w	#4,(IPLDLY).l
	nop
	move.w	#$2700,sr		; request qualifies inside this insn
	nop
	nop
	nop
	move.w	(cnt_int2).l,d6
	sub.w	d5,d6
	cmp.w	#1,d6
	beq.s	irq_hold_ok
	failt	136			; qualified request lost to a later mask
irq_hold_ok:
	move.w	#0,(IPLREG).l
	move.w	#$2700,sr

	; Downgrade, not withdrawal: a second device keeps requesting at a
	; LOWER level when the qualified one lets go, so the IPL encoder
	; falls back instead of going idle.  The retained hold must
	; requalify against the mask -- the lower level is a fresh request
	; and may never be taken at or below it (levels 1-6 are accepted
	; only strictly above SR[10:8]; a real 040 resamples IPL
	; continuously).  A hold that merely tracks the pins DOWN retargets
	; the qualified level 5 to the still-asserted level 3 and fires it
	; against mask 3 as a phantom.  The divide keeps the instruction
	; boundary away until after the downgrade so the level-5 request
	; qualifies mid-instruction; sweep the width across the synchronizer
	; window.  The testbench independently fails any interrupt accepted
	; at or below the mask.
	move.w	#$2300,sr		; mask 3: 5 qualifies, 3 never may
	moveq	#1,d5
irq_step_loop:
	move.w	d5,d6
	lsl.w	#8,d6
	ori.w	#$35,d6			; width:levels -- 5 now, 3 after width
	move.w	d6,(IPLSTEP).l
	move.l	#100,d0
	divu.w	#3,d0			; boundary held off past the downgrade
	nop
	move.w	#0,(IPLREG).l
	addq.w	#1,d5
	cmp.w	#8,d5
	bls.s	irq_step_loop
	move.w	#$2700,sr
	chkcnt	cnt_int3,0,137		; downgraded level fired at/below mask
fine_inj_done:

	move.w	(IPLCAP).l,d0	; tests 138-141 inject fetch bus errors
	btst	#2,d0
	beq	fberr_done

	; Queue fault discipline (X2.2): a bus error on a SPECULATIVE
	; instruction fetch must never surface as an exception.  $F154 arms
	; a one-shot fetch berr at a written address -- the only reachable
	; speculative-fault source, since prefetch never leaves the page.
	; The four nops keep the armed word beyond the queue's 16-byte
	; lookahead at arming time; the divide then gives the fill engine
	; the idle cycles to reach it.
	; 138: the faulted word is discarded by a redirect; nothing fires
	; and the stream continues.
	clr.w	(cnt_fberr).l
	lea	t138_y(pc),a0
	move.l	a0,(fberr_fa).l
	move.w	a0,(FBERRCTL).l
	nop
	nop
	nop
	nop
	move.l	#100,d0
	divu.w	#3,d0		; the queue runs ahead across the branch
	bra.s	t138_done	; redirect discards the faulted fetch
t138_y:
	nop
	nop
t138_done:
	chkcnt	cnt_fberr,0,138
	move.w	#0,(FBERRCTL).l	; disarm

	; 139: the faulted word IS reached: the fault is only recorded, the
	; demand point re-issues the fetch, the consumed one-shot lets it
	; succeed, and execution continues -- no exception anywhere.
	clr.w	(cnt_fberr).l
	moveq	#0,d1
	lea	t139_x(pc),a0
	move.l	a0,(fberr_fa).l
	move.w	a0,(FBERRCTL).l
	nop
	nop
	nop
	nop
	; The premise is that t139_x is reached ONLY by a speculative fetch.
	; The queue fetches ALIGNED LONGWORDS, so that holds only if the DIVU
	; occupies a longword by itself: straddling two, its own demand fetch
	; for the extension word covers the next word too and faults for real
	; (X2.3a -- this used to hold by accident of layout, and any timing
	; change that moved the code exposed it).  move.l #imm,d0 is 6 bytes,
	; so aligning it to 4n+2 puts the DIVU on a longword boundary.
	cnop	2,4
	move.l	#100,d0
	divu.w	#3,d0		; speculative fetch of t139_x faults here
	cnop	0,4		; and t139_x starts its own longword
t139_x:
	moveq	#1,d1		; reached via the re-issued fetch
	chkcnt	cnt_fberr,0,139
	cmp.l	#1,d1
	beq.s	t139_ok
	failt	140
t139_ok:
	move.w	#0,(FBERRCTL).l

	; 141: a DEMAND fetch fault takes the access error: format $7,
	; FA = the fetch address, supervisor program space, ATC clear; RTE
	; restarts the fetch, which then succeeds.  The target sits BEHIND
	; the arming code so it can only ever be fetched through the
	; branch's flush -- a demand fetch by construction -- and in a
	; different 64-byte sector from it: the branch-refill buffer keeps
	; the last redirect's sector (the arming code's, entered by the
	; bra.s), and a target resident there is served without a bus
	; cycle, so the one-shot bus error would never be seen (P171).
	bra.s	t141_arm
t141_t:
	nop			; restarted after the handler
	bra.s	t141_chk
	cnop	0,64
t141_arm:
	clr.w	(cnt_fberr).l
	lea	t141_t(pc),a0
	move.l	a0,(fberr_fa).l
	move.w	a0,(FBERRCTL).l
	bra.s	t141_t		; backward: flush, then a faulting demand
t141_chk:
	chkcnt	cnt_fberr,1,141
	move.w	#0,(FBERRCTL).l
	clr.l	(fberr_fa).l
fberr_done:

	move.w	(IPLCAP).l,d0	; the 142/143 sweeps time IPL2 into exact
	btst	#1,d0		; instruction boundaries: cycle-fine IPLDLY
	beq	fine_sweep_done

	; Trace vs interrupt at one boundary.  The INTERRUPT is processed
	; first; these sweeps pin that, and hardware corroborates it: an odd
	; IRQ vector producing a nested address error stacks an SR carrying
	; the ACCEPTED interrupt mask, only reachable if the interrupt was
	; processed first.
	;
	; CORRECTION (2026-08-22).  An earlier version of this comment, and
	; commit cc742655, claimed WinUAE also DELIVERS the trace at the
	; interrupt handler's entry.  It does not, and the reasoning there
	; was wrong: it tracked SPCFLAG_DOTRACE across do_specialties passes
	; but missed exception_check_trace(), which every Exception() calls:
	;     unset_special(SPCFLAG_TRACE | SPCFLAG_DOTRACE);
	;     if (regs.t1) {
	;         if (currprefs.cpu_model < 68040 && internalexception(nr))
	;             set_special(SPCFLAG_DOTRACE);
	;     }
	;     regs.t1 = regs.t0 = 0;
	; On a 68040 the `< 68040` guard fails, so the interrupt's Exception()
	; CLEARS the armed DOTRACE and both T bits: WinUAE takes the
	; interrupt and DROPS the trace.
	;
	; AP040 instead delivers it at the handler entry through the texc
	; machinery.  That divergence is OPEN -- it may be a spurious trace
	; no real 68040 produces -- but it is deliberately left alone here
	; rather than changed on a reading, since the ordering these tests
	; check is the part hardware actually settled.  Any change needs a
	; corpus record or hardware capture that exercises a trace pending
	; when an interrupt is accepted.
	; 142 covers the change-of-flow boundary: sweep IPL2 into a traced
	; RTS; at least one delay must land both at the RTS boundary, where
	; the IRQ frame returns to the RTS target and the trace frame
	; stacks the IRQ handler's entry address.
	; The delay at which the coincidence lands scales with the core's
	; instruction timing, so the sweep must be wide enough to survive a
	; faster core (X2.3a).  It only needs ONE hit; a wider range costs
	; iterations, not accuracy.
	clr.w	(order_hit).l
	moveq	#2,d5
tio_loop:
	clr.l	(int2_pc).l
	clr.l	(trace_pc).l
	lea	tio_ret(pc),a0
	move.l	a0,-(sp)	; RTS target
	move.w	d5,(IPLDLY).l
	move.w	#$A000,sr	; T1 + S, mask 0; pins still idle here
	rts			; traced change of flow; IPL2 lands inside
tio_ret:
	move.w	#$2700,sr	; stop; the handlers already ran
	move.l	(int2_pc).l,d0
	lea	tio_ret(pc),a0
	cmp.l	a0,d0		; IRQ frame returned to the RTS target?
	bne.s	tio_next
	move.l	(trace_pc).l,d0
	lea	(h_int2).l,a0
	cmp.l	a0,d0		; trace delivered at the IRQ handler entry?
	bne.s	tio_next
	move.w	#1,(order_hit).l
tio_next:
	addq.w	#1,d5
	cmp.w	#40,d5
	bls.s	tio_loop
	move.w	#0,(IPLREG).l
	tst.w	(order_hit).l
	bne.s	tio_ok
	failt	142		; interrupt never won a simultaneous trace
tio_ok:

	; 143 covers the straight-line boundary: sweep IPL2 into a traced
	; divide so some delay lands trace and interrupt simultaneously
	; pending at its boundary.  The interrupt goes first and the trace
	; must arrive at the IRQ handler entry (trace frame PC = h_int2) --
	; a core that drops the trace event resumes tracing only at the
	; following instruction and stacks a later PC, and that signature
	; must then never appear across the whole sweep.
	; Same widening as 142: the coincidence delay moves with core timing.
	clr.w	(order_hit).l
	moveq	#2,d5
tio2_loop:
	clr.l	(int2_pc).l
	clr.l	(trace_pc).l
	move.w	d5,(IPLDLY).l
	move.w	#$A000,sr	; T1 + S, mask 0
	move.l	#100,d0
	divu.w	#3,d0		; traced; IPL2 qualifies inside
	move.w	#$2700,sr
	move.l	(trace_pc).l,d0
	lea	(h_int2).l,a0
	cmp.l	a0,d0
	bne.s	tio2_next
	move.w	#1,(order_hit).l
tio2_next:
	addq.w	#1,d5
	cmp.w	#48,d5
	bls.s	tio2_loop
	move.w	#0,(IPLREG).l
	tst.w	(order_hit).l
	bne.s	tio2_ok
	failt	143		; simultaneous trace lost or misplaced
tio2_ok:
fine_sweep_done:

;-------------------------- level-sensitive IPL: the NetBSD ports shape
; Amiga INT2 is shared (CIA-A, gayle IDE, other ports devices).  When a
; second device asserts before the first is acknowledged, the line NEVER
; DROPS across the handler's RTE.  A real 68040 samples IPL by level and
; re-enters; a core that waits for an edge never services level 2 again
; while every other level keeps running -- exactly the live NetBSD hang
; (uvmexp counters: vbl and clock advancing, ports frozen).  Here the
; handler leaves the line asserted through its first RTE: the second
; entry must be taken at that RTE boundary, before the guard runs.
	move.w	#$2700,sr
	clr.w	(hold2).l
	clr.w	(kick5).l
	clr.w	(nest52).l
	move.w	(cnt_int2).l,d6	; count relative to here
	move.w	#1,(hold2).l	; first entry: no acknowledge
	move.w	#2,(IPLREG).l	; assert under mask 7 and let it settle
	move.w	#40,d1
ihold_settle:
	dbra	d1,ihold_settle
	move.w	#$2000,sr	; mask 0: entry 1 at THIS boundary
irq_hold_guard:
	moveq	#1,d0		; runs only after BOTH entries
	move.w	#$2700,sr
	moveq	#0,d1
	move.w	(cnt_int2).l,d1
	sub.w	d6,d1
	chkl	d1,2,152		; held line re-entered after RTE
	move.l	(int2_pc).l,d0
	chkl	d0,irq_hold_guard,153 ; re-entry AT the RTE boundary

	; Nested service must not wedge the lower level: level 5 arrives
	; while level 2 is in service, the level-5 handler is nested, and on
	; its RTE the level-2 request is STILL standing -- but level 2 is in
	; service (mask 2), so it must not re-enter.  After the level-2
	; handler acknowledges and returns, a fresh level-2 request must
	; still be taken -- a core that latched "level 2 seen" during the
	; nesting would leave ports dead from then on.
	move.w	(cnt_int2).l,d6
	move.w	(cnt_int5).l,d5
	move.w	#1,(kick5).l	; h_int2 raises level 5 mid-handler
	move.w	#1,(nest52).l	; h_int5 hands the line back to level 2
	move.w	#2,(IPLREG).l	; assert under mask 7 and let it settle
	move.w	#40,d1
inest_settle:
	dbra	d1,inest_settle
	move.w	#$2000,sr	; entry: int2 -> nested int5 -> back
	moveq	#0,d1
	move.w	(cnt_int2).l,d1
	sub.w	d6,d1
	chkl	d1,1,154		; exactly one level-2 service, no re-entry
	moveq	#0,d1
	move.w	(cnt_int5).l,d1
	sub.w	d5,d1
	chkl	d1,1,156		; the nested level 5 really was taken
	move.w	#2,(IPLREG).l	; the NEXT ports request after the nesting
	move.l	#20000,d0
inext_wait:
	moveq	#0,d1
	move.w	(cnt_int2).l,d1
	sub.w	d6,d1
	cmp.w	#2,d1
	beq.s	inext_done
	subq.l	#1,d0
	bne.s	inext_wait
inext_done:
	chkl	d1,2,155		; is still delivered
	move.w	#$2700,sr

;----- self-modifying code in the CHIP window needs no cache flush (157/158)
; Instruction fetches from chip RAM must bypass the I-cache when the
; internal caches are on: nothing invalidates I lines when the blitter,
; trackdisk DMA, or a CPU decruncher writes code there (the snoop and
; store_inv reach the D bank only), and A500-era programs never CINV --
; Phenomena's Enigma crashed exactly here.  Capability bit 3 marks a
; bench whose cache_allow window models production (cache_allow_all=0),
; where the bypass is active.
	move.w	(IPLCAP).l,d0
	btst	#3,d0
	beq	smc_done
	move.l	#$80008000,d0
	movec	d0,cacr			; caches on: the I-line must be cacheable
	lea	(smc_t).l,a0
	jsr	(a0)			; first run: would cache the line
	chkl	d0,1,157
	move.w	#$7002,(a0)		; moveq #1 -> moveq #2, NO flush
	jsr	(a0)
	chkl	d0,2,158		; a stale I-line still returns 1
	moveq	#0,d0
	movec	d0,cacr
	cinva	bc
	bra.s	smc_done
smc_t:
	moveq	#1,d0
	rts
smc_done:

;----- the RTG register block must answer (159) ------------------------
; Hardware: reading the RTG ID at $B8010E gives $5001 on TG68K-020 and
; $0000 on this CPU, so the register block never responds and the driver
; can never enable RTG -- a black screen with everything downstream
; irrelevant.  The fastchip path (RTG regs, IDE, Akiko) was stubbed off
; in every bench, selack/ready tied low, so this handshake had never been
; simulated.  rtg.v holds ID/VERSION at $B8010E per the MiSTer.card.asm
; register map, and its read needs the cycle held until rtg_ready.
	move.w	(IPLCAP).l,d0
	btst	#4,d0			; the real fastchip/rtg block is present
	beq	rtgid_done

	; SetPatch turns the caches on as well as the MMU, and the RTG ID
	; reads $5001 before it and $0000 after -- so run the whole block
	; cached.  $B8xxxx is outside cache_win, so the L1 must treat every
	; one of these as nocache and bypass; if it ever caches one, the
	; register reads below stop tracking the hardware.
	move.l	#$80008000,d0
	movec	d0,cacr

	move.w	($B8010E).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$00005001,159	; ID = $50, VERSION = $01

; The driver never touches a register with a plain word move: it writes
; base as a longword at $B80100 and reads back through the same window.
; A longword access is one CPU request that the 16-bit adapter splits into
; two chipset cycles, so it re-enters fastchip with fastchip_lw asserted --
; a different path from the word read above, and the one MiSTer.card.asm
; actually uses.
	move.l	#$01345678,d0
	move.l	d0,($B80100).l		; base[31:16] then base[15:0]
	move.l	($B80100).l,d1
	chkl	d1,$01345678,160

	move.w	#$0567,($B80108).l	; HSIZE, 12 bits
	move.w	#$0345,($B8010A).l	; VSIZE, 12 bits
	move.l	($B80108).l,d1		; both halves in one longword read
	chkl	d1,$05670345,161

	move.w	#$1234,($B8010C).l	; STRIDE, 14 bits -> $1234
	move.l	($B8010C).l,d1		; stride in the high half, ID in the low
	chkl	d1,$12345001,162

	move.w	#$0015,($B80104).l	; FORMAT, 5 bits
	move.w	#$0001,($B80106).l	; ENABLE
	move.l	($B80104).l,d1
	chkl	d1,$00150001,163

; HRTMon reads memory a byte at a time, which is how the ID was first seen
; to differ between CPUs.  A byte read drives only one of uds/lds; rtg
; ignores the strobes on reads and always returns the whole word, so the
; adapter has to pick the right half.
	moveq	#0,d1
	move.b	($B8010E).l,d1
	chkl	d1,$00000050,164	; ID
	moveq	#0,d1
	move.b	($B8010F).l,d1
	chkl	d1,$00000001,165	; VERSION

; and the whole register file must survive being written back to the
; values the driver leaves behind, read as one 32-bit access each
	move.l	#$02000000,($B80100).l	; MEMORY_BASE, as the driver sets it
	move.l	($B80100).l,d1
	chkl	d1,$02000000,166

; The CLUT at $B80400-$B807FF is the one RTG window whose read handshake is
; longer -- rtg.v holds the cycle for three clk_sys edges (rd_r[2]) rather
; than one -- and an 8-bit RTG screen with an all-zero palette is black
; whether or not the framebuffer holds pixels.  Each entry is 4 bytes:
; the word at +0 carries RR in its low byte, the word at +2 carries GGBB,
; and rtg commits the whole 24 bits when the second word is written.
	move.w	#$0012,($B80400).l	; entry 0: RR
	move.w	#$3456,($B80402).l	; entry 0: GGBB, commits $123456
	move.w	#$00AB,($B80404).l	; entry 1: RR
	move.w	#$CDEF,($B80406).l	; entry 1: GGBB, commits $ABCDEF
	moveq	#0,d1
	move.w	($B80400).l,d1
	chkl	d1,$00000012,167
	moveq	#0,d1
	move.w	($B80402).l,d1
	chkl	d1,$00003456,168
	move.l	($B80404).l,d1		; both halves of entry 1 at once
	chkl	d1,$00ABCDEF,169

; The framebuffer aperture.  cpu_wrapper routes $02xxxxxx to the RAM port
; (ramaddr[26:23] = 4'b1110 -> DDR3 $27000000) rather than to fastchip, so a
; pixel write and an RTG register access use two different targets -- and
; cpu_wrapper's bus_complete ORs chipready, ramready and fastchip_ready
; without asking which target the current access belongs to.  The RAM
; controllers hold their acknowledgement until cpuCS falls, which lags the
; request, so a register access issued straight after a framebuffer write
; can be completed by the PREVIOUS access's ready.  That is the sequence the
; driver runs constantly: blit pixels, then touch a register.
	move.l	#$12345678,($02000000).l
	move.l	($02000000).l,d1
	chkl	d1,$12345678,175	; the aperture itself round-trips

	move.l	#$0BADC0DE,($02000010).l	; a RAM access immediately before
	move.w	($B8010E).l,d0			; ...an RTG register read
	and.l	#$FFFF,d0
	chkl	d0,$00005001,176	; must not be completed by the RAM ready

	move.l	($02000010).l,d1	; and a RAM read immediately before
	move.w	($B8010E).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$00005001,177
	chkl	d1,$0BADC0DE,178

; Consecutive accesses INSIDE the $B80xxx window.  aen (sel_rtg) covers the
; whole 4K page, so it stays high across the boundary between one access and
; the next, and rtg's read pipeline -- rd_r <= rd_ready ? 0 : {rd_r,aen&rd}
; -- is a free-running shift register that no address change requeues.  dout
; defaults to 16'h0000 for any in-window address that is neither a control
; register ($B80100-$B8010F) nor palette ($B80400-$B807FF), so a read that
; lands on one of those and is immediately followed by a register read is
; the shape that could hand back a stale $0000.  A memory dump walks exactly
; across the $B8010F/$B80110 boundary.
	moveq	#0,d1
	move.w	($B80110).l,d1		; in window, no register there
	chkl	d1,$00000000,179
	move.w	($B8010E).l,d0		; ...immediately followed by the ID
	and.l	#$FFFF,d0
	chkl	d0,$00005001,180

	move.w	($B80120).l,d1		; again, other way round
	move.l	($B8010C).l,d1		; stride:ID as one longword
	chkl	d1,$12345001,181

	; and a run straight through the end of the register block, which is
	; what dumping memory from $B80100 does
	lea	($B80100).l,a0
	move.l	(a0)+,d1
	chkl	d1,$02000000,182	; base
	move.l	(a0)+,d1
	chkl	d1,$00150001,183	; format:ena
	move.l	(a0)+,d1
	chkl	d1,$05670345,184	; hsize:vsize
	move.l	(a0)+,d1
	chkl	d1,$12345001,185	; stride:ID
	move.l	(a0)+,d1
	chkl	d1,$00000000,186	; past the block
	move.w	($B8010E).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$00005001,187	; and back to the ID afterwards

	; and the same reads once more after a full cache invalidate
	cinva	bc
	move.w	($B8010E).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$00005001,188
	move.l	($B80100).l,d1
	chkl	d1,$02000000,189

	moveq	#0,d0
	movec	d0,cacr			; back to the uncached regime
	cinva	bc
rtgid_done:

;-------------------- immediate group: destination must be data alterable
; ORI/ANDI/SUBI/ADDI/EORI with a PC-relative or immediate destination are
; illegal; executing them instead consumes the following words as operands
; and runs off into the instruction stream (cputest 68040_default ILLEGAL).
; The handler resumes at the opcode + 2, so the words that would have been
; the operands are NOPs here.  CMPI is the exception the 68020 added: it
; may read program space, and must not trap.
	clr.w	(cnt_ill).l
	dc.w	$003A,$4E71,$4E71	; ori.b  #x,(d16,pc)
	chkcnt	cnt_ill,1,70
	dc.w	$023A,$4E71,$4E71	; andi.b #x,(d16,pc)
	chkcnt	cnt_ill,2,71
	dc.w	$043A,$4E71,$4E71	; subi.b #x,(d16,pc)
	chkcnt	cnt_ill,3,72
	dc.w	$063A,$4E71,$4E71	; addi.b #x,(d16,pc)
	chkcnt	cnt_ill,4,73
	dc.w	$0A3A,$4E71,$4E71	; eori.b #x,(d16,pc)
	chkcnt	cnt_ill,5,74
	dc.w	$043C,$4E71		; subi.b #x,#imm: no CCR form exists
	chkcnt	cnt_ill,6,75
	dc.w	$0C3C,$4E71		; cmpi.b #x,#imm: immediate is not a source
	chkcnt	cnt_ill,7,76

; static bit ops: BCHG/BCLR/BSET need a data alterable destination, while
; BTST may read program space and even an immediate operand
	dc.w	$08BA,$0000,$4E71	; bclr #0,(d16,pc)
	chkcnt	cnt_ill,8,78
	dc.w	$08FA,$0000,$4E71	; bset #0,(d16,pc)
	chkcnt	cnt_ill,9,79
	dc.w	$0849,$4E71		; bchg #x,a1: an address register is illegal
	chkcnt	cnt_ill,10,80
	btst	#0,bittgt(pc)		; BTST may read program space
	chkcnt	cnt_ill,10,81
	dc.w	$017A,$0000,$4E71	; bchg d0,(d16,pc)
	chkcnt	cnt_ill,11,111
	dc.w	$01BA,$0000,$4E71	; bclr d0,(d16,pc)
	chkcnt	cnt_ill,12,112
	dc.w	$01FA,$0000,$4E71	; bset d0,(d16,pc)
	chkcnt	cnt_ill,13,113
	dc.w	$813A,$4E71		; or.b  d0,(d16,pc)
	chkcnt	cnt_ill,14,115
	dc.w	$913A,$4E71		; sub.b d0,(d16,pc)
	chkcnt	cnt_ill,15,116
	dc.w	$B13A,$4E71		; eor.b d0,(d16,pc)
	chkcnt	cnt_ill,16,117
	dc.w	$C13A,$4E71		; and.b d0,(d16,pc)
	chkcnt	cnt_ill,17,118
	dc.w	$D13A,$4E71		; add.b d0,(d16,pc)
	chkcnt	cnt_ill,18,119
	subq.w	#8,(cnt_ill).l
					; keep the legacy counter sequence below
	bra.s	bitok
bittgt:
	dc.b	0,0
bitok:

; single-operand writes also need a data alterable destination
	dc.w	$42BA,$4E71		; clr.l (d16,pc)
	chkcnt	cnt_ill,11,83
	dc.w	$4AFA,$4E71		; tas (d16,pc)
	chkcnt	cnt_ill,12,84
	dc.w	$403A,$4E71		; negx.b (d16,pc)
	chkcnt	cnt_ill,13,106
	dc.w	$443A,$4E71		; neg.b (d16,pc)
	chkcnt	cnt_ill,14,107
	dc.w	$463A,$4E71		; not.b (d16,pc)
	chkcnt	cnt_ill,15,108
	dc.w	$483A,$4E71		; nbcd (d16,pc)
	chkcnt	cnt_ill,16,109
	dc.w	$40FA,$4E71		; move sr,(d16,pc)
	chkcnt	cnt_ill,17,110
	dc.w	$42FA,$4E71		; move ccr,(d16,pc)
	chkcnt	cnt_ill,18,111

; Bit 8 is fixed clear in the unary/MOVEM subfamilies below.  The reserved
; aliases must not fall through by looking only at the size bits.
	dc.w	$4140			; reserved alias of negx.w d0
	chkcnt	cnt_ill,19,120
	dc.w	$4340			; reserved alias of clr.w d0
	chkcnt	cnt_ill,20,121
	dc.w	$4540			; reserved alias of neg.w d0
	chkcnt	cnt_ill,21,122
	dc.w	$4740			; reserved alias of not.w d0
	chkcnt	cnt_ill,22,123
	dc.w	$4908,$4E71,$4E71	; must not decode as link.l a0,#imm32
	chkcnt	cnt_ill,23,124
	dc.w	$4B40			; reserved alias of tst.w d0
	chkcnt	cnt_ill,24,125
	dc.w	$4D40,$4E71		; must not decode as divl + extension
	chkcnt	cnt_ill,25,126

; Reserved mode-7 source registers and PC-relative quick destinations are
; encoding errors, including when a privileged SR operation is attempted.
	dc.w	$46FE			; move <reserved>,sr
	chkcnt	cnt_ill,26,127
	dc.w	$42FE			; move <reserved>,ccr
	chkcnt	cnt_ill,27,128
	dc.w	$503A,$4E71		; addq.b #8,(d16,pc)
	chkcnt	cnt_ill,28,129
	; (Scc with a PC-relative encoding does not exist: mode 111 reg 010,
	; 011 and 100 are TRAPcc on the 68020 and later, and are legal.)
	tst.l	tsttgt(pc)		; TST may read program space on 020+
	chkcnt	cnt_ill,28,85
	bra.s	tstok
tsttgt:
	dc.l	0
tstok:

; CMPI against program space is legal on the 68040 and must not trap
	moveq	#0,d0
	cmpi.b	#0,cmpitgt(pc)
	chkcnt	cnt_ill,28,77
	bra.s	cmpiok
cmpitgt:
	dc.b	0,0
cmpiok:

;------------------------------------------------------- physical bus error
; The testbench rejects the first access to $F140 and allows its restart.
	move.w	(IPLCAP).l,d0
	btst	#2,d0
	beq	buserr_done
; The handler verifies format $7, a clear ATC-fault bit and the original
; supervisor-data function code before returning to the faulting instruction.
	move.w	#5,(buserr_fc).l
	move.l	(FCREG+$20).l,d0
	chkcnt	cnt_buserr,1,86

	; A faulting MOVES reports its selected transfer modifier in the SSW, but
	; exception stack/vector traffic itself must use supervisor-data FC=5.
	move.w	#1,(BERRCTL).l	; arm a second rejection in the testbench
	moveq	#1,d0
	movec	d0,sfc
	move.w	#1,(buserr_fc).l
	lea	(FCREG+$20).l,a0
	moves.l	(a0),d0
	chkcnt	cnt_buserr,2,92
buserr_done:

;----------------------------------------------------------------- all done

;=========== WinUAE-oracle exception/stack-frame battery (2026-08-07) ======
	clr.w	(exp_srv).l

; RTE to an odd PC commits the restored SR before taking the address error;
; the frame therefore carries that restored image (hardware cputest
; 68040_ae RTE/0001).
	lea	ex_t1_cont(pc),a0
	move.l	a0,(resume).l
	move.l	#$400,(exp_addr).l
	lea	ex_t1_rte(pc),a0	; the frame names the RTE itself
	move.l	a0,(exp_pc).l
	move.w	#$0000,-(sp)	; format $0
	move.l	#$00000401,-(sp)	; odd return PC
	move.w	#$0013,-(sp)	; restored SR: user, CCR=$13
	move.w	#1,(exp_srv).l
	move.w	#$0013,(exp_sr).l
ex_t1_rte:
	rte
ex_t1_cont:
	chkcnt	cnt_addr,5,104

; RTR to an odd address commits the popped CCR before taking the address
; error, and the format-$2 frame carries that updated image (the v20
; corpus validates the frame byte; newer WinUAE instead models a 68040
; quirk stacking the pre-RTR SR, but the v20 generator predates it and
; the corpus is the hardware acceptance test).  A7 keeps its pre-RTR
; value: the pop is backed out before the fault, so the popped words are
; still on the stack and the frame sits directly below them.
	move.w	#$2000,sr	; pin the upper byte: supervisor, mask 0
	lea	ex_t2_cont(pc),a0
	move.l	a0,(resume).l
	move.l	#$400,(exp_addr).l
	lea	ex_t2_rtr(pc),a0	; same as RTE: the frame names the RTR
	move.l	a0,(exp_pc).l
	pea	($00000401).l	; odd return address
	move.w	#$0000,-(sp)	; popped CCR = 0
	move.l	sp,d7		; pre-RTR A7
	move.w	#1,(exp_srv).l
	move.w	#$2000,(exp_sr).l	; frame image = supervisor + popped CCR=0
	move.w	#$1F,ccr	; old CCR: all set (last: MOVE to memory clears NZVC)
ex_t2_rtr:
	rtr
ex_t2_cont:
	addq.l	#6,sp		; the un-popped CCR word and return address
	move.l	(got_fsp).l,d6
	add.l	#12,d6		; frame base + format-$2 size
	cmp.l	d7,d6		; must equal the pre-RTR A7
	beq.s	ex_t2_sp_ok
	failt	130
ex_t2_sp_ok:
	chkcnt	cnt_addr,6,105

; The interrupt throwaway frame's SR image is the ORIGINAL SR with only S
; forced: the old interrupt mask and M survive in the image.
	move.w	#$3100,sr	; S + M, mask 1
	move.w	#2,(IPLREG).l
ex_t3_wait:
	move.w	(cnt_mflag).l,d0
	cmp.w	#2,d0
	bne.s	ex_t3_wait
	move.w	(tw_sr).l,d0
	chkl	d0,$3100,106	; M and the OLD mask kept, S already set
	andi.w	#$EFFF,sr	; back to the interrupt stack
	move.w	#$2700,sr

; An odd handler address is not a halt (that is reserved for vectors 2 and
; 3): any other exception whose vector holds an odd address takes a nested
; address error instead.
	move.l	($84).l,d5	; save TRAP #1 vector
	move.l	#$00000401,($84).l
	lea	ex_t4_cont(pc),a0
	move.l	a0,(resume).l
	move.l	#$400,(exp_addr).l
	; PC field: the VECTOR TABLE ENTRY that supplied the odd address
	; ($84 = vbr + 4*33 for TRAP #1), not the original exception's next
	; PC.  This test required the latter until the v24 corpus contradicted
	; it -- its ODD_EXC group stacks vbr + 4*vec, and honouring that took
	; the group from 0/33 to 20/33 with nothing else moving.
	move.l	#$84,(exp_pc).l
	movea.l	sp,a5		; the TRAP frame stays behind: unwind after
	trap	#1
ex_t4_next:
ex_t4_cont:
	movea.l	a5,sp		; drop the leaked TRAP frame
	move.l	d5,($84).l
	chkcnt	cnt_addr,7,107
	chkcnt	cnt_trapu,1,108	; the TRAP handler itself never ran (count
				; unchanged from the user round-trip test)

; A pending T0 trace does NOT survive an exception on the 68040 -- not for
; the internal ones (CHK et al) and not for the others either.  This test
; used to require a "survivor" trace at the handler's first instruction,
; from reading WinUAE's Exception_cpu_oldpc as the path every exception
; takes.  It is not: gencpu emits exception_cpu() only for divide-by-zero,
; CHK, TRAPV, TRAP #n and the RTE format error, and on a 68040 that path
; forces t0 = false for exactly those; everything else -- op_illg's
; vector 4 included -- goes through plain Exception(), which clears T0.
; Hardware settled it: cputest basic/all and fbasic/all both reported
; "Got unexpected trace exception ... Exception 4 also pending" after the
; ILLEGAL that terminates every test, for plain integer instructions as
; well as FP ones.
	move.w	(cnt_trace).l,d5
	move.w	#$6000,sr	; T0, supervisor
	dc.w	$4AFC		; ILLEGAL: no trace at h_ill entry
	move.w	#$2700,sr	; T0 restored by h_ill's RTE: this SR write
				; is itself a T0 change-of-flow, so it traces
	move.w	(cnt_trace).l,d6
	sub.w	d5,d6
	cmp.w	#1,d6		; the SR write only: no survivor
	beq.s	ex_t5a_ok
	failt	109
ex_t5a_ok:
	move.w	(cnt_trace).l,d5
	move.w	#$6000,sr	; T0 again
	move.l	#5,d0
	chk.w	#3,d0		; CHK trap: internal, T0 trace CANCELLED
	move.w	#$2700,sr	; only this SR write traces
	move.w	(cnt_trace).l,d6
	sub.w	d5,d6
	cmp.w	#1,d6
	beq.s	ex_t5b_ok
	failt	110
ex_t5b_ok:

; BSR and JSR to an ODD target fault BEFORE the return-address push:
; A7 must be untouched and no return address stored (hardware cputest
; 68040_odd_stk BSR.B on build 50: frame matched but A7 was pushed).
; Frame: format $2 vector 3, PC = the branch instruction (restart),
; address field = target with A0 cleared.
	lea	ex_t6_cont(pc),a0
	move.l	a0,(resume).l
	lea	ex_t6_op(pc),a0
	move.l	a0,(exp_pc).l
	lea	2(a0),a0
	move.l	a0,(exp_addr).l	; (op+3) & ~1 = op+2
	movea.l	sp,a4
ex_t6_op:
	dc.w	$6101		; bsr.b to ex_t6_op+3 (odd)
ex_t6_cont:
	chkcnt	cnt_addr,8,111
	cmpa.l	sp,a4
	beq.s	ex_t6_ok
	failt	112		; A7 changed: the push must not happen
ex_t6_ok:

	lea	ex_t7_cont(pc),a0
	move.l	a0,(resume).l
	lea	ex_t7_op(pc),a0
	move.l	a0,(exp_pc).l
	lea	6(a0),a0
	move.l	a0,(exp_addr).l	; target op+2+5 = op+7; field = op+6
	movea.l	sp,a4
ex_t7_op:
	dc.w	$6100,$0005	; bsr.w: base op+2, disp 5 -> odd target
ex_t7_cont:
	chkcnt	cnt_addr,9,113
	cmpa.l	sp,a4
	beq.s	ex_t7_ok
	failt	114
ex_t7_ok:

	lea	ex_t8_cont(pc),a0
	move.l	a0,(resume).l
	lea	ex_t8_tgt(pc),a0
	addq.l	#1,a0		; odd JSR target
	; Unlike BSR and JMP, gencpu guards i_JSR's odd-target special case
	; with cpu_level <= 1, so a 68040 pushes and then faults on the
	; INSTRUCTION FETCH at the odd target: the frame names that target.
	move.l	a0,(exp_pc).l
	move.l	a0,d0
	bclr	#0,d0
	move.l	d0,(exp_addr).l
	movea.l	sp,a4
ex_t8_op:
	jsr	(a0)
ex_t8_cont:
	chkcnt	cnt_addr,10,115
	cmpa.l	sp,a4
	beq.s	ex_t8_ok
	failt	116
ex_t8_ok:
ex_t8_tgt:

; DBcc checks the branch-target parity BEFORE the condition on the 040:
; DBT (condition always true, loop exits, no branch) to an odd label
; still takes the address error, with D-reg and A7 untouched.
	lea	ex_t9_cont(pc),a0
	move.l	a0,(resume).l
	lea	ex_t9_op(pc),a0
	move.l	a0,(exp_pc).l
	lea	6(a0),a0
	move.l	a0,(exp_addr).l	; target = op+2+5 = op+7; field = op+6
	moveq	#3,d3
	movea.l	sp,a4
ex_t9_op:
	dc.w	$50CB,$0005	; dbt d3,ex_t9_op+7 (odd target)
ex_t9_cont:
	chkcnt	cnt_addr,11,117
	cmpa.l	sp,a4
	beq.s	ex_t9_sp
	failt	118
ex_t9_sp:
	cmp.l	#3,d3		; DBT: counter must be untouched
	beq.s	ex_t9_ok
	failt	119
ex_t9_ok:

; RTS to an odd return address: the 68040 backs the pop out of A7 before
; taking the address error (gencpu cpu_level>=4 rolls areg7 back by 4), so
; the return address is still on the stack and the frame sits directly
; below the pre-RTS A7.  The stacked SR is the plain live SR: no CCR pop
; is involved, exception3_read_prefetch_only carries no override.
	move.w	#$2000,sr
	lea	ex_ta_cont(pc),a0
	move.l	a0,(resume).l
	move.l	#$400,(exp_addr).l
	lea	ex_ta_rts(pc),a0	; go_pc frames identify the instruction
	move.l	a0,(exp_pc).l
	move.w	#1,(exp_srv).l
	move.w	#$2000,(exp_sr).l
	pea	($00000401).l	; odd return address
	move.l	sp,d7		; pre-RTS A7
	move.w	#$00,ccr	; pin CCR incl. X for the exact frame-SR compare
ex_ta_rts:
	rts
ex_ta_cont:
	addq.l	#4,sp		; the un-popped return address
	move.l	(got_fsp).l,d6
	add.l	#12,d6
	cmp.l	d7,d6
	beq.s	ex_ta_sp_ok
	failt	131
ex_ta_sp_ok:
	chkcnt	cnt_addr,12,132

; RTD likewise: the pop and the displacement adjustment are both backed
; out (gencpu rolls areg7 back by 4+offs), so A7 is the pre-RTD value.
	move.w	#$2000,sr
	lea	ex_tb_cont(pc),a0
	move.l	a0,(resume).l
	move.l	#$400,(exp_addr).l
	lea	ex_tb_rtd(pc),a0
	move.l	a0,(exp_pc).l
	move.w	#1,(exp_srv).l
	move.w	#$2000,(exp_sr).l
	pea	($00000401).l	; odd return address
	move.l	sp,d7		; pre-RTD A7
	move.w	#$00,ccr	; pin CCR incl. X for the exact frame-SR compare
ex_tb_rtd:
	rtd	#8
ex_tb_cont:
	addq.l	#4,sp		; the un-popped return address
	move.l	(got_fsp).l,d6
	add.l	#12,d6
	cmp.l	d7,d6
	beq.s	ex_tb_sp_ok
	failt	133
ex_tb_sp_ok:
	chkcnt	cnt_addr,13,134

	move.w	#$600D,(DONEREG).l
	stop	#$2700

mfail:
	failt	45

;----------------------------------------------------------------- wait loops
wait_int2_1:
	move.l	#20000,d0
wi21:
	move.w	(cnt_int2).l,d1
	cmp.w	#1,d1
	beq.s	wi2done
	subq.l	#1,d0
	bne.s	wi21
	failt	50
wi2done:
	rts

wait_int2_3:
	move.l	#20000,d0
wi23:
	move.w	(cnt_int2).l,d1
	cmp.w	#3,d1
	beq.s	wi2done
	subq.l	#1,d0
	bne.s	wi23
	failt	51

wait_int5_1:
	move.l	#20000,d0
wi51:
	move.w	(cnt_int5).l,d1
	cmp.w	#1,d1
	beq.s	wi5done
	subq.l	#1,d0
	bne.s	wi51
	failt	52
wi5done:
	rts

wait_nmi_1:
	move.l	#20000,d0
wn1:
	move.w	(cnt_nmi).l,d1
	cmp.w	#1,d1
	beq.s	wndone
	subq.l	#1,d0
	bne.s	wn1
	failt	53
wndone:
	rts

wait_nmi_2:
	move.l	#20000,d0
wn2:
	move.w	(cnt_nmi).l,d1
	cmp.w	#2,d1
	beq.s	wndone
	subq.l	#1,d0
	bne.s	wn2
	failt	54

;----------------------------------------------------------------- handlers
h_trap0:
	move.w	#1,(hid).l	; X2.3a: identify the handler for hfail
	move.w	#1,(trap_guard).l	; deliberately the first handler instruction
	cmpi.w	#$0080,6(sp)
	bne	hfail
	addq.w	#1,(cnt_trap0).l
	rte

h_trap1:
	move.w	#2,(hid).l	; X2.3a: identify the handler for hfail
	; arrived from user mode: check stacked SR, USP and user stack data
	move.l	d0,-(sp)
	move.w	4(sp),d0	; stacked SR
	andi.w	#$2000,d0
	bne	hfail
	movec	usp,d0
	cmp.l	#$3BFC,d0
	bne	hfail
	move.l	($3BFC).l,d0
	cmp.l	#$CAFEBABE,d0
	bne	hfail
	addq.w	#1,(cnt_trapu).l
	move.l	(sp)+,d0
	rte

h_ill:
	move.w	#3,(hid).l	; X2.3a: identify the handler for hfail
	cmpi.w	#$0010,6(sp)
	bne	hfail
	addq.l	#2,2(sp)	; skip the 2-byte opcode
	addq.w	#1,(cnt_ill).l
	rte

h_aline:
	move.w	#4,(hid).l	; X2.3a: identify the handler for hfail
	cmpi.w	#$0028,6(sp)
	bne	hfail
	addq.l	#2,2(sp)
	addq.w	#1,(cnt_aline).l
	rte

h_fline:
	move.w	#5,(hid).l	; X2.3a: identify the handler for hfail
	cmpi.w	#$002C,6(sp)
	bne	hfail
	addq.l	#2,2(sp)
	addq.w	#1,(cnt_fline).l
	rte

h_chk:
	move.w	#6,(hid).l	; X2.3a: identify the handler for hfail
	cmpi.w	#$2018,6(sp)
	bne	hfail
	addq.w	#1,(cnt_chk).l
	rte

h_divz:
	move.w	#7,(hid).l	; X2.3a: identify the handler for hfail
	cmpi.w	#$2014,6(sp)
	bne	hfail
	addq.w	#1,(cnt_divz).l
	rte

h_trapv:
	move.w	#8,(hid).l	; X2.3a: identify the handler for hfail
	cmpi.w	#$201C,6(sp)
	bne	hfail
	addq.w	#1,(cnt_trapv).l
	rte

h_trace:
	move.w	#9,(hid).l	; X2.3a: identify the handler for hfail
	cmpi.w	#$2024,6(sp)	; format $2, vector 9
	bne	hfail
	tst.l	(trace_pc).l	; capture the FIRST trace since the clear
	bne.s	ht_nocap
	move.l	2(sp),(trace_pc).l
ht_nocap:
	andi.w	#$3FFF,(sp)	; stop tracing on return
	addq.w	#1,(cnt_trace).l
	rte

h_priv:
	move.w	#10,(hid).l	; X2.3a: identify the handler for hfail
	cmpi.w	#$0020,6(sp)
	bne	hfail
	ori.w	#$2000,(sp)	; return in supervisor mode
	move.l	#super_cont,2(sp)
	addq.w	#1,(cnt_priv).l
	rte

h_fmt:
	move.w	#11,(hid).l	; X2.3a: identify the handler for hfail
	cmpi.w	#$0038,6(sp)
	bne	hfail
	move.l	(resume).l,2(sp)
	addq.w	#1,(cnt_fmt).l
	rte

h_addr:
	move.w	#12,(hid).l	; X2.3a: identify the handler for hfail
	move.l	sp,(got_fsp).l	; frame base, for SP-preservation checks
	cmpi.w	#$200C,6(sp)
	bne	hfail
	move.l	8(sp),d6
	cmp.l	(exp_addr).l,d6
	bne	hfail
	move.l	2(sp),d6
	cmp.l	(exp_pc).l,d6
	bne	hfail
	tst.w	(exp_srv).l
	beq.s	haddr_noexact
	move.w	(sp),d6		; exact stacked-SR compare (RTE/RTR odd-PC
	cmp.w	(exp_sr).l,d6	; commits restored SR / popped CCR first)
	bne	hfail
	clr.w	(exp_srv).l
	andi.w	#$3FFF,(sp)	; resume the test without the restored trace bits
	ori.w	#$2000,(sp)	; and on the supervisor stack/context
	bra.s	haddr_sr_ok
haddr_noexact:
	tst.w	(addr_sflag).l
	beq.s	haddr_sr_ok
	move.w	(sp),d6
	andi.w	#$2000,d6
	beq	hfail
	andi.w	#$3FFF,(sp)	; clear restored trace bits
	clr.w	(addr_sflag).l
haddr_sr_ok:
	move.l	(resume).l,2(sp)
	addq.w	#1,(cnt_addr).l
	rte

h_buserr:
	move.w	#13,(hid).l	; X2.3a: identify the handler for hfail
	move.l	$14(sp),(hfa).l	; X2.3a: record the fault address for hfail
	move.l	2(sp),(hfpc).l
	cmpi.w	#$7008,6(sp)	; format $7, vector 2
	bne	hfail
	move.l	$14(sp),d0	; fault address
	cmp.l	(fberr_fa).l,d0	; the armed FETCH berr address?
	beq.s	hb_fetch
	cmpi.l	#(FCREG+$20),d0
	bne	hfail
	move.w	$0C(sp),d0	; SSW
	andi.w	#$0400,d0	; ATC fault must be clear for physical berr
	bne	hfail
	move.w	$0C(sp),d0
	andi.w	#7,d0		; supervisor data FC
	cmp.w	(buserr_fc).l,d0
	bne	hfail
	addq.w	#1,(cnt_buserr).l
	rte

hb_fetch:
	move.w	$0C(sp),d0	; SSW
	andi.w	#$0400,d0	; ATC clear: physical berr
	bne	hfail
	move.w	$0C(sp),d0
	andi.w	#7,d0		; supervisor program space
	cmp.w	#6,d0
	bne	hfail
	addq.w	#1,(cnt_fberr).l
	rte

h_int2:
	move.w	#14,(hid).l	; X2.3a: identify the handler for hfail
	move.l	d0,-(sp)
	move.l	6(sp),(int2_pc).l
	tst.w	(kick5).l	; nested-service test: a level-5 device
	beq.s	hi2nokick	; asserts while level 2 is in service
	clr.w	(kick5).l
	move.w	#5,(IPLREG).l
hi2nokick:
	tst.w	(irq_exc).l
	beq.s	hi2exc_ok
	tst.w	(trap_guard).l	; original TRAP handler must not have begun
	bne	hfail
	clr.w	(irq_exc).l
hi2exc_ok:
	tst.w	(irq_early).l
	beq.s	hi2early_ok
	tst.w	(irq_guard).l	; RTE takes the request AT its own boundary, so
	bne	hfail		; the restored instruction has NOT run yet
	tst.w	(irq_guard2).l
	bne	hfail
	clr.w	(irq_early).l
hi2early_ok:
	move.w	10(sp),d0	; frame format/vector
	andi.w	#$0FFF,d0
	cmpi.w	#$0068,d0
	bne	hfail
	move.w	10(sp),d0
	andi.w	#$F000,d0
	beq.s	hi2f0
	cmpi.w	#$1000,d0
	bne	hfail
	move.w	4(sp),d0	; capture the throwaway SR image (d0 is
	move.w	d0,(tw_sr).l	; pushed at entry, frame starts at 4(sp))
	addq.w	#1,(cnt_mflag).l
	move.w	sr,d0		; M must already be clear on a throwaway
	andi.w	#$1000,d0
	bne	hfail
hi2f0:
	addq.w	#1,(cnt_int2).l
	tst.w	(hold2).l	; level-sensitivity test: the device is NOT
	beq.s	hi2clr		; acknowledged -- the line stays asserted
	subq.w	#1,(hold2).l	; through the RTE and must be taken again
	bra.s	hi2held
hi2clr:
	move.w	#0,(IPLREG).l
hi2held:
	move.l	(sp)+,d0
	rte

h_int3:
	move.w	#15,(hid).l	; X2.3a: identify the handler for hfail
	cmpi.w	#$006C,6(sp)
	bne	hfail
	addq.w	#1,(cnt_int3).l
	move.w	#0,(IPLREG).l
	rte

h_int5:
	move.w	#16,(hid).l	; X2.3a: identify the handler for hfail
	cmpi.w	#$0074,6(sp)
	bne	hfail
	addq.w	#1,(cnt_int5).l
	tst.w	(nest52).l	; nested-service test: only the level-5
	beq.s	hi5clr		; device is cleared; the level-2 request
	clr.w	(nest52).l	; below it is still standing
	move.w	#2,(IPLREG).l
	rte
hi5clr:
	move.w	#0,(IPLREG).l
	rte

h_nmi:
	move.w	#17,(hid).l	; X2.3a: identify the handler for hfail
	cmpi.w	#$007C,6(sp)
	bne	hfail
	addq.w	#1,(cnt_nmi).l
	move.w	#0,(IPLREG).l
	rte

hfail:
	; hid names the handler that rejected something; the testbench prints
	; it, so a timing shift no longer produces an anonymous "test 98".
	failt	98

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt1:
	bra.s	halt1

unexp:
	move.w	#$0099,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt2:
	bra.s	halt2
