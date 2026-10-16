; AP68060: an interrupt taken while snoops invalidate the instruction cache
; assembled with vasmm68k_mot -Fbin -m68040
;
; The bench's alternate master ($F200..) snoops four distinct lines in one
; tenure: more than the snooper's two-entry instruction-cache queue holds,
; so the fetch unit invalidates every line (a 128-set sweep) and reports a
; hit (sn_ihit) at once -- the back end then refetches the next instruction
; to reach AG (EXC_SNR).  A level-2 interrupt is raised a varying number of
; clocks later ($F148), so that its exception entry runs while the sweeps
; keep reporting: the first uop of the exception routine must not be the
; one refetched (its pc is 0 -- the processor would continue at address 0,
; executing the vector table).  Every interrupt must run its handler once;
; the program restarting from address 0 is the failure the bug gives.
;   1   the alternate master's transfers failed
;   2   an interrupt was not taken (or its handler did not run)
;   3   the program restarted at its start: the PC went through address 0
;   99  an unexpected exception

FAILREG	equ	$F100
DONEREG	equ	$F102
IPLREG	equ	$F110
IPLDLY	equ	$F148		; raise IPL 2 N clocks from now
AMX	equ	$F200		; transfer k: AMX+32k: +0 addr, +4 ctl, +8 data
AMGO	equ	$F280
AMDLY	equ	$F288
AMST	equ	$F28C

RD	equ	1
SZL	equ	0<<1		; long
SNP	equ	1<<3		; SNOOP asserted

ROUNDS	equ	1200		; interrupts, the delay cycling over 1..255 clocks

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

amx	macro
	move.l	#\2,(AMX+32*\1).l
	move.w	#\3,(AMX+32*\1+4).l
	endm

	org	0
	dc.l	$3400
	dc.l	start
	rept	24
	dc.l	unexp		; vectors 2..25
	endr
	dc.l	h_irq2		; 26: level 2 autovector
	rept	229
	dc.l	unexp		; vectors 27..255
	endr

	org	$400
start:
	tst.w	started
	bne	restarted	; the PC came through address 0
	move.w	#1,started
	move.l	#$0000A040,d0	; DTT1: supervisor data noncachable
	movec	d0,dtt1
	move.l	#$0000C000,d0	; ITT0: every instruction fetch, writethrough
	movec	d0,itt0
	move.l	#$80808000,d0	; data cache, branch cache, instruction cache on
	movec	d0,cacr
	clr.l	count
	clr.l	(AMDLY).l
	amx	0,$A000,RD|SZL|SNP
	amx	1,$A010,RD|SZL|SNP
	amx	2,$A020,RD|SZL|SNP
	amx	3,$A030,RD|SZL|SNP
	move.w	#$2000,sr	; interrupts open
	moveq	#1,d6		; the delay (0 would never raise IPL)
	move.l	#ROUNDS,d5

round:
	move.l	count,d4	; interrupts so far
	move.w	#4,(AMGO).l	; the alternate master takes the bus: four snoops
	move.w	d6,(IPLDLY).l	; level 2 after d6 clocks
	; the transfers end
.am:	move.w	(AMST).l,d0
	beq.s	.am
	cmp.w	#1,d0
	beq.s	.ok
	failt	1
.ok:	; the handler ran once (bounded wait)
	move.l	#4000,d3
.ir:	move.l	count,d0
	sub.l	d4,d0
	cmp.l	#1,d0
	beq.s	.next
	subq.l	#1,d3
	bne.s	.ir
	failt	2
.next:	addq.w	#1,d6
	and.w	#$FF,d6
	bne.s	.nz
	moveq	#1,d6
.nz:
	subq.l	#1,d5
	bne	round

	move.w	#$600D,(DONEREG).l
	bra.s	*

h_irq2:
	move.w	#0,(IPLREG).l	; the request is withdrawn
	addq.l	#1,count
	rte

restarted:
	failt	3
unexp:
	failt	99
fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

	even
started:
	dc.w	0
count:
	dc.l	0
