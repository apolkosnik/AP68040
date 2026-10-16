; AP68060 halt, LPSTOP and RESET: the processor's bus goes quiet
; assembled with vasmm68k_mot -Fbin -m68040
;
; The bench fails any processor bus cycle that starts while RSTO is
; asserted, and any after the LPSTOP broadcast other than the acknowledge
; cycle of the interrupt that ends it.  $F158 asks it to answer a halt with
; a reset: it checks the halted processor for 64 clocks (PST $1C, no bus
; cycle, BR negated) and pulses RSTI; $F15A counts those resets, so the
; program knows which phase it is in when it starts again.
;
; phase 0 (no reset yet)
;   1-2  LPSTOP #$2000 with IPL 2 raised 200 clocks later: one level 2
;        interrupt, its frame's PC the instruction after LPSTOP (7.8.2.1);
;        the bus quiet between (the bench's check)
;   8-9  LPSTOP again, its broadcast terminated by TEA: TEA completes it as
;        TA does (7.8.2.1), the stop is entered and the interrupt wakes it;
;        no access error
;   3-4  RESET with four writes waiting in the store buffer: they reach
;        memory before RSTO (2.8.4), no bus cycle while it is asserted
;   11   RESET at each of eight places in a line not yet fetched, the
;        instruction cache on and off: the fetch of the line may be under
;        way, RSTO waits for it to end (2.8.4; the bench fails a transfer
;        that overlaps RSTO)
;   then an address error whose frame write gets TEA: a double bus fault
;        (7.9.3): the processor halts, the bench resets it
; phase 1 (one reset)
;   5    the access error handler never ran
;   6    the reset's SR: supervisor, mask 7, no trace
;   then an address error whose handler address is odd: the second
;        address error, during the first one's exception processing, is a
;        double bus fault too
; phase 2 (two resets)
;   7    the address error handler never ran
;   then a reset while the 68060 owns the bus (the bench checks the end of
;        tenure and the pins' release), a dirty line left in the (then
;        disabled) data cache; phase 3:
;   10   the reset left the cache alone (5.3): CPUSHA, allowed with the
;        cache disabled, writes the dirty line;
;        then the same reset a clock later
;        (both phases of a half-rate bus clock); phase 4: done
;
;   98   an unexpected exception

FAILREG	equ	$F100
DONEREG	equ	$F102
BERRARM	equ	$F142
IPLDLY	equ	$F148
HALTRST	equ	$F158
HALTCNT	equ	$F15A
LPSTEA	equ	$F15C
TRESET	equ	$F168
IPLREG	equ	$F110

cnt_irq	equ	$3600
irq_pc	equ	$3604
cnt_ae	equ	$3608
cnt_adr	equ	$360C
reset_sr equ	$3610

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

chkl	macro			; register, value, test
	cmp.l	#\2,\1
	beq.s	.ok\@
	failt	\3
.ok\@:
	endm

	org	0
	dc.l	$3400
	dc.l	start
	dc.l	h_ae		; 2 access error
	dc.l	h_adr		; 3 address error
	rept	22
	dc.l	unexp
	endr
	dc.l	h_irq		; 26 level 2 autovector
	rept	229
	dc.l	unexp
	endr

	org	$400
start:
	move.w	sr,(reset_sr).l
	move.l	#h_adr,($C).l		; (phase 1 leaves it odd)
	move.w	(HALTCNT).l,d0
	beq	phase0
	cmp.w	#1,d0
	beq	phase1
	bra	phase2

;------------------------------------------------ phase 0
phase0:
	clr.l	(cnt_irq).l
	clr.l	(cnt_ae).l
	clr.l	(cnt_adr).l
	tst.l	($F140).l		; the reset-armed TEA: a counted access error
	clr.l	(cnt_ae).l

; 1-2 LPSTOP
	move.w	#200,(IPLDLY).l
	dc.w	$F800,$01C0,$2000	; lpstop #$2000
lps_next:
	nop
	move.l	(cnt_irq).l,d1
	chkl	d1,1,1
	move.l	(irq_pc).l,d1
	chkl	d1,lps_next,2
	move.w	#$2700,sr

; 8-9 LPSTOP with TEA on its broadcast
	clr.l	(cnt_irq).l
	move.w	#1,(LPSTEA).l
	move.w	#200,(IPLDLY).l
	dc.w	$F800,$01C0,$2000	; lpstop #$2000
lps_next2:
	nop
	move.l	(cnt_irq).l,d1
	chkl	d1,1,8
	move.l	(irq_pc).l,d1
	chkl	d1,lps_next2,9
	move.w	#$2700,sr

; 3-4 RESET with the store buffer full
	move.l	#$20000000,d0		; CACR ESB
	movec	d0,cacr
	nop
	move.l	#$11111111,($9000).l
	move.l	#$22222222,($9004).l
	move.l	#$33333333,($9008).l
	move.l	#$44444444,($900C).l
	reset
	moveq	#0,d0
	movec	d0,cacr
	move.l	($9000).l,d1
	chkl	d1,$11111111,3
	move.l	($900C).l,d1
	chkl	d1,$44444444,4

; 11 RESET with an instruction fetch in flight
	moveq	#0,d5			; CACR: the instruction cache off, then on
rs_pass:
	lea	rs_tab(pc),a2
	moveq	#7,d4
rs_loop:
	dc.w	$F4D8			; cinva bc: the block's line is cold
	movec	d5,cacr
	moveq	#0,d2
	move.l	(a2)+,a1
	jsr	(a1)
	chkl	d2,2,11
	dbra	d4,rs_loop
	tst.l	d5
	bne.s	rs_done
	move.l	#$00008000,d5
	bra.s	rs_pass
rs_done:
	moveq	#0,d0
	movec	d0,cacr

; the double bus fault: the address error's frame ($F138-$F143) meets TEA
	move.w	#1,(HALTRST).l
	move.w	#1,(BERRARM).l
	move.l	#$F144,sp
	jmp	($0401).l
	failt	97			; (not reached)

;------------------------------------------------ phase 1
phase1:
	move.l	(cnt_ae).l,d1
	chkl	d1,0,5
	moveq	#0,d1
	move.w	(reset_sr).l,d1
	chkl	d1,$2700,6
; the second: the address error vector itself is odd
	move.w	#2,(HALTRST).l
	move.l	#h_adr+1,($C).l
	jmp	($0401).l
	failt	97

;------------------------------------------------ phase 2
phase2:
	cmp.w	#2,d0
	bne.s	phase3
	move.l	(cnt_adr).l,d1
	chkl	d1,0,7
	move.w	#0,(HALTRST).l
; a dirty line at $9100 (memory $11112222), the cache disabled after it
	move.l	#$11112222,($9100).l
	dc.w	$F4D8			; cinva bc
	move.l	#$0000A020,d1		; DTT0: supervisor data copyback
	movec	d1,dtt0
	move.l	#$80000000,d1
	movec	d1,cacr
	move.l	#$AAAABBBB,($9100).l	; dirty
	moveq	#0,d1
	movec	d1,cacr
	movec	d1,dtt0
; a reset while the 68060 owns the bus explicitly (back-to-back transfers):
; the bench checks its end of tenure (7.11.2) and the pins' release
	move.w	#61,(TRESET).l
busy:	move.l	d1,($9000).l
	move.l	($9000).l,d1
	bra.s	busy

;------------------------------------------------ phase 3
; again a clock later: with BCLK at half CLK the two resets are recognised
; in the two phases of the bus clock
phase3:
	cmp.w	#3,d0
	bne.s	phase4
	dc.w	$F478			; cpusha dc (the cache disabled: allowed)
	move.l	($9100).l,d1
	chkl	d1,$AAAABBBB,10		; the dirty line survived the reset
	move.w	#62,(TRESET).l
	bra.s	busy

;------------------------------------------------ phase 4
phase4:
	move.w	#$600D,(DONEREG).l
	bra.s	*

; RESET after n MOVEQs from the start of a line, then more of the line
rsblk	macro
	cnop	0,16
rsb\1:
	rept	\1
	moveq	#0,d3
	endr
	reset
	rept	16
	moveq	#2,d2
	endr
	rts
	endm
	rsblk	0
	rsblk	1
	rsblk	2
	rsblk	3
	rsblk	4
	rsblk	5
	rsblk	6
	rsblk	7
rs_tab:
	dc.l	rsb0,rsb1,rsb2,rsb3,rsb4,rsb5,rsb6,rsb7

fail_all:
	move.l	#$3400,sp
	move.w	#$2700,sr
	move.w	#0,(IPLREG).l
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

h_irq:
	addq.l	#1,(cnt_irq).l
	move.l	2(sp),(irq_pc).l
	move.w	#0,(IPLREG).l
	nop
	rte

h_ae:
	addq.l	#1,(cnt_ae).l
	tst.w	(HALTCNT).l		; (the reset-armed one is expected)
	bne.s	h_bad
	cmp.l	#$F140,8(sp)
	bne.s	h_bad
	rte
h_adr:
	addq.l	#1,(cnt_adr).l
h_bad:
	failt	98
unexp:
	failt	98
