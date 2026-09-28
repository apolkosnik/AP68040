; AP040 a write-back's bus error on the pipelined core (caches stage R)
; assembled with vasmm68k_mot -Fbin -m68040 -no-opt
;
; testbench protocol: $F100 fail number, $F102 result magic; $F110 w: the
; interrupt level requested; $F156 w / $F158 w: a one-shot bus error on the
; next data write / read sub-cycle to that longword; $F15A w: how many
; cycles that write sub-cycle is held before it errs; $F134 w / $F136 w: a
; word written to memory behind the CPU; $F138 w / $F13A r: memory's word
; at an address, whatever the data cache holds
;
; MC68040UM 8.4.6, Table 8-6: a write the processor has already let go --
; its instruction completed -- that meets a bus error is an access error
; taken at a later instruction boundary, "the instruction executing at the
; time the fault was detected", which the frame's PC names and RTE resumes.
; The SSW describes the write (RW 0, ATC 0); FA is its logical address; WB1S
; is valid and WB1A/WB1D are the write, WB1D in the byte lanes it was
; written on (Table 8-5), for the handler to complete (case 3). A push's
; bus error has TT 0, TM 0, a physical FA, WB1S invalid and the line in
; PD0-PD3 (case 2). MOVE16's write has TT 1, SIZE line, and its line in
; PD0-PD3 (case 4). No handler here completes a write-back: memory must
; show the write that erred never landed. A dirty line a line read was to
; replace goes back to its place when that read errs (4.6.2): nothing is
; written, and the line stays dirty (test 12).
;
; Two orderings are this core's: a trace already owed is taken before the
; fault, which is then held in the trace handler's first instruction (the
; 68040 sets CT, a continuation this core's RTE does not have) -- tests 7
; and 11; the fault is taken before an interrupt at the same boundary --
; test 8. A bus error on an
; exception frame's write is a double fault (tb_ap040_pipe_dblfault_bus16.v).
;
; memory map (physical):
;   $0400 code, $3400 ISP top
;   $3600 variables, $3700 the last access error frame, copied
;   $4000 root, $4200 pointer table, $4400 page table
;   $5000 page 5, write-through; $6000 page 6, copyback
;   $F000 page 15, the protocol registers, inhibited
; logical (tests 8-9, translated): identity, but for page 10, which is
; physical page 6 (copyback)

FAILREG		equ	$F100
DONEREG		equ	$F102
IPLREG		equ	$F110
PEEKA		equ	$F138
PEEKD		equ	$F13A
WBERR		equ	$F156
WBDLY		equ	$F15A
RBERR		equ	$F158
POKEA		equ	$F134
POKED		equ	$F136

cnt_a		equ	$3600		; access errors taken
cnt_t		equ	$3602		; traces taken
cnt_i		equ	$3604		; level 2 interrupts taken
pc_t		equ	$3608		; the last trace frame's PC
pc_i		equ	$360C		; the last interrupt frame's PC
frm		equ	$3700		; the last access error frame (60 bytes)
W		equ	$5000		; the writes that err

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

; fw <offset>,<word>,<test>: the frame's word
fw	macro
	cmp.w	#\2,(frm+\1).l
	beq.s	ok\@
	failt	\3
ok\@:
	endm

; fl <offset>,<longword>,<test>: the frame's longword
fl	macro
	cmp.l	#\2,(frm+\1).l
	beq.s	ok\@
	failt	\3
ok\@:
	endm

; peekl <address>: memory's longword, into d0
peekl	macro
	move.w	#\1,(PEEKA).l
	move.w	(PEEKD).l,d0
	swap	d0
	move.w	#(\1)+2,(PEEKA).l
	move.w	(PEEKD).l,d0
	endm

; poke <address>,<longword>: memory changed behind the CPU
poke	macro
	move.w	#\1,(POKEA).l
	move.w	#(\2)>>16,(POKED).l
	move.w	#(\1)+2,(POKEA).l
	move.w	#(\2)&$FFFF,(POKED).l
	endm

; wb <test>: the write-back fields of a write that erred -- SSW, WB1S, FA
; (and EA, and WB1A), WB1D, the other statuses clear, and one fault taken
wb	macro	; ssw, wb1s, fa, wb1d, test
	cmp.w	#1,(cnt_a).l
	beq.s	ok\@
	failt	\5
ok\@:
	fw	$06,$7008,\5+1
	fw	$0C,\1,\5+2
	fw	$0E,$0000,\5+3
	fw	$10,$0000,\5+3
	fw	$12,\2,\5+4
	fl	$14,\3,\5+5
	fl	$08,\3,\5+5
	fl	$28,\3,\5+5
	fl	$2C,\4,\5+6
	endm

; pcin <from>,<to>,<test>: the frame's PC in [from, to]
pcin	macro
	move.l	(frm+2).l,d0
	cmp.l	#\1,d0
	bcs.s	no\@
	cmp.l	#\2,d0
	bls.s	ok\@
no\@:
	failt	\3
ok\@:
	endm

	org	0
	dc.l	$3400
	dc.l	start
	dc.l	unexp		; 2, set below
	rept	253
	dc.l	unexp		; 3-255
	endr

	org	$400
start:
	move.w	#$2700,sr
	moveq	#0,d0
	movec	d0,cacr
	movec	d0,tc
	movec	d0,itt0
	movec	d0,itt1
	movec	d0,dtt0
	movec	d0,dtt1
	cinva	bc
	move.l	#h_aerr,(2*4).l
	move.l	#h_trace,(9*4).l
	move.l	#h_int2,(26*4).l
	clr.l	(cnt_a).l
	clr.l	(cnt_i).l
	lea	(W).l,a0
	move.w	#191,d1
wipe:	clr.l	(a0)+
	dbra	d1,wipe

;---------------- 1: a longword, straight through; each instruction once
; The run alternates MOVEQ #-1 (N) and ADDQ (flags clear): the SR stacked
; is the one after every instruction before the PC it names -- EX and WB
; drained -- so its NZVC say which came last.
	bsr	arm_clr
	move.w	#W,(WBERR).l
	moveq	#0,d6
	move.l	#$2001A5A5,(W).l	; NZVC clear
r1:
	rept	12
	moveq	#-1,d0
	addq.l	#1,d6
	endr
r1x:	tst.w	(FAILREG).l		; a read: behind the write on the bus;
	tst.w	(FAILREG).l		; this one only once it returns
r1e:	addq.l	#1,d6
	chkl	d6,13,1			; each ran once, the held one after RTE
	wb	$0005,$0085,W,$2001A5A5,2
	pcin	r1,r1x-2,9		; taken inside the run
	move.l	(frm+2).l,d0		; the stacked NZVC: N only after a MOVEQ
	sub.l	#r1,d0
	moveq	#0,d1
	btst	#1,d0			; an odd instruction (an ADDQ) follows a MOVEQ #-1
	beq.s	t1c
	moveq	#8,d1
t1c:	move.w	(frm).l,d0
	and.w	#$000F,d0
	cmp.w	d1,d0
	beq.s	t1d
	failt	105			; the SR stacked before the last instruction's flags
t1d:
	fl	$30,$00000000,10	; PD1-PD3 clear
	fl	$34,$00000000,10
	fl	$38,$00000000,10
	chkl	(W).l,0,11		; the write never landed

;---------------- 2-4: sizes and offsets, WB1D in the lanes written
	bsr	arm_clr
	move.w	#W+$20,(WBERR).l
	move.w	#$BEEF,(W+$21).l
	tst.w	(FAILREG).l
	tst.w	(FAILREG).l
	nop
	wb	$0045,$00C5,W+$21,$00BEEF00,12

	bsr	arm_clr
	move.w	#W+$30,(WBERR).l
	move.b	#$C3,(W+$33).l
	tst.w	(FAILREG).l
	tst.w	(FAILREG).l
	nop
	wb	$0025,$00A5,W+$33,$000000C3,20

	bsr	arm_clr
	move.w	#W+$40,(WBERR).l
	move.l	#$11223344,(W+$42).l
	tst.w	(FAILREG).l
	tst.w	(FAILREG).l
	nop
	wb	$0005,$0085,W+$42,$33441122,28

;---------------- 5-6: MOVES: FC 1, TM 1; FC 3, TT 2
	bsr	arm_clr
	moveq	#1,d0
	movec	d0,dfc
	lea	(W+$50).l,a0
	move.l	#$20050001,d1
	move.w	#W+$50,(WBERR).l
	moves.l	d1,(a0)
	tst.w	(FAILREG).l
	tst.w	(FAILREG).l
	nop
	wb	$0001,$0081,W+$50,$20050001,36

	bsr	arm_clr
	moveq	#3,d0
	movec	d0,dfc
	lea	(W+$54).l,a0
	move.l	#$20050003,d1
	move.w	#W+$54,(WBERR).l
	moves.l	d1,(a0)
	tst.w	(FAILREG).l
	tst.w	(FAILREG).l
	nop
	wb	$0013,$0093,W+$54,$20050003,44

;---------------- 7: a trace already owed goes first
; The store is traced: its trace entry is taken at the instruction after
; it, and the fault then in the trace handler's first instruction.
	bsr	arm_clr
	clr.w	(cnt_t).l
	move.w	#W+$60,(WBERR).l
	move.w	#$A700,sr		; T1
	move.l	#$20070000,(W+$60).l
r7:	addq.l	#1,d6
	move.w	#$2700,sr
	cmp.w	#1,(cnt_t).l
	beq.s	t7a
	failt	52			; not one trace
t7a:	chkl	(pc_t).l,r7,53		; the trace entry at the store's successor
	wb	$0005,$0085,W+$60,$20070000,54
	fl	$02,h_trace,61		; the fault in the trace handler's first instruction

;---------------- 8: before an interrupt at the same boundary
; Level 2 asked for under mask 7; the store's write will err, 40 cycles on,
; so the MOVE to SR lowering the mask arrives before the fault. Its refetch
; waits behind the write: the instruction after it arrives with both
; pending. The access error is taken there, and the interrupt in the
; access error handler's first instruction.
	bsr	arm_clr
	move.w	#2,(IPLREG).l
	rept	8
	nop				; the level delivered, masked
	endr
	move.w	#40,(WBDLY).l
	move.w	#W+$70,(WBERR).l
	move.l	#$20080000,(W+$70).l
	move.w	#$2000,sr
r8:	addq.l	#1,d6
	move.w	#$2700,sr
	move.w	#0,(WBDLY).l
	cmp.w	#1,(cnt_i).l
	beq.s	t8a
	failt	62			; not one interrupt
t8a:	wb	$0005,$0085,W+$70,$20080000,63
	fl	$02,r8,70		; the access error at the MOVE to SR's successor
	chkl	(pc_i).l,h_aerr,71	; the interrupt in its handler's first instruction

;---------------- translated: tables
	lea	($4400).l,a0
	moveq	#0,d0
	moveq	#63,d1
tloop:
	move.l	d0,d2
	lsl.l	#8,d2
	lsl.l	#4,d2			; i << 12
	addq.l	#3,d2			; resident, CM 00 (write-through)
	move.l	d2,(a0)+
	addq.l	#1,d0
	dbra	d1,tloop
	move.l	#$00006023,($4418).l	; page 6: copyback
	move.l	#$00006023,($4428).l	; page 10 -> physical 6, copyback
	move.l	#$0000F043,($443C).l	; page 15: inhibited
	move.l	#$00004203,($4000).l
	move.l	#$00004403,($4200).l
	move.l	#$4000,d0
	movec	d0,urp
	movec	d0,srp
	pflusha
	move.l	#$8000,d0		; E, 4K pages
	movec	d0,tc
	move.l	#$80000000,d0		; DE
	movec	d0,cacr

;---------------- 9: a push's bus error
; Written through logical page 10, the line is physical $6000's, copyback:
; longwords 1 and 3 dirty. CPUSHL (a physical address) pushes 1, then 3,
; which errs: SSW 0 (TT 0, TM 0, a longword), FA and WB1A physical, WB1S
; invalid, PD0-PD3 the line. CPUSH is done only once its push is: the
; fault is taken at the instruction after it.
	bsr	arm_clr
	move.l	#$11110000,($6000).l	; memory's line, through page 6
	move.l	#$11110004,($6004).l
	move.l	#$11110008,($6008).l
	move.l	#$1111000C,($600C).l
	cpusha	dc
	move.l	#$20090004,($A004).l
	move.l	#$2009000C,($A00C).l
	move.w	#$600C,(WBERR).l
	lea	($6000).l,a0
	cpushl	dc,(a0)
r9:	addq.l	#1,d6
	cmp.w	#1,(cnt_a).l
	beq.s	t9a
	failt	72
t9a:	fw	$06,$7008,73
	fw	$0C,$0000,74
	fw	$12,$0000,75
	fl	$14,$600C,76
	fl	$28,$600C,76
	fl	$2C,$11110000,77	; PD0-PD3: the line
	fl	$30,$20090004,78
	fl	$34,$11110008,78
	fl	$38,$2009000C,79
	fl	$02,r9,80		; at the instruction after the CPUSH
	peekl	$6004
	chkl	d0,$20090004,81		; the other dirty longword landed
	peekl	$600C
	chkl	d0,$1111000C,82		; the one that erred did not

;---------------- 10: MOVE16's write
; The destination's second longword errs: TT 1, SIZE line, FA its address,
; WB1S valid, PD0-PD3 the four longwords.
	bsr	arm_clr
	lea	(W+$100).l,a0
	move.l	#$200A0000,(a0)+
	move.l	#$200A0001,(a0)+
	move.l	#$200A0002,(a0)+
	move.l	#$200A0003,(a0)+
	lea	(W+$100).l,a0
	lea	(W+$200).l,a1
	move.w	#W+$204,(WBERR).l
	move16	(a0)+,(a1)+
	tst.w	(FAILREG).l
	tst.w	(FAILREG).l
	nop
	cmp.w	#1,(cnt_a).l
	beq.s	t10a
	failt	83
t10a:	fw	$0C,$006D,84
	fw	$12,$00ED,85
	fl	$14,W+$204,86
	fl	$28,W+$204,86
	fl	$2C,$200A0000,87
	fl	$30,$200A0001,87
	fl	$34,$200A0002,87
	fl	$38,$200A0003,87
	chkl	(W+$204).l,0,88		; the longword that erred never landed
	chkl	(W+$208).l,$200A0002,89	; the rest did

;---------------- 11: a trace owed and the fault held at one boundary
; CPUSHL is traced, and its push errs before it completes: the instruction
; after it arrives owing the trace with the fault already held. The trace
; goes first, the fault in the trace handler's first instruction.
	bsr	arm_clr
	clr.w	(cnt_t).l
	move.l	#$200B0004,($A014).l	; physical line $6010, longword 1 dirty
	move.w	#$6014,(WBERR).l
	lea	($6010).l,a0
	move.w	#$A700,sr		; T1
	cpushl	dc,(a0)
r11:	addq.l	#1,d6
	move.w	#$2700,sr
	cmp.w	#1,(cnt_t).l
	beq.s	t11a
	failt	90			; not one trace
t11a:	chkl	(pc_t).l,r11,91		; the trace at the CPUSH's successor
	cmp.w	#1,(cnt_a).l
	beq.s	t11b
	failt	92			; not one access error
t11b:	fl	$02,h_trace,93		; the fault in the trace handler's first instruction
	fw	$0C,$0000,94		; a push's
	fl	$14,$6014,94

;---------------- 12: a replaced dirty line goes back when its new line errs
; Set 0: four copyback lines, each dirty. A fifth line of the set is read
; (write-through page 5 allocates); its first longword answers the read,
; its second errs, and the line read is abandoned. The dirty line it was to
; replace was not written, and is back: memory still holds what was poked,
; the cache the dirty data, and CPUSHA writes all four.
	bsr	arm_clr
	cpusha	dc
	poke	$6000,$0C0C0000
	poke	$6400,$0C0C0001
	poke	$6800,$0C0C0002
	poke	$6C00,$0C0C0003
	move.l	#$200C0000,($6000).l
	move.l	#$200C0001,($6400).l
	move.l	#$200C0002,($6800).l
	move.l	#$200C0003,($6C00).l
	poke	$5000,$0C0C5000
	move.w	#$5004,(RBERR).l
	move.l	($5000).l,d0
	chkl	d0,$0C0C5000,100	; answered from its own longword
	tst.w	(cnt_a).l
	beq.s	t12a
	failt	101			; the abandoned line faulted the read
t12a:	peekl	$6000
	chkl	d0,$0C0C0000,102	; nothing written
	peekl	$6400
	chkl	d0,$0C0C0001,102
	peekl	$6800
	chkl	d0,$0C0C0002,102
	peekl	$6C00
	chkl	d0,$0C0C0003,102
	chkl	($6000).l,$200C0000,103	; the lines, from the cache
	chkl	($6400).l,$200C0001,103
	chkl	($6800).l,$200C0002,103
	chkl	($6C00).l,$200C0003,103
	cpusha	dc
	peekl	$6000
	chkl	d0,$200C0000,104	; still dirty: pushed now
	peekl	$6400
	chkl	d0,$200C0001,104
	peekl	$6800
	chkl	d0,$200C0002,104
	peekl	$6C00
	chkl	d0,$200C0003,104

;----------------------------------------------------------------- done
	cpusha	dc
	moveq	#0,d0
	movec	d0,cacr
	movec	d0,tc
	pflusha
	cinva	bc
	move.w	#$600D,(DONEREG).l
	stop	#$2700

; clear the frame copy and the access error count
arm_clr:
	lea	(frm).l,a0
	moveq	#14,d0
ac1:	clr.l	(a0)+
	dbra	d0,ac1
	clr.w	(cnt_a).l
	rts

; the access error: its frame copied, counted; RTE resumes the instruction
; it names -- no write-back completed
h_aerr:
	movem.l	d0/a0-a1,-(sp)
	lea	12(sp),a0
	lea	(frm).l,a1
	moveq	#14,d0
ha1:	move.l	(a0)+,(a1)+
	dbra	d0,ha1
	addq.w	#1,(cnt_a).l
	movem.l	(sp)+,d0/a0-a1
	rte

; a trace: counted, its PC kept, tracing off in the SR it returns to
h_trace:
	move.l	2(sp),(pc_t).l
	addq.w	#1,(cnt_t).l
	andi.w	#$3FFF,(sp)
	rte

; level 2: counted, its PC kept, the request withdrawn
h_int2:
	move.l	2(sp),(pc_i).l
	addq.w	#1,(cnt_i).l
	move.w	#0,(IPLREG).l
	rte

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
