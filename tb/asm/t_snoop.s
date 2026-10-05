; AP68040-60 bus snooping (MC68040UM 4.4, 4.7 tables 4-3 and 4-4, 7.9)
; assembled with vasmm68k_mot -Fbin -m68040
;
; The bench's alternate bus master (tb/m68040_alt_master.sv, registers at
; $F200) takes the bus from the 68040 and runs transfers with a chosen
; SC1/SC0, so every snoop case can be checked from software:
;
;   data the 68040 holds dirty is supplied to the other master (SC 01 and
;   10), memory not being written; SC 10 then invalidates the line; an SC 01
;   byte/word/long write into a dirty line is sunk into the cache; any
;   other snooped write hit invalidates; SC 00 is not snooped; the
;   instruction cache drops a line for any snooped write and an SC 10 read;
;   a dirty victim waiting in the push buffer, or a push queued for the
;   bus, is snooped like the cache.
;
; Cached data is reached through MOVES with SFC = DFC = 1 (user data), which
; DTT0 maps copyback for user accesses only.  DTT1 makes every supervisor
; data access noncachable, so the bench registers, the stack and the
; memory prefills never touch the data cache.  Memory itself is read back
; with SC = 00 transfers of the alternate master, which the cache ignores.

FAILREG	equ	$F100
DONEREG	equ	$F102
AMX	equ	$F200		; transfer k: AMX+32k: +0 addr, +4 ctl, +8 data
AMGO	equ	$F280
AMTRIG	equ	$F284
AMDLY	equ	$F288
AMST	equ	$F28C
DUMMY	equ	$3800		; a harmless word for the warm-up "go"
STUB	equ	$3FF0		; I-cache test routine (set $3F)

RD	equ	1
SZL	equ	0<<1		; long
SZB	equ	1<<1		; byte
SZW	equ	2<<1		; word
SZLN	equ	3<<1		; line
SC0	equ	0<<3
SC1	equ	1<<3
SC2	equ	2<<3

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

chkm	macro			; memory (absolute), value, test
	cmp.l	#\2,(\1).l
	beq.s	.ok\@
	failt	\3
.ok\@:
	endm

; transfer k: address, control
amx	macro
	move.l	#\2,(AMX+32*\1).l
	move.w	#\3,(AMX+32*\1+4).l
	endm

; transfer k: write data (one long word, as on D31-D0)
amd	macro
	move.l	#\2,(AMX+32*\1+8).l
	endm

; run n transfers and wait for them
amgo	macro
	move.w	#\1,d0
	bsr	amrun
	endm

; cached (copyback, user space) write and read
cwr	macro
	lea	(\1).l,a0
	move.l	#\2,d0
	moves.l	d0,(a0)
	endm
crd	macro
	lea	(\1).l,a0
	moves.l	(a0),d1
	endm

; noncachable memory write (supervisor space)
mwr	macro
	move.l	#\2,(\1).l
	endm

	org	0
	dc.l	$3400
	dc.l	start
	rept	254
	dc.l	unexp
	endr

	org	$400
start:
	move.l	#$0000A040,d0	; DTT1: all of $00xxxxxx, supervisor, noncachable
	movec	d0,dtt1
	move.l	#$00008020,d0	; DTT0: all of $00xxxxxx, user, copyback
	movec	d0,dtt0
	moveq	#1,d0
	movec	d0,sfc
	movec	d0,dfc
	move.l	#$80008000,d0
	movec	d0,cacr
	clr.l	(AMDLY).l

;------------------------------------------- SC 01 read of a dirty line
	mwr	$8000,$AAAA0000
	cwr	$8000,$11111111		; allocate, now dirty
	amx	0,$8000,RD|SZL|SC1
	amx	1,$8000,RD|SZL|SC0
	amgo	2
	chkm	AMX+8,$11111111,1	; the 68040 supplied its data
	chkm	AMX+32+8,$AAAA0000,2	; memory was not written
	crd	$8000
	chkl	d1,$11111111,3
	cpusha	dc			; still dirty: the push writes it
	amx	0,$8000,RD|SZL|SC0
	amgo	1
	chkm	AMX+8,$11111111,4

;------------------------------ SC 01 line read: four beats, A3:A2 wraps
	mwr	$8010,$BBBB0000
	mwr	$8014,$BBBB0001
	mwr	$8018,$BBBB0002
	mwr	$801C,$BBBB0003
	cwr	$8010,$21000000
	cwr	$8014,$21000001
	cwr	$8018,$21000002
	cwr	$801C,$21000003
	amx	0,$8018,RD|SZLN|SC1	; starts at the third long word
	amx	1,$8010,RD|SZLN|SC0
	amgo	2
	chkm	AMX+8,$21000000,5
	chkm	AMX+12,$21000001,6
	chkm	AMX+16,$21000002,7
	chkm	AMX+20,$21000003,8
	chkm	AMX+32+8,$BBBB0000,9
	chkm	AMX+32+20,$BBBB0003,10

;-------------------------- SC 10 read of a dirty line: supply, invalidate
	mwr	$8020,$CCCC0000
	cwr	$8020,$22222222
	amx	0,$8020,RD|SZL|SC2
	amgo	1
	chkm	AMX+8,$22222222,11
	crd	$8020			; gone: memory's (stale) data comes back
	chkl	d1,$CCCC0000,12

;------------------------------------ SC 10 read of a clean line: invalidate
	mwr	$8030,$DDDD0000
	crd	$8030
	chkl	d1,$DDDD0000,13
	amx	0,$8030,RD|SZL|SC2
	amx	1,$8030,SZL|SC0		; memory changes behind the cache
	amd	1,$33333333
	amgo	2
	chkm	AMX+8,$DDDD0000,14
	crd	$8030
	chkl	d1,$33333333,15		; the line was dropped by the read

;---------------------------------------------- SC 00 is not snooped
	amx	0,$8030,SZL|SC0
	amd	0,$34343434
	amgo	1
	crd	$8030
	chkl	d1,$33333333,16		; stale: nothing told the cache
	lea	($8030).l,a0
	cinvl	dc,(a0)
	crd	$8030
	chkl	d1,$34343434,17

;------------------------------- SC 01 long write into a dirty line: sink
	mwr	$8040,$EEEE0000
	mwr	$8044,$EEEE0001
	cwr	$8040,$44444444
	cwr	$8044,$44444445
	amx	0,$8044,SZL|SC1
	amd	0,$55555555
	amx	1,$8044,RD|SZL|SC0
	amgo	2
	chkm	AMX+32+8,$EEEE0001,18	; memory was inhibited
	crd	$8044
	chkl	d1,$55555555,19
	crd	$8040
	chkl	d1,$44444444,20
	cpusha	dc			; the line stayed dirty
	amx	0,$8040,RD|SZLN|SC0
	amgo	1
	chkm	AMX+8,$44444444,21
	chkm	AMX+12,$55555555,22

;----------------------------------------- byte and word sinks, lanes
	cwr	$8050,$66666666
	amx	0,$8051,SZB|SC1
	amd	0,$00A50000		; byte 1: D23-D16
	amx	1,$8052,SZW|SC1
	amd	1,$00001234		; word 1: D15-D0
	amx	2,$8050,RD|SZL|SC0
	amgo	3
	crd	$8050
	chkl	d1,$66A51234,23
	chkm	AMX+64+8,$00000000,24	; memory untouched

;------------------------- SC 01 write to a clean line: invalidate, memory
	mwr	$8060,$12340000
	crd	$8060
	chkl	d1,$12340000,25
	amx	0,$8060,SZL|SC1
	amd	0,$77777777
	amgo	1
	crd	$8060
	chkl	d1,$77777777,26

;---------------- SC 10 write to a dirty line: invalidate, the dirty data
;---------------- is lost (MC68040UM table 4-4, D11)
	mwr	$8070,$56780000
	mwr	$8074,$56780001
	cwr	$8070,$88888888
	cwr	$8074,$88888889
	amx	0,$8070,SZL|SC2
	amd	0,$99999999
	amgo	1
	crd	$8070
	chkl	d1,$99999999,27
	crd	$8074
	chkl	d1,$56780001,28

;------------------- SC 01 line write to a dirty line: invalidate (D13)
	cwr	$8080,$AB000000
	amx	0,$8080,SZLN|SC1
	move.l	#$C0000000,(AMX+8).l
	move.l	#$C0000001,(AMX+12).l
	move.l	#$C0000002,(AMX+16).l
	move.l	#$C0000003,(AMX+20).l
	amgo	1
	crd	$8080
	chkl	d1,$C0000000,29
	crd	$808C
	chkl	d1,$C0000003,30

;------------------------------------ a snooped write that misses: memory
	amx	0,$8090,SZL|SC1
	amd	0,$BEEF0001
	amgo	1
	crd	$8090
	chkl	d1,$BEEF0001,31

;------------------------------------------------------- instruction cache
	mwr	STUB,$70014E75		; moveq #1,d0 ; rts
	jsr	(STUB).l
	chkl	d0,1,32
	amx	0,STUB,SZL|SC0		; not snooped
	amd	0,$70024E75
	amgo	1
	jsr	(STUB).l
	chkl	d0,1,33			; the cached copy still runs
	amx	0,STUB,SZL|SC1		; a snooped write drops the line
	amd	0,$70034E75
	amgo	1
	jsr	(STUB).l
	chkl	d0,3,34
	amx	0,STUB,SZL|SC0
	amd	0,$70044E75
	amx	1,STUB,RD|SZL|SC1	; an SC 01 read leaves it
	amgo	2
	jsr	(STUB).l
	chkl	d0,3,35
	amx	0,STUB,RD|SZL|SC2	; an SC 10 read drops it
	amgo	1
	jsr	(STUB).l
	chkl	d0,4,36

;---------------------- a push queued behind the alternate master (set $10)
; Four dirty lines fill the set; a fifth line's fill evicts one of them.
; The bench takes the bus right after the fill, so the victim's push is
; waiting for the bus while the other master reads all four lines.
	cpusha	dc
	mwr	$A100,$5A5A0000
	mwr	$A500,$5A5A0001
	mwr	$A900,$5A5A0002
	mwr	$AD00,$5A5A0003
	cwr	$A100,$D0000000
	cwr	$A500,$D0000001
	cwr	$A900,$D0000002
	cwr	$AD00,$D0000003
	amx	0,$A100,RD|SZL|SC1
	amx	1,$A500,RD|SZL|SC1
	amx	2,$A900,RD|SZL|SC1
	amx	3,$AD00,RD|SZL|SC1
	move.l	#$B100,(AMTRIG).l
	move.w	#$0104,(AMGO).l		; four transfers, after the fill of $B100
	crd	$B100
	bsr	amwait
	chkm	AMX+8,$D0000000,37
	chkm	AMX+32+8,$D0000001,38
	chkm	AMX+64+8,$D0000002,39
	chkm	AMX+96+8,$D0000003,40
	cpusha	dc
	amx	0,$A100,RD|SZL|SC0
	amx	1,$A500,RD|SZL|SC0
	amx	2,$A900,RD|SZL|SC0
	amx	3,$AD00,RD|SZL|SC0
	amgo	4
	chkm	AMX+8,$D0000000,41
	chkm	AMX+32+8,$D0000001,42
	chkm	AMX+64+8,$D0000002,43
	chkm	AMX+96+8,$D0000003,44

;------------ the victim in the push buffer while its fill waits (set $10)
; The other master holds the bus before the 68040 misses: the victim sits
; in the push buffer while the fill waits for the bus.  The instructions
; that run meanwhile come from the instruction cache (warmed first).
	mwr	$B500,$6B6B0000
	cwr	$A100,$E0000000
	cwr	$A500,$E0000001
	cwr	$A900,$E0000002
	cwr	$AD00,$E0000003
	amx	0,$A100,RD|SZL|SC1
	amx	1,$A500,RD|SZL|SC1
	amx	2,$A900,RD|SZL|SC1
	amx	3,$AD00,RD|SZL|SC1
	move.w	#200,(AMDLY).l
	lea	(DUMMY).l,a2		; warm-up: the routine into the I-cache
	lea	($A100).l,a0
	moveq	#4,d2
	bsr	pvgo
	lea	(AMGO).l,a2
	lea	($B500).l,a0
	bsr	pvgo
	bsr	amwait
	chkl	d1,$6B6B0000,45
	chkm	AMX+8,$E0000000,46
	chkm	AMX+32+8,$E0000001,47
	chkm	AMX+64+8,$E0000002,48
	chkm	AMX+96+8,$E0000003,49
	clr.w	(AMDLY).l
	cpusha	dc
	amx	0,$A100,RD|SZL|SC0
	amx	1,$A500,RD|SZL|SC0
	amx	2,$A900,RD|SZL|SC0
	amx	3,$AD00,RD|SZL|SC0
	amgo	4
	chkm	AMX+8,$E0000000,50
	chkm	AMX+32+8,$E0000001,51
	chkm	AMX+64+8,$E0000002,52
	chkm	AMX+96+8,$E0000003,53

;------------------- a read from a serialized page waits for earlier writes
; (MC68040UM 7.7): the bench copies a write of $F294 into $F2A8 (another
; line: no address collision orders them), so the read sees the new value
; only if the write went out first.  DTT1 makes both supervisor accesses
; noncachable serialized.
; The pair runs from the instruction cache (a first call warms it), so
; nothing but the ordering rule keeps the read behind the write.
	lea	($F294).l,a3
	lea	($F2A8).l,a4
	move.l	#$600DF00D,d3
	bsr	serrw
	chkl	d0,$600DF00D,54
	move.l	#$0BADCAFE,d3
	bsr	serrw
	chkl	d0,$0BADCAFE,55

;------------- ... and is read once: nothing older may abort it afterwards
; (MC68040UM 7.7).  An interrupt taken at the end of a long DIVU.L would
; restart the next instruction; if its serialized read had already gone
; out, the device would be read twice.  $F2B0 counts its bus reads; the
; interrupt (bench $F148: IPL 2 after n clocks) is swept across the
; division.  The routine runs from the instruction cache.  (A property
; check: this core's DIVU.L holds the read back until its last uop, and the
; DMU additionally waits for EX and WB to drain, so both keep it to one.)
	lea	($F2B0).l,a5
	move.l	#lev2,(26*4).w
	move.l	#$7FFFFFFF,d1
	moveq	#3,d2
	bsr	divrd			; warm-up, no interrupt
	move.w	#$2000,sr		; interrupts on
	moveq	#1,d4			; n: clocks to the interrupt
.sweep:
	clr.l	($F2B4).l
	moveq	#0,d5
	move.w	d4,($F148).l
	move.l	#$7FFFFFFF,d1
	bsr	divrd
.wait:	tst.w	d5			; the interrupt has been taken
	beq.s	.wait
	cmp.l	#1,($F2B4).l
	beq.s	.once
	move.w	#$2700,sr
	failt	56
.once:
	addq.w	#1,d4
	cmp.w	#80,d4
	bls.s	.sweep
	move.w	#$2700,sr

;----------------------------------------------------------------- done
	move.w	#$600D,(DONEREG).l
	bra.s	*

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

unexp:
	failt	99

; start the transfers (d0 = command word) and wait for them
amrun:
	move.w	d0,(AMGO).l
amwait:
	move.w	(AMST).l,d0
	beq.s	amwait
	cmp.w	#1,d0
	beq.s	.ok
	failt	98
.ok:	rts

; the "go" write to (a2), then a cached read at (a0) into d1.  A later
; read may pass an earlier write on the 68040 (MC68040UM 7.7): the NOP
; puts the write on the bus first.
	cnop	0,16
pvgo:
	move.w	d2,(a2)
	nop
	moves.l	(a0),d1
	rts

; a write to (a3) of d3, then a read of (a4) into d0
	cnop	0,16
serrw:
	move.l	d3,(a3)
	move.l	(a4),d0
	rts

; a division by d2, then a read of (a5): serialized (supervisor, DTT1)
	cnop	0,16
divrd:
	divu.l	d2,d1
	move.l	(a5),d0
	rts

lev2:
	clr.w	($F110).l		; withdraw IPL
	moveq	#1,d5
	rte
