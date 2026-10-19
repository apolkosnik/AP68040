; AP68040-60 bench: start-up for C programs (vbcc), linked first at 0
; The bench reads $F100 (test number) and $F102 ($600D pass), and prints
; cycles between writes of $F108.

	section	text,code
	xdef	_stamp
	xdef	_done
	xref	_main

	dc.l	$7F000		; SSP
	dc.l	start
	rept	254
	dc.l	unexp
	endr

start:
	move.l	#$80008000,d0	; both caches on
	movec	d0,cacr
	ifd	COPYBACK
	; user data copyback (DTT0), supervisor data noncachable (DTT1, the
	; bench registers): the program runs in user mode, stamps trap
	move.l	#$00008020,d0
	movec	d0,dtt0
	move.l	#$0000A040,d0
	movec	d0,dtt1
	move.l	#tstamp,($80).w	; TRAP #0
	move.l	#tdone,($84).w	; TRAP #1
	lea	($7E000).l,a0
	move.l	a0,usp
	andi.w	#$DFFF,sr		; to user mode
	endc
	jsr	_main
	ifd	COPYBACK
	trap	#1			; back to supervisor with d0
	endc
	; d0 = 0: pass
_done:
	tst.l	d0
	bne.s	fail
	move.w	#$600D,($F102).l
	bra.s	*
fail:
	move.w	d0,($F100).l
	move.w	#$BAD0,($F102).l
	bra.s	*
unexp:
	moveq	#99,d0
	bra.s	fail

; void stamp(void): print the cycles since the previous stamp
_stamp:
	ifd	COPYBACK
	trap	#0
	rts
tstamp:
	endc
	nop			; everything before has completed
	move.w	#1,($F108).l
	nop
	ifd	COPYBACK
	rte
tdone:
	addq.l	#6,sp			; drop the frame: stay in supervisor
	bra	_done
	else
	rts
	endc
