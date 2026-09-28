; AP040 core-bench startup for compiled C: vectors, caches on, main, verdict
	section	"CODE",code
	xref	_main
	dc.l	$0000E000		; SSP
	dc.l	_start
	rept	254
	dc.l	_unexp
	endr
_start:
	move.l	#$80008000,d0		; both caches on
	movec	d0,cacr
	jsr	_main
	tst.l	d0
	bne.s	_fail
	move.w	#$600D,($F102).l
	stop	#$2700
_fail:
	move.w	d0,($F100).l
	move.w	#$BAD0,($F102).l
_h1:	bra.s	_h1
_unexp:
	move.w	#$0099,($F100).l
	move.w	#$BAD0,($F102).l
_h2:	bra.s	_h2
