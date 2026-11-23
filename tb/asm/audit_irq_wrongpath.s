; Audit: an interrupt taken while the front end runs down a mispredicted
; path.  A subroutine rewrites its return address, so the return stack
; predicts the RTS to the wrong place, and a level-3 interrupt is raised a
; few instructions before that RTS (0-24 NOPs: user mode, caches off; caches
; on; the MMU on (identity, 4K pages); supervisor mode with the ATC flushed
; just before, so the fetches down either path walk the tables).  The handler must run once and the RTS must go to the rewritten
; address; the predicted path (back_*) must never run.  On the Amiga
; (Maximig, cputest) the CPU went from the level-3 entry to the predicted
; path's $14 instead of the handler.
;   1  the predicted path ran, 2  the handler ran a wrong number of times,
;   99 an unexpected exception
 org 0
 dc.l $3400,start
 rept 25
 dc.l unexpected
 endr
 dc.l h_irq
 rept 4
 dc.l unexpected
 endr
 dc.l h_trap0
 rept 223
 dc.l unexpected
 endr

HITS equ $3600

VARIANT macro
 clr.l (HITS).l
 bsr.w sub_\@
back_\@:
 move.w #1,($f100).l
 bra failed
ok_\@:
 cmpi.l #1,(HITS).l
 beq.s next_\@
 move.w #2,($f100).l
 bra failed
sub_\@:
 move.l #ok_\@,(sp)
 move.w #3,($f110).l
 rept \1
 nop
 endr
 tst.l d0
 rts
next_\@:
 endm

; the same in supervisor mode, the ATC flushed before the interrupt
VARIANTF macro
 clr.l (HITS).l
 bsr.w fsub_\@
fback_\@:
 move.w #1,($f100).l
 bra failed
fok_\@:
 cmpi.l #1,(HITS).l
 beq.s fnext_\@
 move.w #2,($f100).l
 bra failed
fsub_\@:
 move.l #fok_\@,(sp)
 pflusha
 move.w #3,($f110).l
 rept \1
 nop
 endr
 tst.l d0
 rts
fnext_\@:
 endm

SWEEPF macro
 VARIANTF 0
 VARIANTF 1
 VARIANTF 2
 VARIANTF 3
 VARIANTF 4
 VARIANTF 5
 VARIANTF 6
 VARIANTF 7
 VARIANTF 8
 VARIANTF 9
 VARIANTF 10
 VARIANTF 11
 VARIANTF 12
 VARIANTF 13
 VARIANTF 14
 VARIANTF 15
 VARIANTF 16
 VARIANTF 17
 VARIANTF 18
 VARIANTF 19
 VARIANTF 20
 VARIANTF 21
 VARIANTF 22
 VARIANTF 23
 VARIANTF 24
 endm

SWEEP macro
 VARIANT 0
 VARIANT 1
 VARIANT 2
 VARIANT 3
 VARIANT 4
 VARIANT 5
 VARIANT 6
 VARIANT 7
 VARIANT 8
 VARIANT 9
 VARIANT 10
 VARIANT 11
 VARIANT 12
 VARIANT 13
 VARIANT 14
 VARIANT 15
 VARIANT 16
 VARIANT 17
 VARIANT 18
 VARIANT 19
 VARIANT 20
 VARIANT 21
 VARIANT 22
 VARIANT 23
 VARIANT 24
 endm

 org $400
start:
 move.w #$2700,sr
 moveq #0,d0
 movec d0,cacr
 movec d0,tc
 clr.w ($f110).l
 lea $3000,a0
 move.l a0,usp
 move.w #$0000,sr
 SWEEP
 ; again with both caches on
 moveq #0,d1
 trap #0
 SWEEP
 ; the MMU on (and the caches), user mode
 moveq #1,d1
 trap #0
 SWEEP
 ; supervisor mode, the ATC flushed before each interrupt
 moveq #2,d1
 trap #0
 SWEEPF
 move.w #$600d,($f102).l
.done:
 bra.s .done

; trap #0 (vector 32): d1 = 0 the caches on, 1 and the MMU (identity, 4K
; pages, the tables at $4000), 2 and stay in supervisor mode
h_trap0:
 move.l #$80008000,d0
 movec d0,cacr
 cpusha bc
 tst.l d1
 beq.s .t_rte
 lea ($4400).l,a0
 moveq #0,d0
 moveq #63,d2
.tl:
 move.l d0,d3
 lsl.l #8,d3
 lsl.l #4,d3
 addq.l #3,d3
 cmpi.l #15,d0
 bne.s .tc
 ori.w #$40,d3            ; the bench registers ($F000): cache inhibited
.tc:
 move.l d3,(a0)+
 addq.l #1,d0
 dbra d2,.tl
 move.l #$00004203,($4000).l
 move.l #$00004403,($4200).l
 cpusha dc
 move.l #$4000,d0
 movec d0,urp
 movec d0,srp
 pflusha
 move.l #$8000,d0
 movec d0,tc
 cmpi.l #2,d1
 bne.s .t_rte
 ori.w #$2000,(sp)        ; return in supervisor mode
.t_rte:
 rte

h_irq:
 addq.l #1,(HITS).l
 clr.w ($f110).l
 rte

unexpected:
 move.w #99,($f100).l
failed:
 move.w #$bad0,($f102).l
.f:
 bra.s .f
