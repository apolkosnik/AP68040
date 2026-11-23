; Audit: an IRQ unmasked by the final RTE must precede its target.
 org 0
 dc.l $3400,start
 rept 24
 dc.l unexpected
 endr
 dc.l h_irq
 rept 229
 dc.l unexpected
 endr
 org $400
start:
 move.w #$2700,sr
 moveq #0,d0
 movec d0,cacr
 movec d0,tc
 clr.l ($3604).l
 clr.w ($3600).l
 lea $33f8,sp
 move.w #$2000,(sp)
 move.l #returned,2(sp)
 clr.w 6(sp)
 lea -8(sp),sp
 move.w #$2700,(sp)
 move.l #unexpected,2(sp)
 move.w #$1000,6(sp)
 move.w #2,($f110).l
 nop
 nop
 rte
returned:
 move.l #$12345678,($3604).l
 cmpi.w #1,($3600).l
 bne unexpected
 move.w #$600d,($f102).l
 stop #$2700
h_irq:
 move.l 2(sp),($f180).l
 move.l ($3604).l,($f184).l
 cmpi.l #returned,2(sp)
 bne bad_boundary
 tst.l ($3604).l
 bne bad_boundary
 addq.w #1,($3600).l
 clr.w ($f110).l
 rte
bad_boundary:
 move.w #1,($f100).l
 bra failed
unexpected:
 move.w #99,($f100).l
failed:
 move.w #$bad0,($f102).l
 stop #$2700
