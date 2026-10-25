 org 0
 dc.l $3400,start
 rept 254
 dc.l unexpected
 endr
 org $400
start:
 move.w #$2700,sr
 moveq #0,d0
 movec d0,cacr
 movec d0,tc
 clr.w ($3600).l
 lea $1000,a0
 move.w #255,d0
vectors:
 move.l #unexpected,(a0)+
 dbra d0,vectors
 move.l #$1000,d0
 movec d0,vbr
 move.l #h_irq,($1060).l
 move.w #2,($f110).l
 move.w #$2000,sr
 move.w #1000,d0
wait_irq:
 tst.w ($3600).l
 bne done
 dbra d0,wait_irq
 bra unexpected
done:
 move.w #$2700,sr
 cmpi.w #1,($3600).l
 bne unexpected
 move.w #$600d,($f102).l
 stop #$2700
h_irq:
 cmpi.w #$0060,6(sp)
 bne badframe
 move.w sr,d1
 andi.w #$3700,d1
 cmpi.w #$2200,d1
 bne badframe
 addq.w #1,($3600).l
 clr.w ($f110).l
 rte
badframe:
 move.w #1,($f100).l
 bra failed
unexpected:
 move.w #99,($f100).l
failed:
 move.w #$bad0,($f102).l
 stop #$2700
