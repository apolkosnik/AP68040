 org 0
 dc.l $3400,start,h_access
 rept 253
 dc.l unexpected
 endr
 org $400
start:
 move.w #$2700,sr
 moveq #0,d0
 movec d0,cacr
 movec d0,tc
 clr.w ($3600).l
 lea $303c,sp
original_fault:
 move.w ($f140).l,d0
 cmpa.l #$303c,sp
 bne badsp
 cmpi.w #2,($3600).l
 bne unexpected
 move.w #$600d,($f102).l
 stop #$2700
h_access:
 addq.w #1,($3600).l
 move.l sp,($f180).l
 move.l 2(sp),($f184).l
 move.l 20(sp),($f188).l
 cmpi.w #1,($3600).l
 bne nested
 cmpa.l #$3000,sp
 bne badsp
 move.w #$300c,($f15e).l
 rte
nested:
 cmpi.w #2,($3600).l
 bne unexpected
 cmpa.l #$2fc4,sp
 bne badsp
 rte
badsp:
 move.w #1,($f100).l
 bra failed
unexpected:
 move.w #99,($f100).l
failed:
 move.w #$bad0,($f102).l
 stop #$2700
