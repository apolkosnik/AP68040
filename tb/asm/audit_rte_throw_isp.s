 org 0
 dc.l $3400,start,h_access
 rept 5
 dc.l unexpected
 endr
 dc.l h_priv
 rept 247
 dc.l unexpected
 endr
 org $400
start:
 move.w #$2700,sr
 moveq #0,d0
 movec d0,cacr
 movec d0,tc
 move.l #$3800,d0
 movec d0,msp
 lea $3c00,a0
 move.l a0,usp
 move.w #$2700,($3400).l
 move.l #returned,($3402).l
 clr.w ($3406).l
 lea $33f8,sp
 move.w #$2700,(sp)
 move.l #unexpected,2(sp)
 move.w #$1000,6(sp)
 rte
returned:
 move.w #$600d,($f102).l
 stop #$2700
h_access:
 move.w #2,($f100).l
 bra failed
h_priv:
 move.w #8,($f100).l
 bra failed
unexpected:
 move.w #99,($f100).l
failed:
 move.w #$bad0,($f102).l
 stop #$2700
