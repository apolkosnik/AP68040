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
 clr.w ($3600).l
 lea $7ff4,sp
 move.w #$2700,(sp)
 move.l #returned,2(sp)
 move.w #$7008,6(sp)
 clr.l 8(sp)
 clr.w 12(sp)
 move.l #$4203,($4000).l
 move.l #$4403,($4200).l
 lea $4400,a0
 moveq #0,d0
 moveq #63,d1
tables:
 move.l d0,d2
 lsl.l #8,d2
 lsl.l #4,d2
 addq.l #3,d2
 move.l d2,(a0)+
 addq.l #1,d0
 dbra d1,tables
 clr.l ($4420).l
 move.l #$4000,d0
 movec d0,srp
 move.l #$8000,d0
 movec d0,tc
 pflusha
faulting_rte:
 rte
returned:
 cmpa.l #$8030,sp
 bne badsp
 cmpi.w #1,($3600).l
 bne badcount
 move.w #$600d,($f102).l
 stop #$2700
h_access:
 move.l sp,($f180).l
 move.l 2(sp),($f184).l
 move.l 20(sp),($f188).l
 addq.w #1,($3600).l
 cmpi.w #1,($3600).l
 bne badcount
 cmpa.l #$7fb8,sp
 bne badsp
 move.l #$8003,($4420).l
 pflusha
 rte
badsp:
 move.w #1,($f100).l
 bra failed
badcount:
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
