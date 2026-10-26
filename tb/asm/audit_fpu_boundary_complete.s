; Audit: ignored FP EA extensions at a translation boundary.
 org 0
 dc.l $3400,start,h_access
 rept 253
 dc.l failed
 endr
 org $400
start:
 move.w #$2700,sr
 moveq #0,d0
 movec d0,cacr
 movec d0,tc
 lea $4400,a0
 moveq #63,d1
tables:
 move.l d0,d2
 lsl.l #8,d2
 lsl.l #4,d2
 addq.l #3,d2
 move.l d2,(a0)+
 addq.l #1,d0
 dbra d1,tables
 move.l #$4203,($4000).l
 move.l #$4403,($4200).l
 clr.l ($4420).l
 move.l #$4000,d0
 movec d0,srp
 move.l #$8000,d0
 movec d0,tc
 pflusha
 jmp ($7ffc).l
h_access:
 move.l 2(sp),($f180).l
 move.l 20(sp),($f184).l
 cmpi.l #$8000,2(sp)
 bne failed
 cmpi.l #$8000,20(sp)
 bne failed
 move.w #$600d,($f102).l
 stop #$2700
failed:
 move.w #99,($f100).l
 move.w #$bad0,($f102).l
 stop #$2700
 org $7ffc
 dc.w $f22a,$0000
