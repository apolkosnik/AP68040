; Loop-target MOVE decode: partial Dn writes, X preservation, An updates,
; signed d16 displacement, and A7's special byte increment/decrement.
    org 0
    dc.l $3400,start
    rept 254
    dc.l fail
    endr
    org $400
start:
    move.l #$80008000,d0
    movec d0,cacr
    lea ($3f00).l,a0
    moveq #127,d1
fill:
    move.l #$5a5a5a5a,(a0)+
    dbra d1,fill

check macro
    move.l #$4000,\1
    move.l #$aabbccdd,d0
    moveq #7,d1
    move.w #$10,ccr
loop\@:
    move.\2 \3,d0
    dbra d1,loop\@
    move.w sr,d2
    andi.w #$1f,d2
    cmpi.w #$10,d2
    bne fail
    cmpi.l #\4,d0
    bne fail
    cmpa.l #\5,\1
    bne fail
    endm

    check a0,b,(a0),$aabbcc5a,$4000
    check a0,b,(a0)+,$aabbcc5a,$4008
    check a0,b,-(a0),$aabbcc5a,$3ff8
    check a0,b,16(a0),$aabbcc5a,$4000
    check a0,b,-16(a0),$aabbcc5a,$4000
    check a0,w,(a0),$aabb5a5a,$4000
    check a0,w,(a0)+,$aabb5a5a,$4010
    check a0,w,-(a0),$aabb5a5a,$3ff0
    check a0,w,16(a0),$aabb5a5a,$4000
    check a0,w,-16(a0),$aabb5a5a,$4000
    check a0,l,(a0),$5a5a5a5a,$4000
    check a0,l,(a0)+,$5a5a5a5a,$4020
    check a0,l,-(a0),$5a5a5a5a,$3fe0
    check a0,l,16(a0),$5a5a5a5a,$4000
    check a0,l,-16(a0),$5a5a5a5a,$4000
    check a7,b,(a7),$aabbcc5a,$4000
    check a7,b,(a7)+,$aabbcc5a,$4010
    check a7,b,-(a7),$aabbcc5a,$3ff0
    check a7,b,16(a7),$aabbcc5a,$4000
    check a7,b,-16(a7),$aabbcc5a,$4000
    move.w #$600d,($f102).l
    stop #$2700
fail:
    move.w #$efad,($f100).l
    move.w #$bad0,($f102).l
    stop #$2700
