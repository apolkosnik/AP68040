        org 0
        dc.l $3400,start,h_aerr
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
        lea ($4400).l,a0
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
        move.l #$4000,d0
        movec d0,srp
        movec d0,urp
        move.l #$8000,d0
        movec d0,tc
        clr.l ($8000).l
        clr.l ($8004).l
        clr.l ($8008).l
        clr.l ($800C).l
        move.l #$8023,($4420).l
        pflusha
        cinva dc
        move.l #$80000000,d0
        movec d0,cacr
        lea ($8000).l,a0
        move.l #$11223344,(a0)
        move.l #$55667788,4(a0)
        move.l #$99AABBCC,8(a0)
        move.l #$DDEEFF00,12(a0)
        cpushl dc,(a0)
        move.l (a0),($F194).l
        move.l 4(a0),($F198).l
        move.l 8(a0),($F19C).l
        move.l 12(a0),($F1B0).l
        cmpi.l #$11223344,(a0)
        bne fail
        cmpi.l #$55667788,4(a0)
        bne fail
        cmpi.l #$99AABBCC,8(a0)
        bne fail
        cmpi.l #$DDEEFF00,12(a0)
        bne fail
        move.w #$600D,($F102).l
        stop #$2700
h_aerr:
        addq.w #1,($3600).l
        cmpi.w #1,($3600).l
        bne unexpected
        move.l $14(sp),($F184).l
        move.w $c(sp),($F188).l
        move.l 44(sp),($F1A0).l
        move.l 48(sp),($F1A4).l
        move.l 52(sp),($F1A8).l
        move.l 56(sp),($F1AC).l
        rte
unexpected:
        move.w #99,($F100).l
        bra.s failed
fail:
        move.w #1,($F100).l
failed:
        move.w #$BAD0,($F102).l
        stop #$2700
