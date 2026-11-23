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
        move.l #$11111111,($7000).l
        move.l #$22222222,($8000).l
        lea ($7000).l,a0
        lea ($8000).l,a1
        move.l #$11111111,d0
        move.l #$22222222,d1
        move.l #$AAAAAAAA,d2
        move.l #$BBBBBBBB,d3
        move.l #$8007,($4420).l
        pflusha
        cas2.l d0:d1,d2:d3,(a0):(a1)
        move.l ($7000).l,($F194).l
        move.l ($8000).l,($F198).l
        cmpi.l #$AAAAAAAA,($7000).l
        bne fail
        cmpi.l #$BBBBBBBB,($8000).l
        bne fail
        move.w #$600D,($F102).l
        stop #$2700
h_aerr:
        addq.w #1,($3600).l
        cmpi.w #2,($3600).l
        bhi unexpected
        move.l (sp),($F180).l
        move.l $14(sp),($F184).l
        move.w $c(sp),($F188).l
        move.l ($7000).l,($F18C).l
        move.l ($7FFC).l,($F190).l
        move.l #$8003,($4420).l
        pflusha
        rte
unexpected:
        move.w #99,($F100).l
        move.w #$BAD0,($F102).l
        stop #$2700
fail:
        move.w #1,($F100).l
        move.w #$BAD0,($F102).l
        stop #$2700
