; MC68040UM table 9-16: sqrt INEX uses E3/WBTEMP; move uses E1/FPTEMP.
; Exercise forced precision, FPCR precision controls and restored FSQRT $05.
FRAME   equ     $3000
RESTORE equ     $3400

check   macro
        cmp.l   #\2,\1
        bne     fail
        endm

sqrtcase macro
        addq.w  #1,d7
        fmove.l #\3,fpcr
op\@:
        \1.x    \2,fp0
        move.w  (op\@+2).l,d2
        cmp.b   #4,d2
        bne.s   mapped\@
        or.w    #1,d2            ; documented FSQRT command mapping
mapped\@:
        bsr     save_e3
        endm

movecase macro
        addq.w  #1,d7
        fmove.l #\3,fpcr
        lea     \4,a2           ; unrounded intermediate
op\@:
        \1.x    \2,fp0
        move.w  (op\@+2).l,d2
        bsr     save_e1
        endm

        org     0
        dc.l    $e000,start
        rept    254
        dc.l    fail
        endr
start:
        move.w  #$2700,sr
        moveq   #0,d7
        fmove.l #6,fp0
        fmovem.x two,fp1
        sqrtcase fssqrt,two,$200
        sqrtcase fdsqrt,two,$200
        sqrtcase fsqrt,two,$240
        sqrtcase fsqrt,two,$280
        sqrtcase fssqrt,fp1,$200
        sqrtcase fdsqrt,fp1,$200
        sqrtcase fsqrt,fp1,$240
        movecase fdmove,inexact,$200,inexact
        movecase fmove,inexact,$280,inexact
        movecase fsmove,inexact,$200,inexact
        movecase fdmove,tiny,$800,tiny
        movecase fmove,tiny,$880,tiny
        fmovem.x inexact,fp1
        movecase fdmove,fp1,$200,inexact
        movecase fsmove,fp1,$200,inexact
        fmovem.x tiny,fp1
        movecase fdmove,fp1,$800,tiny

        ; A handler resumes a BUSY frame with CU_SAVEPC=$FE. Its saved
        ; command is $05; executable $04 is also accepted as a control.
        lea     (RESTORE).l,a0
        moveq   #24,d0
zero:
        clr.l   (a0)+
        dbra    d0,zero
        move.l  #$41600000,(RESTORE).l
        move.l  #$fe000000,(RESTORE+8).l
        move.l  #$40000000,(RESTORE+88).l
        move.l  #$80000000,(RESTORE+92).l
        fmove.l #$200,fpcr
        move.l  #$48050000,(RESTORE+64).l
        move.w  #$4805,d2
        addq.w  #1,d7
        frestore (RESTORE).l
        bsr     save_e3
        move.l  #$48040000,(RESTORE+64).l
        addq.w  #1,d7
        frestore (RESTORE).l
        bsr     save_e3

        move.w  #$600d,($f102).l
        stop    #$2700

save_e3:
        lea     (FRAME+100).l,a1
        fsave   -(a1)
        check   a1,FRAME
        check   (FRAME).l,$41600000
        move.l  (FRAME+72).l,d0
        and.l   #$06100000,d0
        check   d0,$02000000
        cmp.w   (FRAME+64).l,d2
        bne     fail
        ; sqrt(2): intermediate in WBTEMP, no dyadic FPTEMP operand.
        check   (FRAME+24).l,$3fff0000
        move.l  (FRAME+28).l,d0
        and.l   #$ffff0000,d0
        check   d0,$b5040000
        check   (FRAME+76).l,0
        check   (FRAME+80).l,0
        check   (FRAME+84).l,0
        rts

save_e1:
        lea     (FRAME+52).l,a1
        fsave   -(a1)
        check   a1,FRAME
        check   (FRAME).l,$41300000
        move.l  (FRAME+24).l,d0
        and.l   #$06100000,d0
        check   d0,$04000000
        cmp.w   (FRAME+16).l,d2
        bne     fail
        ; INEX and UNFL moves keep the unrounded intermediate in FPTEMP.
        lea     (FRAME+28).l,a0
        moveq   #2,d1
compare:
        move.l  (a2)+,d0
        cmp.l   (a0)+,d0
        bne     fail
        dbra    d1,compare
        rts

fail:
        move.w  d7,($f100).l
        move.w  #$bad0,($f102).l
        stop    #$2700
two:    dc.l    $40000000,$80000000,0
inexact: dc.l   $3fff0000,$80000080,$00000400
tiny:   dc.l    $3c000000,$ffffffff,$fffffc00
