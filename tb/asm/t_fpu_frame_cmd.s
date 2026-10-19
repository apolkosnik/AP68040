; CMDREG1B retains the original opclass and source register/format through
; F_EXEC, the direct F_BIN/F_ROUND paths, and destination datatype traps.
FRAME   equ     $3000

check   macro
        cmp.l   #\2,\1
        bne     fail
        endm
commandcase macro
        addq.w  #1,d7
        fmove.l #\3,fpcr
op\@:
        \1.x    \2,fp0
        fsave   (FRAME).l
        check   (FRAME).l,$41300000
        move.w  (op\@+2).l,d0
        cmp.w   (FRAME+16).l,d0
        bne     fail
        endm
datatypecase macro
        addq.w  #1,d7
        fmovem.x denormal,fp0
        lea     after\@,a3
        lea     op\@,a4
op\@:
        \1.x    \2,fp0
        bra     fail
after\@:
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
        fmovem.x snan,fp1
        ; F_EXEC, and F_EXEC -> F_BIN (source SNaN).
        commandcase fcmp,fp1,$4000
        commandcase fadd,fp1,$4000
        commandcase fcmp,snan,$4000
        ; F_BIN directly, source is ordinary zero.
        fmove.l #0,fp3
        commandcase fdiv,fp3,$400
        commandcase fdiv,zero,$400
        ; F_ROUND directly, source index deliberately differs from fmt X.
        fmovem.x inexact,fp5
        commandcase fdmove,fp5,$200
        commandcase fdmove,inexact,$200
        ; Capture by F_CAP after finding an unsupported destination, for
        ; both FCMP in F_EXEC and arithmetic in F_BIN.
        move.l  #datatype_handler,($dc).w
        fmove.l #6,fp1
        datatypecase fcmp,fp1
        datatypecase fadd,fp1
        datatypecase fcmp,six
        datatypecase fadd,six
        move.w  #$600d,($f102).l
        stop    #$2700

datatype_handler:
        fsave   (FRAME).l
        check   (FRAME).l,$41600000
        move.w  2(a4),d0
        cmp.w   (FRAME+64).l,d0
        bne     fail
        move.l  a3,2(sp)
        rte
fail:
        move.w  d7,($f100).l
        move.w  #$bad0,($f102).l
        stop    #$2700
snan:   dc.l    $7fff0000,$a0000000,1
zero:   dc.l    0,0,0
six:    dc.l    $40010000,$c0000000,0
inexact: dc.l   $3fff0000,$80000080,$00000400
denormal: dc.l  0,$40000000,0
