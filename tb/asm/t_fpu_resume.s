; FPSP-prepared 68040 BUSY commands: execute ETEMP/FPTEMP, not live FP regs.
        ifd     REV40
BUSY    equ     $40600000
IDLE    equ     $40000000
        else
BUSY    equ     $41600000
IDLE    equ     $41000000
        endif
FRAME   equ     $3000
COUNT   equ     $3800

check   macro
        cmp.l   #\2,\1
        beq.s   ok\@
        move.w  #\3,($f100).l
        jmp     fail
ok\@:
        endm
source  macro
        move.l  #\1,(FRAME+88).l
        move.l  #\2,(FRAME+92).l
        move.l  #\3,(FRAME+96).l
        endm
dest    macro
        move.l  #\1,(FRAME+76).l
        move.l  #\2,(FRAME+80).l
        move.l  #\3,(FRAME+84).l
        endm
command macro
        move.l  #\1,(FRAME+64).l
        endm
run     macro
        lea     (FRAME).l,a1
        frestore (a1)+
        check   a1,FRAME+100,90
        endm
result  macro
        fmove.l fp1,d0
        check   d0,\1,\2
        endm

        org     0
        dc.l    $e000,start
        rept    254
        dc.l    fail
        endr
start:
        move.w  #$2700,sr
        move.l  #dz_handler,($c8).w
        clr.l   (COUNT).l
        bsr     prepare
        ; FE and opclass 2: 2+1 -> FP1, not the live 99+1.
        command $48a20000
        run
        result  3,1
        ; Opclass 0 also uses ETEMP, not the encoded FP source register.
        command $00a20000
        run
        result  3,2
        ; No CU_SAVEPC request: do not replay an ordinary saved frame.
        clr.l   (FRAME+8).l
        fmove.l #99,fp1
        run
        result  99,3
        move.l  #$fd000000,(FRAME+8).l
        run
        result  99,4
        move.l  #$fe000000,(FRAME+8).l
        ; Register stores are not arithmetic resumption requests.
        command $68a20000
        run
        result  99,5

        ; Monadic move ignores stale live and frame destination data.
        command $48800000
        source  0,0,0
        run
        result  0,6
        source  $40000000,$c0000000,0 ; 3
        command $48a30000
        run
        result  6,7
        command $48a80000           ; FPTEMP 2 - ETEMP 3
        run
        result  -1,8
        dest    $40010000,$c0000000,0 ; 6 / 3
        command $48a00000
        run
        result  2,9
        source  $40020000,$90000000,0 ; sqrt(9), internal command 5
        command $48850000
        run
        result  3,10

        ; FCMP/FTST update CC without replacing the destination register.
        source  $3fff0000,$80000000,0 ; 1
        dest    $40000000,$80000000,0 ; 2
        command $48b80000
        fmove.l #99,fp1
        run
        fmove.l fpsr,d0
        and.l   #$0f000000,d0
        check   d0,0,11
        result  99,12
        source  $80000000,0,0       ; negative zero
        command $48ba0000
        run
        fmove.l fpsr,d0
        and.l   #$0f000000,d0
        check   d0,$0c000000,13
        result  99,14

        ; ET15: internal exponent -1 denormalizes by one bit. FTST must
        ; see a positive finite nonzero, not Infinity or NaN (raw e=7fff).
        source  $7fff0000,$80000000,0
        move.l  #$10000000,(FRAME+60).l
        run
        fmove.l fpsr,d0
        and.l   #$0f000000,d0
        check   d0,0,15
        ; Both operands exponent -1: division must normalize both -> 1.
        dest    $7fff0000,$80000000,0
        move.l  #$10000000,(FRAME+68).l
        command $48a00000
        run
        result  1,16
        ; Different tiny magnitudes: (2^-1)/(2^-2) -> 2.
        source  $7ffe0000,$80000000,0
        run
        result  2,17
        ; Far-negative internal exponent truncates all bits, preserving sign.
        source  $ffc00000,$ffffffff,$ffffffff
        command $48800000
        run
        fmove.l fpsr,d0
        and.l   #$0f000000,d0
        check   d0,$0c000000,18

        ; New execution clears old exception status, retaining accrued flags
        ; and the caller-restored FPIAR; frame FPIARCU is not that register.
        bsr     prepare
        fmove.l #$12345678,fpiar
        fmove.l #$0080ff88,fpsr
        command $48a20000
        run
        fmove.l fpsr,d0
        check   d0,$00800088,19
        fmove.l fpiar,d0
        check   d0,$12345678,20

        ; Enabled arithmetic exception from a resumed command is deferred,
        ; inhibits FP1 writeback, and is raised at the following FP dispatch.
        bsr     prepare
        source  0,0,0
        command $48a00000
        fmove.l #$400,fpcr
        run
        move.l  (COUNT).l,d0
        check   d0,0,21
        fnop
        move.l  (COUNT).l,d0
        check   d0,1,22
        result  99,23
        fmove.l #0,fpcr
        ; IDLE following BUSY must not inherit a resume request.
        move.l  #IDLE,(FRAME).l
        frestore (FRAME).l
        fnop
        move.l  (COUNT).l,d0
        check   d0,1,24

        ; Motorola FPSP get_op/bugfix sets ETE15/FPTE15 for ordinary
        ; exponents below $4000 as well as wrapped negative exponents.
        ; The flag alone must not denormalize 0.5 or 1.0 into zero.
        bsr     prepare
        source  $3ffe0000,$80000000,0 ; 0.5
        dest    $3fff0000,$80000000,0 ; 1.0
        move.l  #$10000000,(FRAME+60).l
        move.l  #$10000000,(FRAME+68).l
        command $48a20000
        run
        fmove.s fp1,d0
        check   d0,$3fc00000,25     ; 1.5, not zero
        source  $bffe0000,$80000000,0 ; -0.5
        run
        fmove.s fp1,d0
        check   d0,$3f000000,26     ; 0.5, preserve the operand sign
        source  $3ffe0000,$80000000,0
        dest    $3ffe0000,$80000000,0
        clr.l   (FRAME+60).l
        run
        result  1,27              ; destination flag must not zero 0.5
        source  $3fff0000,$80000000,0
        dest    $40000000,$80000000,0
        move.l  #$10000000,(FRAME+60).l
        clr.l   (FRAME+68).l
        run
        result  3,28              ; exponent boundary: 1 + 2
        check   sp,$e000,91
        move.w  #$600d,($f102).l
        bra.s   *
prepare:
        lea     (FRAME).l,a0
        moveq   #24,d0
clear:
        clr.l   (a0)+
        dbra    d0,clear
        move.l  #BUSY,(FRAME).l
        move.l  #$fe000000,(FRAME+8).l
        source  $3fff0000,$80000000,0
        dest    $40000000,$80000000,0
        fmove.l #0,fpcr
        fmove.l #0,fpsr
        fmove.l #99,fp1
        rts
dz_handler:
        addq.l  #1,(COUNT).l
        rte
fail:
        move.w  #$bad0,($f102).l
        bra.s   *
