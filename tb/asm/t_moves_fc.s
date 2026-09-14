; MOVES must latch SFC/DFC before the fast memory-issue path runs.
; NeXTSTEP copyinstr read a NUL from supervisor ROM instead of the user
; pathname when its MOVES.B source read issued before SFC took effect.
; Keep both caches and translation enabled, and let the prefetch queue
; fill during a register shift so the data port is free on MOVES issue.
;
; VA $2000: supervisor -> PA $2000 (zero); user -> PA $A000 ("/etc/init").
; Supervisor VA $A000 aliases the user page to verify writes independently.

FAILREG equ $F100
DONEREG equ $F102

check macro
        beq.s   ok\@
        move.w  #\1,d7
        bra     fail
ok\@:
        endm

        org     0
        dc.l    $3400,start
        rept    254
        dc.l    fail_exception
        endr
        org     $400
start:
        move.w  #$2700,sr
        lea     ($4400).l,a0
        lea     ($4C00).l,a1
        moveq   #0,d0
        moveq   #31,d1
tables:
        move.l  d0,d2
        lsl.l   #8,d2
        lsl.l   #5,d2
        addq.l  #3,d2
        move.l  d2,(a0)+
        move.l  d2,(a1)+
        addq.l  #1,d0
        dbra    d1,tables
        move.l  #$4203,($4000).l
        move.l  #$4403,($4200).l
        move.l  #$4A03,($4800).l
        move.l  #$4C03,($4A00).l
        move.l  #$A003,($4C04).l
        clr.l   ($2000).l
        move.l  #$2F657463,($A000).l
        move.l  #$2F696E69,($A004).l
        move.w  #$7400,($A008).l
        move.l  #$4000,d0
        movec   d0,srp
        move.l  #$4800,d0
        movec   d0,urp
        move.l  #$C000,d0
        movec   d0,tc
        move.l  #$80008000,d0
        movec   d0,cacr
        moveq   #1,d0
        movec   d0,sfc
        movec   d0,dfc
        lea     ($2000).l,a0
        moveq   #31,d2
loop:
        moveq   #63,d3
        lsl.l   d3,d4
        moves.b (a0),d1
        cmp.b   #$2F,d1
        check   1
        lea     ($A000).l,a1
        moveq   #$55,d1
        lsl.l   d3,d4
        moves.b d1,(a0)
        cmp.b   #$55,(a1)
        check   2
        tst.l   (a0)
        check   3
        move.b  #$2F,(a1)

        lsl.l   d3,d4
        moves.w (a0),d1
        cmp.w   #$2F65,d1
        check   4
        move.w  #$1234,d1
        lsl.l   d3,d4
        moves.w d1,(a0)
        cmp.w   #$1234,(a1)
        check   5
        tst.l   (a0)
        check   6
        move.w  #$2F65,(a1)

        lsl.l   d3,d4
        moves.l (a0),d1
        cmp.l   #$2F657463,d1
        check   7
        move.l  #$12345678,d1
        lsl.l   d3,d4
        moves.l d1,(a0)
        cmp.l   #$12345678,(a1)
        check   8
        tst.l   (a0)
        check   9
        move.l  #$2F657463,(a1)
        dbra    d2,loop

        move.w  #$600D,(DONEREG).l
        stop    #$2700
fail_exception:
        move.w  #99,d7
fail:
        move.w  d7,(FAILREG).l
        move.w  #$BAD0,(DONEREG).l
        stop    #$2700
