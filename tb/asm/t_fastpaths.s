; Sequencer ports from MacQuadra800: load/use dependencies, short DBcc,
; queued extensions, and silicon-defined index-suppressed indirect EAs.
FAILREG equ $F100
DONEREG equ $F102

check   macro
        cmp.l   #\2,\1
        beq.s   ok\@
        move.w  #\3,d7
        bra     fail
ok\@:
        endm

        org     0
        dc.l    $3400,start
        rept    254
        dc.l    unexpected
        endr

        org     $400
start:
        move.l  #$80008000,d0
        movec   d0,cacr
        move.l  #$80ff8000,($3000).l
        move.l  #$11223344,($3004).l
        move.l  #$55667788,($3008).l
        lea     ($3000).l,a0

        ; Partial register writes, X preservation and immediate consumers.
        move.l  #$aabbccdd,d0
        move.w  #$10,ccr
        move.b  (a0),d0
        move.w  ccr,d1
        check   d0,$aabbcc80,1
        and.l   #$1f,d1
        check   d1,$18,2
        move.w  2(a0),d0
        check   d0,$aabb8000,3
        move.w  #$10,ccr
        movea.w 2(a0),a1
        move.w  ccr,d1
        check   a1,$ffff8000,4
        and.l   #$1f,d1
        check   d1,$10,5

        ; A memory-result producer feeds the following register instruction.
        move.l  4(a0),d0
        add.l   d0,d0
        check   d0,$22446688,6
        moveq   #-1,d0
        add.l   4(a0),d0
        move.w  ccr,d1
        check   d0,$11223343,7
        and.l   #$1f,d1
        check   d1,$11,8
        cmp.l   4(a0),d0
        bcs.s   compare_ok
        moveq   #9,d7
        bra     fail
compare_ok:
        check   d0,$11223343,10

        ; A load destination can alias its postincremented address register.
        movea.w (a0)+,a0
        check   a0,$ffff80ff,11
        lea     ($3000).l,a0
        move.l  (a0)+,d0
        move.l  (a0)+,d1
        check   d0,$80ff8000,12
        check   d1,$11223344,13
        check   a0,$3008,14

        ; DBcc reads the latest Dn and preserves its high half and CCR.
        move.l  #$abcd0003,d0
        moveq   #0,d1
        move.w  #$15,ccr
count:
        lea     (a1),a1             ; no CCR update
        dbra    d0,count
        move.w  ccr,d2
        check   d0,$abcdffff,15
        and.l   #$1f,d2
        check   d2,$15,16
        move.l  #$12340002,d0
        dbt     d0,unexpected
        check   d0,$12340002,17

        ; The immediate's low word is on a new instruction page.
        jsr     ($ffc).l
        check   d0,$12345678,18

        ; Captured Quadra encodings: index suppressed, I/IS=101/110/111.
        ; The index register's nonzero value must be ignored.
        lea     ($3100).l,a6
        move.l  #$3004,(a6)
        move.l  #$deadbeef,d0
        dc.w    $2236,$0165,$0000   ; MOVE.L ([bd.W=0,A6]),D1
        check   d1,$11223344,19
        dc.w    $2236,$0166,$0000,$0004
        check   d1,$55667788,20
        dc.w    $2236,$0167,$0000,$0000,$0004
        check   d1,$55667788,21

        ; Four-word loop exercises branch retention, then invalidation.
        moveq   #0,d0
        moveq   #4,d1
        jsr     ($1200).l
        check   d0,5,22
        move.w  #$5380,($1200).l     ; SUBQ.L #1,D0
        cinva   ic
        moveq   #0,d0
        moveq   #6,d1
        jsr     ($1200).l
        check   d0,$fffffff9,23

        ; CACR remains enabled, but this instruction window is uncached.
        ; Rewriting the loop now needs neither CINV nor a branch-buffer flush.
        move.l  #$0000c040,d0
        movec   d0,itt0
        move.w  #$5280,($1200).l     ; ADDQ.L #1,D0
        moveq   #0,d0
        moveq   #2,d1
        jsr     ($1200).l
        check   d0,3,24
        move.w  #$5380,($1200).l
        moveq   #0,d0
        moveq   #2,d1
        jsr     ($1200).l
        check   d0,$fffffffd,25

        ; Change an uncached loop WHILE it runs. The taken DBcc must fetch
        ; the replacement opcode, even if the previous iteration filled a
        ; branch sector. First iteration adds one; the next two subtract.
        lea     ($1300).l,a0
        moveq   #0,d0
        moveq   #2,d1
        jsr     (a0)
        check   d0,$ffffffff,26

        move.w  #$600d,(DONEREG).l
        stop    #$2700
unexpected:
        move.w  #99,d7
fail:
        move.w  d7,(FAILREG).l
        move.w  #$bad0,(DONEREG).l
        bra.s   *

        org     $ffc
        move.l  #$12345678,d0
        rts

        org     $1200
cached_loop:
        addq.l  #1,d0
        dbra    d1,cached_loop
        rts

        org     $1300
uncached_loop:
        addq.l  #1,d0
        move.w  #$5380,(a0)         ; replaces ADDQ with SUBQ
        dbra    d1,uncached_loop
        rts
