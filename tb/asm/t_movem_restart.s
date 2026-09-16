; Format-7 CM/saved-EA restart, including a MOVEM which overwrites its own
; memory-indirect pointer before faulting. All cases run in every bus phase.
FAILREG         equ $F100
DONEREG         equ $F102
which           equ $3600
faults          equ $3602
nested          equ $3604
irqs            equ $3606

        org     0
        dc.l    $3400,start,h_aerr
        rept    23
        dc.l    unexpected
        endr
        dc.l    h_int2
        rept    229
        dc.l    unexpected
        endr

        org     $400
start:
        move.w  #$2700,sr
        clr.w   (which).l
next_case:
        move.w  #$2700,sr
        moveq   #0,d0
        movec   d0,tc
        pflusha
        clr.w   (faults).l
        clr.w   (nested).l
        clr.w   (irqs).l
        lea     ($4400).l,a0
        moveq   #63,d1
tables:
        move.l  d0,d2
        lsl.l   #8,d2
        lsl.l   #4,d2
        addq.l  #3,d2
        move.l  d2,(a0)+
        addq.l  #1,d0
        dbra    d1,tables
        move.l  #$4203,($4000).l
        move.l  #$4403,($4200).l
        move.l  #$6FF8,($6FF8).l
        move.l  #$BBBBBBBB,($7000).l
        clr.l   ($6008).l
        clr.l   ($6018).l
        clr.l   ($6108).l
        clr.l   ($441C).l             ; page 7 invalid
        clr.l   ($4430).l             ; nested handler's page 12 invalid
        move.w  #$48F0,($D000).l      ; MOVEM.L D1-D3,([A0]); RTS
        move.w  #$000E,($D002).l
        move.w  #$0151,($D004).l
        move.w  #$4E75,($D006).l
        lea     ($6FF8).l,a0
        cmpi.w  #5,(which).l
        bne.s   not_displaced
        lea     ($6FE8).l,a0
        move.l  #$6FE8,($6FF8).l      ; BD=16, OD=16 -> original EA 6FF8
not_displaced:
        cmpi.w  #6,(which).l
        bne.s   enable
        move.l  #$7003,($441C).l      ; operand writes do not fault this time
        clr.l   ($4424).l             ; fault while reading the EA pointer
        move.l  #$6FF8,($9000).l
        lea     ($9000).l,a0
enable:
        move.l  #$4000,d0
        movec   d0,urp
        movec   d0,srp
        move.l  #$8000,d0
        movec   d0,tc
        pflusha
        move.l  #$6000,d1
        move.l  #$22222222,d2
        move.l  #$33333333,d3
        cmpi.w  #8,(which).l
        bne.s   local_movem
        jsr     ($D000).l             ; handler will invalidate its instruction page
        bra.s   check
local_movem:
        cmpi.w  #5,(which).l
        beq.s   displaced
        movem.l d1-d3,([a0])          ; 48F0 000E 0151
        bra.s   check
displaced:
        movem.l d1-d3,([16.w,a0],16.l)
check:
        moveq   #0,d0
        movec   d0,tc
        pflusha
        moveq   #1,d0
        cmpi.w  #1,(which).l
        beq.s   twice
        cmpi.w  #8,(which).l
        bne.s   count_ready
twice:
        moveq   #2,d0
count_ready:
        cmp.w   (faults).l,d0
        bne     fail_count
        cmpi.w  #2,(which).l
        bne.s   check_dest
        cmpi.w  #1,(nested).l
        bne     fail_nested
check_dest:
        cmpi.w  #7,(which).l
        bne.s   irq_checked
        cmpi.w  #1,(irqs).l
        bne     fail_irq
irq_checked:
        cmpi.w  #3,(which).l
        beq.s   edited
        cmpi.w  #4,(which).l
        beq.s   cleared
        cmpi.l  #$33333333,($7000).l
        bne     fail_dest
        bra.s   no_stray
edited:
        cmpi.l  #$33333333,($6108).l  ; handler changed the stacked EA
        bne     fail_dest
        bra.s   untouched
cleared:
        cmpi.l  #$33333333,($6008).l  ; clearing CM explicitly recomputes EA
        bne     fail_dest
untouched:
        cmpi.l  #$BBBBBBBB,($7000).l
        bne     fail_dest
        cmpi.w  #4,(which).l
        beq.s   done_case
no_stray:
        tst.l   ($6008).l
        bne     fail_stray
        tst.l   ($6018).l
        bne     fail_stray
done_case:
        addq.w  #1,(which).l
        cmpi.w  #9,(which).l
        bne     next_case
        move.w  #$600D,(DONEREG).l
        stop    #$2700

h_aerr:
        movem.l d0/a1,-(sp)          ; frame offsets below include 8-byte save
        cmpi.w  #$7008,14(sp)
        bne     unexpected
        cmpi.l  #$C000,28(sp)
        beq     inner
        addq.w  #1,(faults).l
        cmpi.w  #2,(faults).l
        bhi     fail_count           ; bounded even if restart loops
        move.w  20(sp),d0
        cmpi.w  #6,(which).l
        beq     pointer_fault
        btst    #12,d0
        beq     fail_cm
        cmpi.l  #$6FF8,16(sp)
        bne     fail_ea
        cmpi.w  #8,(which).l
        bne.s   operand_fault
        cmpi.w  #2,(faults).l
        bne.s   operand_fault
        cmpi.l  #$D000,28(sp)
        bne     fail_fa
        move.l  #$D003,($4434).l      ; instruction refault preserved outer CM/EA
        bra     flush
operand_fault:
        cmpi.l  #$7000,28(sp)
        bne     fail_fa
        cmpi.w  #1,(which).l
        bne.s   repair
        cmpi.w  #1,(faults).l
        beq     return               ; refault with the still-invalid ATC entry
repair:
        move.l  #$7003,($441C).l
        cmpi.w  #8,(which).l
        bne.s   arm_irq
        clr.l   ($4434).l             ; fault fetching the resumed instruction
arm_irq:
        cmpi.w  #7,(which).l
        bne.s   nest_check
        andi.w  #$F8FF,8(sp)          ; returning SR unmasks level 2
        move.w  #2,($F110).l
nest_check:
        cmpi.w  #2,(which).l
        bne.s   edit_frame
        move.l  ($C000).l,d0          ; nested access error must not lose outer EA
edit_frame:
        cmpi.w  #3,(which).l
        bne.s   clear_cm
        move.l  #$6100,16(sp)
clear_cm:
        cmpi.w  #4,(which).l
        bne.s   flush
        andi.w  #$EFFF,20(sp)
        bra.s   flush
pointer_fault:
        btst    #12,d0
        bne     fail_cm               ; EA calculation is not a MOVEM transfer
        cmpi.l  #$9000,28(sp)
        bne     fail_fa
        move.l  #$9003,($4424).l
flush:
        pflusha
return:
        movem.l (sp)+,d0/a1
        rte
inner:
        move.w  20(sp),d0
        btst    #12,d0
        bne     fail_cm
        addq.w  #1,(nested).l
        cmpi.w  #1,(nested).l
        bne     fail_nested
        move.l  #$C003,($4430).l
        bra.s   flush

h_int2:
        ; CM resumes the interrupted instruction before a pending interrupt.
        cmpi.l  #$33333333,($7000).l
        bne     fail_irq
        addq.w  #1,(irqs).l
        clr.w   ($F110).l
        rte

fail_count:     moveq #1,d7
                bra.s fail
fail_nested:    moveq #2,d7
                bra.s fail
fail_dest:      moveq #3,d7
                bra.s fail
fail_stray:     moveq #4,d7
                bra.s fail
fail_cm:        moveq #5,d7
                bra.s fail
fail_ea:        moveq #6,d7
                bra.s fail
fail_fa:        moveq #7,d7
                bra.s fail
fail_irq:       moveq #8,d7
                bra.s fail
unexpected:    moveq #15,d7
fail:
        move.w  d7,(FAILREG).l
        move.w  #$BAD0,(DONEREG).l
halt:   bra.s   halt
