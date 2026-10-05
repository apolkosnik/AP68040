#!/bin/sh
# where.sh <program> <test number>: the source around a failing test
f=asm/$1.s
n=$(grep -nE "(,|failt[[:space:]]+)$2([[:space:]]*(;.*)?)?\$" "$f" | head -1 | cut -d: -f1)
[ -z "$n" ] && { echo "test $2 not found in $f"; exit 1; }
s=$((n - ${3:-15})); [ $s -lt 1 ] && s=1
sed -n "${s},$((n + 2))p" "$f"
