#!/bin/bash
# AP68040 regression: every program under five bus configurations (zero
# wait states, random wait states, waits with burst inhibit and retries,
# BCLK at half PCLK, and all of those together).
# Usage: run_tests.sh [program ...]   (default: the passing set)
# JOBS=<n> simulations run in parallel (default: half the CPUs).
set -e
cd "$(dirname "$0")"
PROGS=${*:-"smoke t_nx8k t_integer t_exceptions t_mmu t_cache t_atcprobe t_bitfield_cache t_bitfield_mmu t_movem_restart t_moves_fc t_fpu t_fpu_frames t_fpu_resume t_fpu_frame_class t_fpu_frame_snan t_fpu_frame_cmd t_snoop t_btb t_stld t_pins t_snstress t_snkill t_eredir t_cbsplit t_cmodes t_loops t_early t_irq7 t_iplglitch t_snirq t_xnest t_halt t_cache2 t_fpexc t_ipendsr audit_cas2_0 audit_cas2_1 audit_cas2_cache_0 audit_cas2_cache_1 audit_bf3_0 audit_bf3_1 audit_bf3_cache_0 audit_bf3_cache_1 audit_bf5_0 audit_bf5_1 audit_bf5_cache_0 audit_bf5_cache_1 audit_negx_0 audit_negx_1 audit_negx_mmufault audit_roxl_0 audit_roxl_1 audit_roxl_mmufault audit_push_0 audit_push_frame audit_rte_fault_0000 audit_rte_fault_3000 audit_rte_fault_3004 audit_rte_fault_3008 audit_rte_fault_300c audit_rte_throw_isp audit_rte_throw_msp audit_rte_throw_usp audit_rte_natural_0000 audit_rte_natural_300c audit_rte_prefetch_control audit_rte_mmu audit_rte_mmu_prefetch_control audit_iack_auto audit_rte_irq_throw audit_rte_irq_control audit_fpu_ea audit_fpu_boundary_complete audit_fpu_boundary_missing_cmd audit_irq_wrongpath audit_irq_vecpath audit_cpusha_raw t_dedis t_ptestu t_fmovecr_ea t_fpiar_cc"}
WORK=${WORK:-$(cd .. && pwd)/obj}; export WORK
JOBS=${JOBS:-$(( $(nproc) / 2 ))}
./build_asm.sh $PROGS
./build_sim.sh > build/sim_build.log 2>&1 || { tail -30 build/sim_build.log; exit 1; }
# the exhaustive decoder check (every operation word)
./build_decode.sh | grep -q "DECODE PASS" || { echo "DECODE FAIL"; exit 1; }
SIM=$WORK/obj_prog/tb_ap68040
CFGS=(
	""
	"+waits"
	"+waits +tbi=2 +retry=10"
	"+bclk2"
	"+bclk2 +waits +tbi=1 +retry=20"
)
out=$(for p in $PROGS; do for c in "${CFGS[@]}"; do printf '%s|%s\n' "$p" "$c"; done; done |
	xargs -d '\n' -P "$JOBS" -I{} bash -c '
		p=${1%%|*}; c=${1#*|}
		r=$('"$SIM"' +prog=build/$p.hex $c +timeout=8000000 2>&1 | grep -E "^(PASS|FAIL)" | tail -1)
		printf "%-16s %-34s %s\n" "$p" "${c:-(zero wait)}" "${r:-FAIL (no result)}"' _ {})
# the interrupt acknowledge with a supplied vector and ending in TEA
if [ -z "$*" ]; then
	IA="audit_iack_vector|+audit_ivec=255_+audit_dbgvec=255 audit_iack_spurious|+audit_iacktea_+audit_dbgvec=24 audit_irq_slowlo|+slowlo=40"
	./build_asm.sh audit_iack_vector audit_iack_spurious audit_irq_slowlo
	for pa in $IA; do
		p=${pa%%|*}; x=${pa#*|}; x=${x//_+/ +}
		for c in "${CFGS[@]}"; do
			r=$($SIM +prog=build/$p.hex $c $x +timeout=4000000 2>&1 | grep -E "^(PASS|FAIL)" | tail -1)
			out=$out$'\n'$(printf "%-16s %-34s %s" "$p" "${c:-(zero wait)} $x" "${r:-FAIL (no result)}")
		done
	done
fi
printf '%s\n' "$out" | sort
if printf '%s\n' "$out" | grep -qv " PASS "; then echo "FAILURES"; exit 1; fi
echo "ALL PASSED"
