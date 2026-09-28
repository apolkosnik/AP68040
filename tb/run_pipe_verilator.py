#!/usr/bin/env python3
"""Run the pipelined core's milestone benches under Verilator.

This replaces the milestone-17 tb/run_pipe_tests.sh, which drove iverilog
over a fixed twelve-file list. The pipelined core is developed and run
under Verilator only. The milestone benches each poke their own program
into ap040_inst_fetch.v's ROM at time 0, so they need no assembler and no
4-state X. tb_ap040_pipe_program and tb_ap040_pipe_program_local run
tb/asm's self-checking programs, which are assembled here with
vasmm68k_mot (VASM, else on PATH); dhry is C and comes from tb/build/, so
run ./build_tests.sh first.

The benches are the Minimig-AGA tree's tests/ap040/pipe/ unchanged, and
their comments give paths in that tree's layout: rtl/ap040_pipe/ is rtl/
here, rtl/ap040/ is rtl_old/, tests/ap040/pipe/ and tests/ap040/ are tb/,
and sim_dpram.v is rtl_old/primitives/dpram.v. That tree also runs its
cpu_wrapper.v boot bench on this core (tb_cpu_wrapper_boot_bridge); that
one needs the Amiga chipset's sources and stays there.

tb_ap040_pipe_l1_wbuf tests ap040_pipe_l1.v standalone, so it gets only
that file; compiling it against the full list would put two
top-level-instantiable modules in one unit. tb_ap040_pipe_alu_equiv
compares ap040_pipe_alu.v against the FSM core's rtl_old/ap040_alu.v,
so it gets those two files and both include directories.
"""
import argparse, os, shutil, subprocess, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
RTL = ROOT / "rtl"
# the sequential core: the pipelined one runs its FPU engine, 16-bit bus
# adapter and defines, and the differential benches run all of it
OLD = ROOT / "rtl_old"
DPRAM = OLD / "primitives/dpram.v"
TB = HERE

CORE = [RTL / n for n in (
    "ap040_pipe_core.v", "ap040_pipe_cpu.v", "ap040_pipe_sys.v",
    "ap040_pipe_membus.v", "ap040_pipe_imu.v", "ap040_inst_fetch.v", "ap040_decode.v",
    "ap040_ea_calc.v", "ap040_ea_fetch.v", "ap040_execute.v",
    "ap040_writeback.v", "ap040_pipe_alu.v", "ap040_pipe_regfile.v",
    "ap040_pipe_l1.v", "ap040_pipe_fpu.v", "ap040_pipe_irq.v",
    # the IMU's instruction cache arrays, and their block RAMs' models
    "ap040_pipe_cache_arr.v")] + [OLD / "ap040_fpu.v", HERE / "sim_pipe_ram.v"]
# tb_ap040_pipe_program runs tb/asm's self-checking programs, the ones
# run_tests.sh runs on the sequential core. REQUIRED
# must each print ALL TESTS PASSED; OPEN run too and are reported with the
# gap that holds them, so a known gap is visible on every run without
# hiding a regression anywhere else -- none is open since the MMU
# (bundle 10). bench_* are the sequential core's no-cache exclusions as
# well (run_verilator.py). t_cache runs since this core has its data cache
# (caches stage C). t_icache is this core's alone: the sequential core
# widens every CINV/CPUSH to all lines, and t_icache tests that a line or
# page operation leaves the others.
PROGRAMS_REQUIRED = ["t_integer", "t_fastpaths", "t_fpu", "t_fpu_frames", "t_fpu_resume", "t_cinv_moves", "dhry",
                     "t_exceptions", "t_moves_fc", "t_mmu", "t_bitfield_mmu", "t_bitfield_cache", "t_atcprobe",
                     "t_movem_restart", "t_fault_edges", "t_agu", "t_walk_order", "t_moves_alt", "t_smc_mmu",
                     "t_icache", "t_cache", "t_dcache", "t_copyback", "t_wberr", "t_snoop", "t_reset"]
PROGRAMS_OPEN = {}
# tb_ap040_pipe_program_local runs the programs that need no bus devices and
# no MMU on ap040_pipe_core.v, whose one-cycle array feeds decode two words a
# cycle (phase 8) -- see the bench's header.
PROGRAMS_LOCAL = ["t_integer", "t_fastpaths", "t_agu", "t_fpu_frames", "t_fpu_resume", "dhry",
                  "bench_alu", "bench_loop"]
VASM = Path(os.environ.get("VASM") or shutil.which("vasmm68k_mot") or "vasmm68k_mot")


def program_image(name, work):
    """Assemble tb/asm/<name>.s into work, or fall back to the image
    build_tests.sh leaves in tb/build/ (dhry is C, built only there)."""
    src = HERE / "asm" / (name + ".s")
    if src.exists() and VASM.exists():
        b = work / (name + ".bin")
        h = work / (name + ".hex")
        if subprocess.run([str(VASM), "-quiet", "-Fbin", "-m68040", "-no-opt", "-o", str(b), str(src)]).returncode == 0:
            subprocess.run([sys.executable, str(HERE / "bin2hex.py"), str(b), str(h)], stdout=subprocess.DEVNULL)
            return h
    return HERE / "build" / (name + ".hex")


# ap040_pipe_bus16.v carries the data memory unit and the MMU with a port
# for each memory port (ap040_pipe_mmu.v, whose ATC is dpram rows:
# rtl_old/primitives/dpram.v).
BUS16 = [RTL / "ap040_pipe_bus16.v", OLD / "ap040_bus16_adapter.v",
         RTL / "ap040_pipe_dmu.v", RTL / "ap040_pipe_mmu.v", DPRAM]
# ap040_pipe_fpu.v runs the shared FPU engine, which includes rtl_old's
# ap040_defs.svh, so every build takes that directory too.


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--work", type=Path, default=HERE / "build/pipe")
    ap.add_argument("--jobs", type=int, default=8)
    ap.add_argument("--only", help="comma-separated bench names")
    ap.add_argument("--keep-obj", action="store_true",
                    help="keep every bench's Verilator build directory. The default deletes "
                         "one as soon as its bench PASSES: a full suite leaves about 10 GB of "
                         "precompiled headers and object files behind otherwise, ninety "
                         "milestones of that filled a 921 GB disk, and a full disk fails the "
                         "g++ step with no diagnostic at all -- which reads as a suite-wide "
                         "RTL regression. A FAILING bench keeps its directory either way, "
                         "since that is when the build and the binary are worth having.")
    ap.add_argument("--ce-random", action="store_true",
                    help="drive every bench's clock enable from a pseudo-random sequence "
                         "instead of tying it high. ce is how the real system runs this core "
                         "slower than its clock, so a cycle with it low is a cycle that did "
                         "not happen -- and eight of the thirteen defects three rounds of "
                         "external review found lived behind every bench tying it high.")
    ap.add_argument("--slow-l1", action="store_true",
                    help="build with AP040_PIPE_L1_SLOW: 0-3 extra cycles on every L1 read and "
                         "write-buffer drain, so the benches prove the pipeline waits for memory")
    args = ap.parse_args()
    work = args.work.resolve(); work.mkdir(parents=True, exist_ok=True)
    # g++ writes its temporaries to TMPDIR, which on this machine is a 31 GB
    # tmpfs shared with everything else. A full one makes the compiler exit
    # with no diagnostic, which this harness reports as a failed bench -- the
    # same false regression a full /home produced at milestone 90, from a
    # different disk. The work directory is on real storage and is where the
    # build already lives, so it holds the temporaries too.
    env = dict(os.environ, TMPDIR=str(work))

    benches = [(b, b.stem, []) for b in sorted(TB.glob("tb_ap040_pipe_*.v"))]
    if args.only:
        want = set(args.only.split(","))
        benches = [x for x in benches if x[1] in want or
                   x[1].replace("tb_ap040_pipe_", "") in want]

    passed, failed = [], []
    for b, name, params in benches:
        src, inc = CORE, [RTL, OLD]
        defines, runs = [], None
        if name.endswith("l1_wbuf"):
            src = [RTL / "ap040_pipe_l1.v"]
        elif name.endswith("rmwsup"):
            # ap040_pipe_cpu.v with the memory written in the bench, so the
            # write port can be held busy -- see the bench's header.
            src = [x for x in CORE if x.name not in
                   ("ap040_pipe_core.v", "ap040_pipe_sys.v",
                    "ap040_pipe_membus.v", "ap040_pipe_imu.v", "ap040_pipe_l1.v",
                    "ap040_pipe_cache_arr.v", "sim_pipe_ram.v")]
        elif name.endswith("dmuport"):
            # the data and instruction memory units, the MMU and the bus
            # controller, driven at the CPU's ports -- see the bench's header
            src = [RTL / "ap040_pipe_dmu.v", RTL / "ap040_pipe_mmu.v", RTL / "ap040_pipe_imu.v",
                   RTL / "ap040_pipe_membus.v", RTL / "ap040_pipe_cache_arr.v", HERE / "sim_pipe_ram.v",
                   DPRAM]
        elif name.endswith("dcache"):
            # the DMU, the MMU and the bus controller, as dmuport
            src = [RTL / "ap040_pipe_dmu.v", RTL / "ap040_pipe_mmu.v", RTL / "ap040_pipe_membus.v",
                   RTL / "ap040_pipe_cache_arr.v", HERE / "sim_pipe_ram.v", DPRAM]
        elif name.endswith("icache"):
            # the IMU and the bus controller on their own, as busredirect
            src = [RTL / "ap040_pipe_imu.v", RTL / "ap040_pipe_membus.v",
                   RTL / "ap040_pipe_cache_arr.v", HERE / "sim_pipe_ram.v"]
        elif name.endswith("busredirect"):
            # ap040_pipe_imu.v and ap040_pipe_membus.v standalone, same
            # reason as l1_wbuf above: the bench drives their ports directly.
            src = [RTL / "ap040_pipe_imu.v", RTL / "ap040_pipe_membus.v",
                   RTL / "ap040_pipe_cache_arr.v", HERE / "sim_pipe_ram.v"]
        elif name.endswith("dual"):
            # The differential bench instantiates the FSM core beside the
            # pipelined one, so rtl_old's whole core comes too.
            src = CORE + [RTL / "ap040_pipe_bus16.v", OLD / "ap040_bus16_adapter.v"] + [
                OLD / n for n in (
                    "ap040_tg68k_compat.v", "ap040_core.v",
                    "ap040_regfile.v", "ap040_alu.v", "ap040_muldiv.v",
                    "ap040_mmu.v", "ap040_cache.v")   # ap040_fpu.v is in CORE
            ] + [DPRAM]
            inc = [RTL, OLD]
        elif name.endswith("inject"):
            # ap040_pipe_bus16.v, which brings the FSM core's adapter and MMU
            # with it -- the same top the corpus replay would drive.
            src = CORE + BUS16
            inc = [RTL, OLD]
        elif name.endswith("program"):
            # ap040_pipe_bus16.v with its adapter and MMU, as for the bus16 bench.
            src = CORE + BUS16
            inc = [RTL, OLD]
        elif name.endswith("bus16"):
            # ap040_pipe_bus16.v instantiates the FSM core's own 16-bit
            # adapter and MMU, so those files and their include directory come.
            src = CORE + BUS16
            inc = [RTL, OLD]
        elif name.endswith("alu_equiv"):
            src = [RTL / "ap040_pipe_alu.v", OLD / "ap040_alu.v"]
            inc = [RTL, OLD]
        obj = work / ("obj-" + name)
        log = work / (name + ".log")
        blog_path = work / (name + ".build.log")
        with log.open("w") as out, blog_path.open("w") as blog:
            rc = subprocess.run(
                ["verilator", "--binary", "--timing", "--top-module", b.stem,
                 "--Mdir", str(obj), "-j", str(args.jobs), "-Wno-fatal",
                 *("-I" + str(d) for d in inc),
                 # Every bench's end-of-program wait is `repeat (N * AP040_PIPE_WAIT_SCALE)`:
                 # the slow L1 roughly triples the cycles a program takes, so the
                 # wait scales with it rather than each bench guessing.
                 # Each mode roughly doubles or triples how long a program
                 # takes, so the end-of-program wait scales with them rather
                 # than every bench guessing.
                 "-DAP040_PIPE_WAIT_SCALE=" + ("8" if (args.slow_l1 and args.ce_random)
                                               else "4" if (args.slow_l1 or args.ce_random)
                                               else "1"),
                 *(["-DAP040_PIPE_L1_SLOW"] if args.slow_l1 else []),
                 *(["-DAP040_PIPE_CE_RANDOM"] if args.ce_random else []), *defines, *params,
                 str(b)] + [str(s) for s in src],
                stdout=blog, stderr=subprocess.STDOUT, env=env).returncode
            # The build's own output goes to <bench>.build.log: a warning quotes
            # source lines, and a quoted $display("FAIL: ...") read as a failure.
            # A build that fails is still a failure, and says so here.
            if rc != 0:
                out.write(f"%Error: the build failed, see {blog.name}\n")
            progs = (PROGRAMS_REQUIRED + list(PROGRAMS_OPEN) if name.endswith("program") else
                     PROGRAMS_LOCAL if name.endswith("program_local") else None)
            if rc == 0 and progs is not None:
                for prog in progs:
                    out.write(f"== {prog}\n"); out.flush()
                    image = program_image(prog, work)
                    if not image.exists():
                        out.write(f"FAIL: no image for {prog}: {image} (run ./build_tests.sh)\n")
                        rc = 1
                        continue
                    r = subprocess.run([str(obj / ("V" + b.stem)), "+prog=" + str(image)],
                                       capture_output=True, text=True, timeout=1800, env=env)
                    text_p = r.stdout + r.stderr
                    passed_p = r.returncode == 0 and "ALL TESTS PASSED" in text_p
                    if prog in PROGRAMS_OPEN:
                        # reported, not judged: the output is kept out of the
                        # log's FAIL scan below and summarised instead
                        out.write(f"  open ({PROGRAMS_OPEN[prog]}): {'passes now' if passed_p else 'not yet'}\n")
                    else:
                        out.write(text_p)
                        if not passed_p:
                            out.write(f"FAIL: {prog} did not pass\n")
                            rc = 1
            elif rc == 0 and runs is not None:
                for r_args in runs:
                    out.write("== " + " ".join(r_args) + "\n"); out.flush()
                    r = subprocess.run([str(obj / ("V" + b.stem))] + r_args, capture_output=True, text=True,
                                       timeout=600, env=env)
                    out.write(r.stdout + r.stderr)
                    if r.returncode != 0 or "ALL TESTS PASSED" not in r.stdout + r.stderr:
                        out.write("FAIL: " + " ".join(r_args) + " did not pass\n")
                        rc = 1
            elif rc == 0:
                rc = subprocess.run([str(obj / ("V" + b.stem))], stdout=out,
                                    stderr=subprocess.STDOUT, timeout=300, env=env).returncode
        text = log.read_text()
        ok = rc == 0 and not any(m in text for m in ("FAIL", "ERROR:", "MISMATCH", "%Error"))
        if ok and not args.keep_obj:
            shutil.rmtree(obj, ignore_errors=True)
        (passed if ok else failed).append(name)
        print(f"  {'ok  ' if ok else 'FAIL'} {name}", flush=True)

    print(f"\n{len(passed)}/{len(benches)} pipelined milestone benches passed under Verilator")
    if failed:
        print("failed:", ", ".join(failed))
        raise SystemExit(1)


if __name__ == "__main__":
    main()
