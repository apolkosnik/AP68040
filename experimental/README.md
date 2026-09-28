# Pipeline prototype

This directory is not in the MiSTer build. It contains the first executable
checkpoint of the pipeline rewrite: a resident instruction input, decode,
execute, and in-order writeback for register-only integer instructions.
It is not a replacement CPU and cannot boot a guest.

Run `python3 scripts/cpu/pipeline_prototype.py` from the repository root.
The runner generates instruction streams and an independent architectural
oracle, checks the current AP040 against that oracle at instruction completion,
then checks the prototype against the same sequence. Output is under
`scratch/pipeline_prototype/` by default.

The external instruction source must hold valid/opcode/PC until ready.
Unsupported instructions stay at decode until older operations retire;
`fallback_valid` then identifies the instruction that requires the side engine.
The initial checkpoint deliberately does not implement that engine handover.
`flush` discards all unretired work, including WB; it does not undo committed
registers. It is an external cancellation input, not a branch recovery policy.
