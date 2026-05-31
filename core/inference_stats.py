class InferenceStats:
    """Tracks per-trial outcomes for `--eval --max-steps N [--trials N]`.

    A trial starts on the first user/model action after a scene reset
    (mirrors how `data_collector.recording` starts). It ends with one of:
      - "success"     : termination_manager detected success
      - "timeout"     : step counter exceeded max_steps
      - "manual_fail" : user pressed N during the trial
      - "redo"        : user pressed R, or sim was stopped — discarded, not counted

    Only counted outcomes (success / timeout / manual_fail) advance
    `trial_count`. "redo" entries are silently dropped.
    """

    def __init__(self, max_steps: int | None = None, max_trials: int | None = None):
        self.max_steps = int(max_steps) if max_steps is not None else None
        self.max_trials = max_trials
        self._attempts = []  # list of {"outcome": str, "steps": int, "instruction": str|None}
        self._trial_active = False
        self._trial_steps = 0
        self._trial_instruction = None

    def start_trial(self, instruction: str | None = None):
        """Call after each scene reset to arm a fresh trial. Pass the language
        instruction for this trial (built once at reset, displayed in summary)."""
        self._trial_active = False
        self._trial_steps = 0
        self._trial_instruction = instruction

    def tick(self, is_any_action: bool) -> bool:
        """Advance the per-trial step counter. Returns True iff this tick
        crossed the max_steps threshold (the caller should record a timeout).
        Does nothing before the first action of a trial, so idle staring at
        the scene does not consume the budget.
        """
        if not self._trial_active:
            if is_any_action:
                self._trial_active = True
                self._trial_steps = 1
            return False
        self._trial_steps += 1
        if self.max_steps is None:
            return False  # no timeout configured
        return self._trial_steps >= self.max_steps

    @property
    def trial_active(self) -> bool:
        return self._trial_active

    def record(self, outcome: str, reason: str | None = None):
        """Record the trial's outcome and disarm. No-op if no trial is active
        (prevents double-discards when redo is triggered multiple times in
        the same frame, e.g. window stop + R press). `reason` is an optional
        short note (e.g. why a trial failed) shown in the per-trial summary."""
        if not self._trial_active:
            return
        steps = self._trial_steps
        instruction = self._trial_instruction
        self._trial_active = False
        self._trial_steps = 0
        if outcome == "redo":
            return
        self._attempts.append({
            "outcome": outcome,
            "steps": steps,
            "instruction": instruction,
            "reason": reason,
        })
        tail = f" ({reason})" if reason else ""
        print(f"[Eval] Trial #{len(self._attempts)} {outcome} at step {steps}{tail}")

    @property
    def trial_count(self) -> int:
        return len(self._attempts)

    def is_done(self) -> bool:
        return self.max_trials is not None and self.trial_count >= self.max_trials

    def summary(self) -> str:
        total = self.trial_count
        succ_steps = [a["steps"] for a in self._attempts if a["outcome"] == "success"]
        succ = len(succ_steps)
        timeout = sum(1 for a in self._attempts if a["outcome"] == "timeout")
        fail = sum(1 for a in self._attempts if a["outcome"] == "manual_fail")

        def pct(n):
            return (n / total * 100.0) if total else 0.0

        trials_line = f"{total} / {self.max_trials}" if self.max_trials is not None else f"{total}"

        lines = [
            "========== Eval Summary ==========",
            f"Trials:        {trials_line}",
            f"  Success:      {succ}  ({pct(succ):.1f}%)",
            f"  Timeout:      {timeout}  ({pct(timeout):.1f}%)",
            f"  Manual fail:  {fail}  ({pct(fail):.1f}%)",
        ]
        if succ_steps:
            avg = sum(succ_steps) / len(succ_steps)
            lines.append(f"Success steps: avg={avg:.1f}  min={min(succ_steps)}  max={max(succ_steps)}")

        if self._attempts:
            lines.append("")
            lines.append("Per-trial details:")
            # Pad outcome column to max width for alignment
            w = max(len(a["outcome"]) for a in self._attempts)
            for i, a in enumerate(self._attempts, start=1):
                instr = a["instruction"] or "<no instruction>"
                line = f"  #{i:<3d} {a['outcome']:<{w}}  {a['steps']:>4d} steps  | \"{instr}\""
                if a.get("reason"):
                    line += f"  [{a['reason']}]"
                lines.append(line)

        lines.append("==================================")
        return "\n".join(lines)
