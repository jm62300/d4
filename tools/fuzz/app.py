"""
Textual TUI for the d4 fuzzer.
"""

from __future__ import annotations
import json
from pathlib import Path
from typing import Optional

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.screen import ModalScreen
from textual.widgets import (
    Button, DataTable, Footer, Header, Input, Label,
    Log, TabbedContent, TabPane,
)
from textual import work
import asyncio

from runner import SuiteRunner, Status


STATUS_STYLE = {
    Status.WAITING:  ("dim",      "waiting"),
    Status.RUNNING:  ("green",    "running"),
    Status.PAUSED:   ("yellow",   "paused"),
    Status.STOPPED:  ("dim",      "stopped"),
    Status.MAX_BUGS: ("red bold", "max bugs"),
    Status.ERROR:    ("red bold", "error"),
}


# ---------------------------------------------------------------------------
# Modal: edit max_vars for the selected suite
# ---------------------------------------------------------------------------

class EditMaxVarsModal(ModalScreen[Optional[int]]):
    CSS = """
    EditMaxVarsModal { align: center middle; }
    #modal_box {
        background: $surface;
        border: thick $primary;
        padding: 1 3;
        width: 52;
        height: auto;
    }
    #modal_label   { margin-bottom: 1; }
    #modal_input   { margin-bottom: 1; }
    #modal_buttons { layout: horizontal; height: auto; align: right middle; }
    #modal_buttons Button { margin-left: 1; min-width: 10; }
    """
    BINDINGS = [Binding("escape", "cancel", show=False)]

    def __init__(self, suite_name: str, current: int) -> None:
        super().__init__()
        self._suite_name = suite_name
        self._current    = current

    def compose(self) -> ComposeResult:
        from textual.containers import Vertical, Horizontal
        with Vertical(id="modal_box"):
            yield Label(
                f"Max vars — suite [bold]{self._suite_name}[/bold]",
                id="modal_label",
            )
            yield Input(value=str(self._current), placeholder="e.g. 100",
                        id="modal_input")
            with Horizontal(id="modal_buttons"):
                yield Button("OK",     id="btn_ok",     variant="primary")
                yield Button("Cancel", id="btn_cancel")

    def on_mount(self) -> None:
        self.query_one("#modal_input", Input).focus()

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "btn_ok":
            self._confirm()
        else:
            self.dismiss(None)

    def on_input_submitted(self, _event: Input.Submitted) -> None:
        self._confirm()

    def action_cancel(self) -> None:
        self.dismiss(None)

    def _confirm(self) -> None:
        raw = self.query_one("#modal_input", Input).value.strip()
        try:
            val = int(raw)
            if val < 1:
                raise ValueError
            self.dismiss(val)
        except ValueError:
            self.query_one("#modal_input", Input).add_class("error")


# ---------------------------------------------------------------------------
# Main application
# ---------------------------------------------------------------------------

class FuzzApp(App):
    CSS = """
    DataTable     { height: 1fr; }
    Log           { height: 1fr; border: solid $accent; }
    TabbedContent { height: 1fr; }
    """

    BINDINGS = [
        Binding("q", "quit",             "Quit"),
        Binding("s", "stop_all",         "Stop all"),
        Binding("p", "pause_selected",   "Pause/Resume"),
        Binding("r", "restart_selected", "Restart"),
        Binding("v", "edit_max_vars",    "Max vars"),
    ]

    TITLE = "d4 Fuzzer"

    def __init__(self, runners: list[SuiteRunner]) -> None:
        super().__init__()
        self.runners = runners
        self._reported_bugs: set[tuple[str, int]] = set()

    def compose(self) -> ComposeResult:
        yield Header()
        with TabbedContent():
            with TabPane("Suites", id="tab_suites"):
                yield DataTable(id="suite_table", cursor_type="row")
            with TabPane("Bugs", id="tab_bugs"):
                yield Log(id="bug_log", highlight=True)
        yield Footer()

    def on_mount(self) -> None:
        table = self.query_one("#suite_table", DataTable)
        for label in ("Suite", "Evaluated", "Max vars", "Status",
                       "Tested", "Timeouts", "No answer", "Bugs", "Last bug"):
            table.add_column(label, key=label)
        for r in self.runners:
            table.add_row(
                r._suite_name,
                r._evaluated_name,
                str(r.suite.generator.max_vars),
                "waiting", "0", "0", "0", "0", "",
                key=r.display_name,
            )
        for r in self.runners:
            r.start()
        self._start_refresh()

    # ------------------------------------------------------------------
    # Refresh loop
    # ------------------------------------------------------------------

    @work(exclusive=False)
    async def _start_refresh(self) -> None:
        while True:
            await asyncio.sleep(0.5)
            self._refresh_table()
            self._refresh_bugs()

    def _refresh_table(self) -> None:
        table = self.query_one("#suite_table", DataTable)
        for r in self.runners:
            snap = r.state.snapshot()
            style, label = STATUS_STYLE.get(snap["status"], ("", str(snap["status"])))
            key = r.display_name
            table.update_cell(key, "Max vars",
                              str(r.suite.generator.max_vars))
            table.update_cell(key, "Status",
                              f"[{style}]{label}[/{style}]" if style else label)
            table.update_cell(key, "Tested",    str(snap["tested"]))
            table.update_cell(key, "Timeouts",  str(snap["timeouts"]))
            no_ans   = snap["no_answer"]
            na_style = "yellow" if no_ans else ""
            table.update_cell(key, "No answer",
                              f"[{na_style}]{no_ans}[/{na_style}]" if na_style else str(no_ans))
            table.update_cell(key, "Bugs",     str(snap["bug_count"]))
            table.update_cell(key, "Last bug", snap["last_bug"][:60])

    def _refresh_bugs(self) -> None:
        log = self.query_one("#bug_log", Log)
        for r in self.runners:
            with r.state.lock:
                bugs = list(r.state.bugs)
            for bug in bugs:
                key = (r.display_name, bug.index)
                if key not in self._reported_bugs:
                    self._reported_bugs.add(key)
                    log.write_line(
                        f"[red]BUG[/red] [{r.display_name}] #{bug.index} "
                        f"— {bug.reason}  →  {bug.instance_path}"
                    )

    # ------------------------------------------------------------------
    # Runner selection
    # ------------------------------------------------------------------

    def _selected_runner(self) -> SuiteRunner | None:
        table = self.query_one("#suite_table", DataTable)
        if table.cursor_row < 0 or table.cursor_row >= len(self.runners):
            return None
        return self.runners[table.cursor_row]

    # ------------------------------------------------------------------
    # Actions
    # ------------------------------------------------------------------

    def action_pause_selected(self) -> None:
        r = self._selected_runner()
        if r is None:
            return
        snap = r.state.snapshot()
        if snap["status"] == Status.RUNNING:
            r.pause()
        elif snap["status"] == Status.PAUSED:
            r.resume()

    def action_restart_selected(self) -> None:
        r = self._selected_runner()
        if r is None:
            return
        r.stop()
        import time; time.sleep(0.1)
        r._stop_event.clear()
        r.start()

    def action_stop_all(self) -> None:
        for r in self.runners:
            r.stop()

    def action_quit(self) -> None:
        for r in self.runners:
            r.stop()
        self.exit()

    def action_edit_max_vars(self) -> None:
        r = self._selected_runner()
        if r is None:
            return

        def _on_result(new_val: Optional[int]) -> None:
            if new_val is None:
                return
            # Apply immediately to all runners sharing this suite.
            for runner in self.runners:
                if runner._suite_name == r._suite_name:
                    runner.suite.generator.max_vars = new_val
            # Persist to JSON so hot-reload doesn't revert it.
            if r._config_path:
                _write_max_vars_to_json(r._config_path, r._suite_name, new_val)

        self.push_screen(
            EditMaxVarsModal(r._suite_name, r.suite.generator.max_vars),
            _on_result,
        )


# ---------------------------------------------------------------------------
# JSON persistence helper
# ---------------------------------------------------------------------------

def _write_max_vars_to_json(config_path: str, suite_name: str,
                             max_vars: int) -> None:
    try:
        p = Path(config_path)
        data = json.loads(p.read_text())
        for s in data.get("suites", []):
            if s.get("name") == suite_name:
                s.setdefault("generator", {})["max_vars"] = max_vars
                break
        p.write_text(json.dumps(data, indent=2) + "\n")
    except Exception:
        pass
