#!/usr/bin/env python3
"""Check that a busy worker processes only the newest waiting RGB-D pair."""
import sys
import threading
from types import SimpleNamespace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from point_flow_raft_preview_experimental import ExperimentalRaftPointFlow  # noqa: E402


class WorkerHarness:
    run_worker = ExperimentalRaftPointFlow.run_worker
    submit_pair = ExperimentalRaftPointFlow.submit_pair
    stop_worker = ExperimentalRaftPointFlow.stop_worker

    def __init__(self):
        self.async_pairing = True
        self.profile_pairing = False
        self.max_pair_age_s = 0.5
        self.worker_condition = threading.Condition()
        self.pending_pair = None
        self.worker_running = True
        self.worker_busy = False
        self.started = threading.Event()
        self.release = threading.Event()
        self.finished = threading.Event()
        self.seen = []
        self.worker_thread = threading.Thread(target=self.run_worker)
        self.worker_thread.start()

    def process_pair(self, depth, color):
        self.seen.append((depth, color))
        if depth == 1:
            self.started.set()
            if not self.release.wait(5):
                raise RuntimeError("worker did not resume")
        else:
            self.finished.set()

    def get_logger(self):
        return self

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000))

    def stamp(self, message):
        return 0.2 if message == 4 else 1.0

    def error(self, message):
        print(message, file=sys.stderr)
        self.finished.set()


def main():
    worker = WorkerHarness()
    try:
        worker.submit_pair(1, 1)
        assert worker.started.wait(5)
        worker.submit_pair(2, 2)
        worker.submit_pair(3, 3)
        worker.release.set()
        assert worker.finished.wait(5)
        assert worker.seen == [(1, 1), (3, 3)], worker.seen
        worker.submit_pair(4, 4)
        with worker.worker_condition:
            assert worker.pending_pair is None
    finally:
        worker.release.set()
        worker.stop_worker()
    assert not worker.worker_thread.is_alive()
    print("Latest pair retained; worker stopped cleanly")


if __name__ == "__main__":
    main()
