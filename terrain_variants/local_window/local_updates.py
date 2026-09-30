"""Per-slab provenance for incremental output, independent of ROS clocks."""
import time
from angular_display import result_rows


class LocalUpdates:
    def __init__(self):
        self.pending = {}
        self.sequence = 0

    def clear(self):
        self.pending.clear()

    def invalidate(self, keys, received, sequence, stamp, newest_received=None):
        newest_received = received if newest_received is None else newest_received
        for key in keys:
            previous = self.pending.get(key)
            self.pending[key] = (min(previous[0], received) if previous else received,
                                 newest_received, sequence, stamp)

    def finish(self, engine, keys, now=None):
        now = time.monotonic() if now is None else now
        updates = []
        oldest, newest = [], []
        for key in keys:
            source = self.pending.pop(key)
            oldest.append(source[0])
            newest.append(source[1])
            result = engine.results.get(key)
            updates.append(dict(
                key=list(key), input_sequence=source[2], input_mesh_stamp=source[3],
                input_received_monotonic=source[0],
                latency_ms=(now-source[0])*1000,
                # Whole-slab replacement, including rejected cells. An empty
                # cells list deletes all previous surfaces owned by this slab.
                cells=result_rows(result,engine.config.yaw_bins) if result else []))
        if not keys:
            return None
        self.sequence += 1
        return dict(schema_version=2, cell_fields=['ix','iy','iz','yaw_mask','comfortable_mask','feasible_yaw_radians'], local_sequence=self.sequence,
                    generation=engine.generation, revision=engine.revision,
                    complete_local=True, complete_input=not engine.pending,
                    global_updating=bool(engine.pending),
                    input_received_monotonic=min(oldest),
                    newest_input_received_monotonic=max(newest),
                    latency_oldest_ms=(now-min(oldest))*1000,
                    latency_newest_ms=(now-max(newest))*1000,
                    updated_slabs=updates, pending_slabs=len(engine.pending))
