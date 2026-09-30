"""Latest completed cells per slab for RViz, separate from current validity.

A replacement owns the entire slab, including deletion. New dirty input keeps
the preceding geometry as history; it cannot certify current traversability.
"""


class LocalDisplay:
    def __init__(self):
        self.clear()

    def clear(self):
        self.cells = {}
        self.dirty = set()
        self.archive_cells = {}
        self.old_config = set()

    def archive(self):
        for key, rows in self.cells.items():
            self.archive_cells[key] = list(rows)
        self.cells.clear()
        self.dirty.clear()
        self.old_config.clear()

    def parameters_changed(self):
        self.old_config.update(self.cells)

    def invalidate(self, keys):
        self.dirty.update(keys)

    def replace(self, slabs):
        for slab in slabs:
            key = tuple(slab['key'])
            # Rejected rows erase previously permitted cells, without adding
            # thousands of points to the visualization workload.
            cells = [tuple(row) for row in slab['cells'] if row[3]]
            # No surviving Mesh surfaces: retain the preceding result as
            # historical evidence, never as current green. A populated slab
            # (even all-rejected) authoritatively replaces its older evidence.
            if not slab['cells']:
                previous = self.cells.get(key)
                if previous:
                    self.archive_cells[key] = list(previous)
            else:
                self.archive_cells.pop(key, None)
            if cells:
                self.cells[key] = cells
            else:
                self.cells.pop(key, None)
            self.dirty.discard(key)
            self.old_config.discard(key)

    def snapshot(self, yaw_bins, stale=False):
        full = (1 << yaw_bins)-1
        green, amber, current_green, current_amber, historical, archived = [], [], [], [], [], []
        for rows in self.archive_cells.values():
            archived.extend(row[:3] for row in rows)
        for key, rows in self.cells.items():
            for row in rows:
                xyz = row[:3]
                if key in self.old_config:
                    archived.append(xyz)
                    continue
                else:
                    (green if row[3] == full else amber).append(xyz)
                if stale or key in self.dirty or key in self.old_config:
                    historical.append(xyz)
                else:
                    (current_green if row[3] == full else current_amber).append(xyz)
        return dict(green=green, amber=amber, current_green=current_green,
                    current_amber=current_amber, historical=historical, archived=archived)

    def to_document(self, frame, session, config):
        return dict(schema=1, frame=frame, map_session=session, config=config,
                    slabs=[dict(key=list(k), cells=rows) for k,rows in self.cells.items()],
                    archived=[dict(key=list(k), cells=rows) for k,rows in self.archive_cells.items()])

    def load_document(self, document, frame, session):
        if not session or document.get('map_session')!=session or document.get('frame')!=frame:
            return False
        for slab in document.get('archived',[])+document.get('slabs',[]):
            self.archive_cells[tuple(slab['key'])]=[tuple(row) for row in slab['cells']]
        return True
