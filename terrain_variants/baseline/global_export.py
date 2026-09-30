"""Global historical graph export in a process isolated from real-time ROS I/O."""
import json
import os


def lower_priority():
    os.nice(5)


def export_document(config, results, generation, revision, metadata):
    from paper_pipeline import Engine
    from replay_paper import json_safe
    # snapshot only consumes the completed results and configuration. Never
    # pickle/copy native BVH pointers or initialize a ROS node in this worker.
    view=Engine.__new__(Engine)
    view.config=config;view.results=results;view.pending=set()
    view.generation=generation;view.revision=revision
    document=json_safe(view.snapshot(rebuild_polygons=True))
    document.update(metadata)
    return json.dumps(document,separators=(',',':'),allow_nan=False)
