"""Replay checker using the explicitly selected experiment implementation."""
import os
from pathlib import Path
import sys

here=Path(__file__).resolve().parent
sys.path.insert(0,str(here/os.environ.get('TERRAIN_IMPLEMENTATION','baseline')))
sys.path.insert(1,str(here.parent/'bag_tools'))
# rebuild_node inserts the live tree; preload modules so the chosen baseline wins.
import realtime_node
from rebuild_node import main
# The replay adapter prepends the live tree. Restore the selected tree before
# Engine.__init__ performs its lazy native_mesh/native_nav imports.
implementation=(here/os.environ.get('TERRAIN_IMPLEMENTATION','baseline')).resolve()
sys.path.insert(0,str(implementation))
import native_mesh
import native_nav
for module in (realtime_node,native_mesh,native_nav):
    if Path(module.__file__).resolve().parent!=implementation:
        raise RuntimeError('Mixed terrain implementations: '+str(module.__file__))
print('TERRAIN_IMPLEMENTATION '+str(implementation),flush=True)

if __name__=='__main__':main()
