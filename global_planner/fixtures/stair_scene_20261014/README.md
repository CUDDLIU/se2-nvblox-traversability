# Captured stair route regression fixture

`mesh_corridors.npz` contains the mesh corridors used by the saved routes in
`routes.json`. These small fixture files are versioned together so that
`test_navigation_modes.py` and the captured-stair acceptance script can run
without the original robot map directory.

The fixture covers eight captured routes, including two climbs and six flat
routes. Reversing the climbs also exercises descending classification. It
validates route annotations, not physical stair traversal.
