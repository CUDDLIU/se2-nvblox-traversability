"""Incremental mesh storage with an AABB BVH over blocks and their triangles.

Queries return triangle *candidates*: intersection of triangle and query AABBs
is conservative, and the downstream voxelizer performs the geometric test.
Both levels are actual balanced BVHs; replacing a block only rebuilds its own
triangle tree, while rebuilding the inexpensive block tree is deferred until
the next query. Coordinates and triangles are copied on insertion.
"""

from dataclasses import dataclass, field
from types import MappingProxyType

import numpy as np
from native_bvh import NativeBVH


@dataclass(slots=True)
class _Node:
    low: np.ndarray
    high: np.ndarray
    left: object = None
    right: object = None
    indices: object = None


class _BVH:
    def __init__(self, low, high, leaf_size=8):
        self.low = low
        self.high = high
        self.leaf_size = leaf_size
        self.root = self._build(np.arange(len(low), dtype=np.int64)) if len(low) else None

    def _build(self, indices):
        low = self.low[indices].min(axis=0)
        high = self.high[indices].max(axis=0)
        if len(indices) <= self.leaf_size:
            return _Node(low, high, indices=indices)
        centers = self.low[indices] * 0.5 + self.high[indices] * 0.5
        axis = int(np.argmax(np.ptp(centers, axis=0)))
        order = np.argsort(centers[:, axis], kind="stable")
        middle = len(indices) // 2
        return _Node(low, high, self._build(indices[order[:middle]]),
                     self._build(indices[order[middle:]]))

    def query(self, low, high):
        result = []
        visited = 0
        stack = [self.root] if self.root is not None else []
        lx, ly, lz = low
        hx, hy, hz = high
        while stack:
            node = stack.pop()
            visited += 1
            # A six-scalar overlap test avoids allocating two tiny temporary
            # arrays and performing Python->NumPy reductions for every node.
            if (node.high[0] < lx or node.low[0] > hx
                    or node.high[1] < ly or node.low[1] > hy
                    or node.high[2] < lz or node.low[2] > hz):
                continue
            if node.indices is not None:
                indices = node.indices
                hits = np.all(self.high[indices] >= low, axis=1)
                hits &= np.all(self.low[indices] <= high, axis=1)
                result.extend(indices[hits].tolist())
            else:
                stack.append(node.right)
                stack.append(node.left)
        return np.asarray(sorted(result), dtype=np.int64), visited

    def iter_hits(self, low, high, visits):
        """Yield matching primitive indices lazily, for existence queries."""
        stack = [self.root] if self.root is not None else []
        lx, ly, lz = low
        hx, hy, hz = high
        while stack:
            node = stack.pop()
            visits[0] += 1
            if (node.high[0] < lx or node.low[0] > hx
                    or node.high[1] < ly or node.low[1] > hy
                    or node.high[2] < lz or node.low[2] > hz):
                continue
            if node.indices is not None:
                for index in node.indices:
                    primitive_low, primitive_high = self.low[index], self.high[index]
                    if (primitive_high[0] >= lx and primitive_low[0] <= hx
                            and primitive_high[1] >= ly and primitive_low[1] <= hy
                            and primitive_high[2] >= lz and primitive_low[2] <= hz):
                        yield int(index)
            else:
                stack.append(node.right)
                stack.append(node.left)


@dataclass(slots=True)
class _Block:
    vertices: np.ndarray
    triangles: np.ndarray
    low: np.ndarray
    high: np.ndarray
    tree: _BVH
    raster_cache: dict = field(default_factory=dict)

    def bounds(self):
        return self.low.copy(), self.high.copy()


class MeshIndex:
    """Mutable block mesh and conservative, inclusive AABB queries.

    ``replace`` validates before changing state and returns the old/new bounds.
    Empty triangles delete the block. Bounds include only referenced vertices.
    ``stats`` reports the most recent query, plus cumulative query/rebuild counts.
    """

    def __init__(self, triangle_leaf_size=8, block_leaf_size=4, native=True):
        if int(triangle_leaf_size) < 1 or int(block_leaf_size) < 1:
            raise ValueError("BVH leaf sizes must be positive")
        self.triangle_leaf_size = int(triangle_leaf_size)
        self.block_leaf_size = int(block_leaf_size)
        self._tree_type = NativeBVH if native else _BVH
        self._blocks = {}
        self._keys = []
        self._tree = None
        self._dirty = False
        self._triangle_count = 0
        self.stats = {}
        self._reset_stats()

    @property
    def blocks(self):
        return MappingProxyType(self._blocks)

    def _reset_stats(self):
        self.stats.clear()
        self.stats.update(block_count=0, triangle_count=0, queries=0,
                          top_level_rebuilds=0, top_nodes_visited=0,
                          triangle_nodes_visited=0, visited_nodes=0,
                          candidate_blocks=0, candidate_triangles=0,
                          existence_queries=0, last_query_kind='none')

    @staticmethod
    def _arrays(vertices, triangles):
        vertices = np.asarray(vertices, dtype=np.float64)
        triangles = np.asarray(triangles)
        if vertices.size == 0:
            vertices = vertices.reshape((0, 3))
        if triangles.size == 0:
            triangles = triangles.reshape((0, 3))
        if vertices.ndim != 2 or vertices.shape[1] != 3:
            raise ValueError("vertices must have shape (N, 3)")
        if triangles.ndim != 2 or triangles.shape[1] != 3:
            raise ValueError("triangles must have shape (M, 3)")
        if not np.isfinite(vertices).all():
            raise ValueError("vertices must be finite")
        if triangles.size:
            if triangles.dtype.kind not in 'iuf':
                raise ValueError("triangle indices must be integers")
            if triangles.min() < 0 or triangles.max() >= len(vertices):
                raise ValueError("triangle index outside this block's vertices")
            if triangles.dtype.kind == 'f':
                if not np.isfinite(triangles).all() or np.any(triangles != np.floor(triangles)):
                    raise ValueError("triangle indices must be finite integers")
        return vertices.copy(), np.array(triangles, dtype=np.int64, copy=True)

    def replace(self, key, vertices, triangles):
        if not isinstance(key, tuple):
            raise TypeError("mesh block keys must be tuples")
        vertices, triangles = self._arrays(vertices, triangles)
        return self.replace_validated(key,vertices,triangles)

    def replace_validated(self,key,vertices,triangles):
        """Internal ownership transfer after _arrays; avoid validating/copying twice."""
        old = self._blocks.get(key)
        old_bounds = old.bounds() if old is not None else None
        if not len(triangles):
            if old is not None:
                self._triangle_count -= len(old.triangles)
                del self._blocks[key]
                self._dirty = True
            self.stats.update(block_count=len(self._blocks), triangle_count=self._triangle_count)
            return old_bounds, None
        if (old is not None and np.array_equal(old.vertices, vertices)
                and np.array_equal(old.triangles, triangles)):
            return old_bounds, old.bounds()
        points = vertices[triangles]
        low, high = points.min(axis=1), points.max(axis=1)
        block = _Block(vertices, triangles, low.min(axis=0), high.max(axis=0),
                       self._tree_type(low, high, self.triangle_leaf_size))
        self._blocks[key] = block
        self._triangle_count += len(triangles) - (len(old.triangles) if old is not None else 0)
        self._dirty = True
        self.stats.update(block_count=len(self._blocks), triangle_count=self._triangle_count)
        return old_bounds, block.bounds()

    def _rebuild(self):
        # repr also provides stable ordering for heterogeneous tuple components.
        self._keys = sorted(self._blocks, key=repr)
        low = np.array([self._blocks[key].low for key in self._keys]).reshape((-1, 3))
        high = np.array([self._blocks[key].high for key in self._keys]).reshape((-1, 3))
        self._tree = self._tree_type(low, high, self.block_leaf_size)
        self._dirty = False
        self.stats["top_level_rebuilds"] += 1

    @staticmethod
    def _query_bounds(low, high):
        low, high = np.asarray(low, dtype=np.float64), np.asarray(high, dtype=np.float64)
        if (low.shape != (3,) or high.shape != (3,) or not np.isfinite(low).all()
                or not np.isfinite(high).all() or np.any(low > high)):
            raise ValueError("query bounds must be finite 3-vectors with low <= high")
        return low, high

    def query(self, low, high):
        low, high = self._query_bounds(low, high)
        if self._dirty or self._tree is None:
            self._rebuild()
        candidates, top_visited = self._tree.query(low, high)
        pieces = []
        triangle_visited = 0
        count = 0
        for block_id in candidates:
            block = self._blocks[self._keys[int(block_id)]]
            indices, visited = block.tree.query(low, high)
            triangle_visited += visited
            count += len(indices)
            if len(indices):
                pieces.append(block.vertices[block.triangles[indices]].reshape((-1, 3)))
        self.stats.update(queries=self.stats["queries"] + 1,
                          last_query_kind='geometry',
                          top_nodes_visited=top_visited,
                          triangle_nodes_visited=triangle_visited,
                          visited_nodes=top_visited + triangle_visited,
                          candidate_blocks=len(candidates), candidate_triangles=count)
        vertices = np.concatenate(pieces) if pieces else np.empty((0, 3), dtype=np.float64)
        triangles = np.arange(count * 3, dtype=np.int64).reshape((-1, 3))
        return vertices, triangles

    def has_mesh(self, low, high):
        """Return whether any triangle AABB intersects bounds, without copying.

        Both hierarchy traversals stop at the first triangle hit. This uses
        exactly the inclusive candidate test of ``query``; it does not claim
        geometric triangle-box intersection when only their AABBs intersect.
        Last-query stats report work done before this early termination.
        """
        low, high = self._query_bounds(low, high)
        if self._dirty or self._tree is None:
            self._rebuild()
        top_visits, triangle_visits = [0], [0]
        candidate_blocks, found = 0, False
        for block_id in self._tree.iter_hits(low, high, top_visits):
            candidate_blocks += 1
            block = self._blocks[self._keys[block_id]]
            if next(block.tree.iter_hits(low, high, triangle_visits), None) is not None:
                found = True
                break
        self.stats.update(queries=self.stats['queries'] + 1,
                          existence_queries=self.stats['existence_queries'] + 1,
                          last_query_kind='existence',
                          top_nodes_visited=top_visits[0],
                          triangle_nodes_visited=triangle_visits[0],
                          visited_nodes=top_visits[0] + triangle_visits[0],
                          candidate_blocks=candidate_blocks, candidate_triangles=int(found))
        return found

    def query_spans(self, low, high, resolution, dz, slope_cos, height):
        """Same triangle candidates as query(), reusing block raster records.

        Cache belongs to the immutable block object, so replacement/deletion
        cannot leave stale obstacle or floor records in the next heightfield.
        """
        from paper_native import raster_records, merge_records
        from fast_raster import _dtype
        low, high = self._query_bounds(low, high)
        if self._dirty or self._tree is None:
            self._rebuild()
        candidates, top_visited = self._tree.query(low, high)
        if self._tree_type is NativeBVH:
            from paper_native import batch_mesh_spans
            blocks=[self._blocks[self._keys[int(i)]] for i in candidates]
            result,(triangle_visited,count,misses)=batch_mesh_spans(
                blocks,low,high,resolution,dz,slope_cos,height)
            self.stats.update(queries=self.stats['queries']+1,last_query_kind='cached_spans',
                top_nodes_visited=top_visited,triangle_nodes_visited=triangle_visited,
                visited_nodes=top_visited+triangle_visited,candidate_blocks=len(candidates),
                candidate_triangles=count,raster_cache_misses=misses)
            return result
        pieces = []
        triangle_visited = count = misses = 0
        cache_key = (resolution, slope_cos)
        for block_id in candidates:
            block = self._blocks[self._keys[int(block_id)]]
            contained = bool(np.all(block.low >= low) and np.all(block.high <= high))
            indices, visited = ((None,0) if contained else block.tree.query(low, high))
            triangle_visited += visited
            count += len(block.triangles) if contained else len(indices)
            if not contained and not len(indices):
                continue
            if cache_key not in block.raster_cache:
                block.raster_cache[cache_key] = raster_records(
                    block.vertices, block.triangles, resolution, slope_cos)
                misses += 1
            records = block.raster_cache[cache_key]
            if contained:
                pieces.append(records)
                continue
            selected = np.zeros(len(block.triangles), dtype=bool)
            selected[indices] = True
            # Original triangle IDs are retained in each clipped record.
            pieces.append(records[selected[records['reserved']]])
        self.stats.update(queries=self.stats['queries']+1, last_query_kind='cached_spans',
                          top_nodes_visited=top_visited, triangle_nodes_visited=triangle_visited,
                          visited_nodes=top_visited+triangle_visited,
                          candidate_blocks=len(candidates), candidate_triangles=count,
                          raster_cache_misses=misses)
        records = np.concatenate(pieces) if pieces else np.empty(0, dtype=_dtype)
        return merge_records(records, dz, height)

    def clear(self):
        self._blocks.clear()
        self._keys = []
        self._tree = None
        self._dirty = False
        self._triangle_count = 0
        self._reset_stats()
