"""Compare the two-CSR native search with independent SciPy shortest paths."""
import ctypes
from types import SimpleNamespace
import unittest
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra
from search import NativeGraph, Search


def view(matrix, blocked=None):
    offsets=np.asarray(matrix.indptr,dtype=np.int64)
    target=np.asarray(matrix.indices,dtype=np.int64)
    costs=np.asarray(matrix.data,dtype=np.float64)
    return NativeGraph(matrix.shape[0],offsets.ctypes.data,target.ctypes.data,costs.ctypes.data,
                       blocked.ctypes.data if blocked is not None else None), (offsets,target,costs,blocked)


class NativeSearchTests(unittest.TestCase):
    def test_overlay_lower_bound_and_collision_rejections_preserve_optimal_cost(self):
        library=Search(SimpleNamespace(bins=4)).library
        for seed in range(20):
            rng=np.random.default_rng(seed);n=30;base=23
            rows,cols=np.nonzero(rng.random((n,n))<.12)
            keep=rows!=cols;rows,cols=rows[keep],cols[keep]
            costs=rng.uniform(.01,5.,len(rows))
            static=(rows<base)&(cols<base)
            maps=[csr_matrix((costs[static],(rows[static],cols[static])),shape=(base,base)),
                  csr_matrix((costs[~static],(rows[~static],cols[~static])),shape=(n,n))]
            full=csr_matrix((costs,(rows,cols)),shape=(n,n))
            goal=np.zeros(n,dtype=np.uint8);goal[-2:]=1;start=seed%n
            exact=dijkstra(full.T.tocsr(),directed=True,indices=np.flatnonzero(goal),min_only=True)
            reverse=[view(m.T.tocsr()) for m in maps]
            native=(NativeGraph*2)(*(v for v,_ in reverse))
            h=np.empty(n);expanded=ctypes.c_int64()
            self.assertEqual(library.se2_goal_distances_graphs(n,native,2,goal.ctypes.data,start,
                h.ctypes.data,ctypes.byref(expanded)),0)
            np.testing.assert_allclose(h,np.minimum(exact,exact[start]),rtol=1e-12)

            blocked=[(rng.random(m.nnz)<.2).astype(np.uint8) for m in maps]
            nodes=(rng.random(n)<.1).astype(np.uint8);nodes[start]=0;nodes[-2:]=0
            forward=[view(m,b) for m,b in zip(maps,blocked)]
            native=(NativeGraph*2)(*(v for v,_ in forward))
            rr=[];cc=[];vv=[]
            for m,b in zip(maps,blocked):
                r=np.repeat(np.arange(m.shape[0]),np.diff(m.indptr));c=m.indices
                valid=(b==0)&(nodes[r]==0)&(nodes[c]==0)
                rr.extend(r[valid]);cc.extend(c[valid]);vv.extend(m.data[valid])
            checked=csr_matrix((vv,(rr,cc)),shape=(n,n))
            expected=dijkstra(checked,directed=True,indices=start)[-2:].min()
            output=np.empty(n,dtype=np.int64);length=ctypes.c_int64();total=ctypes.c_double()
            code=library.se2_astar_graphs(n,native,2,nodes.ctypes.data,h.ctypes.data,start,
                goal.ctypes.data,output.ctypes.data,ctypes.byref(length),ctypes.byref(total),ctypes.byref(expanded))
            self.assertEqual(code,0 if np.isfinite(expected) else 1)
            if code==0:
                self.assertAlmostEqual(total.value,expected,places=10)
                self.assertEqual(output[0],start)
                self.assertTrue(goal[output[length.value-1]])


if __name__=='__main__':unittest.main()
