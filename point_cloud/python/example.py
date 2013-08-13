""" Snark point_cloud examples.

Partitioning Example
--------------------

>>> import numpy as np
>>> import snark_py_point_cloud as snark
>>> p1 = np.random.rand(100, 3)
>>> p2 = np.random.rand(5, 3) + [0, 5, 0]
>>> p = np.vstack((p1, p2))
>>> ids = snark.partition(p, resolution=1.0)
>>> (p[ids == 0] == p1).all()
True
>>> (p[ids == 1] == p2).all()
True
>>> ids = snark.partition(p, min_points_per_partition=10)
>>> (ids[-5:] == snark.ID_INVALID).all()
True

"""

if __name__ == '__main__':
    import doctest
    doctest.testmod(verbose=True)
