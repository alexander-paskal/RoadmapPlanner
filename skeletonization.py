import pyximport
import numpy

from _skeletonize2d_cy import _fast_skeletonize

def skeletonize2d(image):
    skeleton, sdf = _skeletonize_2d(image.astype(bool, copy=False))
    sdf[skeleton == 1] = numpy.inf
    return skeleton, sdf

def _skeletonize_2d(image):
    """Return the skeleton of a 2D binary image."""

    if image.ndim != 2:
        raise ValueError("Zhang's skeletonize method requires a 2D array")
    
    return _fast_skeletonize(image)