from setuptools import setup
from Cython.Build import cythonize
import numpy
setup(
    name='Skeletonize2d app',
    ext_modules=cythonize("_skeletonize2d_cy.pyx"),
    include_dirs=numpy.get_include(),
    zip_safe=False
)