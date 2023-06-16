
import numpy as np
cimport numpy as cnp
cnp.import_array()


def _fast_skeletonize(cnp.uint8_t [:, ::1] image):
    cdef cnp.uint8_t *lut = [0, 0, 0, 1, 0, 0, 1, 3, 0, 0, 3, 1, 1, 0,
                             1, 3, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2, 0,
                             3, 0, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             2, 0, 0, 0, 3, 0, 2, 2, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0,
                             0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 2, 0, 0, 0,
                             3, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 3, 0,
                             2, 0, 0, 0, 3, 1, 0, 0, 1, 3, 0, 0, 0, 0,
                             0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 1, 3, 1, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 3, 1, 3,
                             0, 0, 1, 3, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             2, 3, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                             0, 0, 3, 3, 0, 1, 0, 0, 0, 0, 2, 2, 0, 0,
                             2, 0, 0, 0]

    cdef cnp.uint8_t first_pass, pixel_removed, neighbors
    cdef cnp.uint8_t c_distance = 1

    # indices for fast iteration
    cdef Py_ssize_t row, col, nrows, ncols, pass_num
    nrows, ncols = image.shape[:2]
    nrows += 2
    ncols += 2

    # Copy over the image into a larger version with a single pixel border
    # this removes the need to handle border cases below
    cdef cnp.uint8_t [:, ::1] skeleton = np.zeros((nrows, ncols),
                                                  dtype=np.uint8)
    cdef cnp.float32_t [:, ::1] sdf = np.zeros((nrows, ncols),
                                                  dtype=np.float32)
    skeleton[1:-1, 1:-1] = image
    cdef cnp.uint8_t [:, ::1] cleaned_skeleton = skeleton.copy()

    pixel_removed = True

    # the algorithm reiterates the thinning till
    # no further thinning occurred (variable pixel_removed set)
    with nogil:
        while pixel_removed:
            pixel_removed = False

            # there are two phases, in the first phase, pixels labeled
            # (see below) 1 and 3 are removed, in the second 2 and 3

            # nogil can't iterate through `(True, False)` because it is a Python
            # tuple. Use the fact that 0 is Falsy, and 1 is truthy in C
            # for the iteration instead.
            # for first_pass in (True, False):
            for pass_num in range(2):
                first_pass = (pass_num == 0)
                for row in range(1, nrows-1):
                    for col in range(1, ncols-1):
                        # all set pixels ...

                        if skeleton[row, col]:
                            # are correlated with a kernel
                            # (coefficients spread around here ...)
                            # to apply a unique number to every
                            # possible neighborhood ...

                            # which is used with the lut to find the
                            # "connectivity type"

                            neighbors = lut[skeleton[row - 1, col - 1] +
                                            2 * skeleton[row - 1, col] +
                                            4 * skeleton[row - 1, col + 1] +
                                            8 * skeleton[row, col + 1] +
                                            16 * skeleton[row + 1, col + 1] +
                                            32 * skeleton[row + 1, col] +
                                            64 * skeleton[row + 1, col - 1] +
                                            128 * skeleton[row, col - 1]]

                            if (neighbors == 0):
                                continue
                            elif ((neighbors == 3) or
                                  (neighbors == 1 and first_pass) or
                                  (neighbors == 2 and not first_pass)):
                                # Remove the pixel
                                cleaned_skeleton[row, col] = 0
                                sdf[row, col] = c_distance
                                pixel_removed = True

                # once a step has been processed, the original skeleton
                # is overwritten with the cleaned version
                skeleton[:, :] = cleaned_skeleton[:, :]
            c_distance += 1

    return skeleton.base[1:-1, 1:-1], sdf.base[1:-1, 1:-1]
