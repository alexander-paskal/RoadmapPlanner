from spatialgeometry import Cuboid
from spatialmath import SE3

_OBS = None
def get_obstacles():
    a = 30
    b = 1
    c = 35
    d = 5
    e = 11
    f = 28
    global _OBS
    if _OBS is None:

        obstacle1 = Cuboid(scale=[b, a, 1], pose=SE3(-e - b / 2, d + a / 2, 0.5), collision=True)
        obstacle2 = Cuboid(scale=[b, c, 1], pose=SE3(-e - b / 2, -c / 2, 0.5), collision=True)
        obstacle3 = Cuboid(scale=[b, c, 1], pose=SE3(e + b / 2, c / 2, 0.5), collision=True)
        obstacle4 = Cuboid(scale=[b, a, 1], pose=SE3(e + b / 2, -d - a / 2, 0.5), collision=True)
        _OBS = [obstacle1, obstacle2, obstacle3, obstacle4]

    return _OBS