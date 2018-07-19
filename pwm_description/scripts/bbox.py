#!/usr/bin/env python2

"""
Usage : 
    python bbox.py [STL_FILE] | xclip -selection clipboard
Example :
    python bbox.py caster_fl_link.STL | xclip -selection clipboard

"""
import stl
from stl import mesh
import numpy
import sys

def find_mins_maxs(obj):
    minx = maxx = miny = maxy = minz = maxz = None
    for p in obj.points:
        # p contains (x, y, z)
        if minx is None:
            minx = p[stl.Dimension.X]
            maxx = p[stl.Dimension.X]
            miny = p[stl.Dimension.Y]
            maxy = p[stl.Dimension.Y]
            minz = p[stl.Dimension.Z]
            maxz = p[stl.Dimension.Z]
        else:
            maxx = max(p[stl.Dimension.X], maxx)
            minx = min(p[stl.Dimension.X], minx)
            maxy = max(p[stl.Dimension.Y], maxy)
            miny = min(p[stl.Dimension.Y], miny)
            maxz = max(p[stl.Dimension.Z], maxz)
            minz = min(p[stl.Dimension.Z], minz)
    return minx, maxx, miny, maxy, minz, maxz

def as_box(minx,maxx,miny,maxy,minz,maxz):
    sx = (maxx - minx)
    sy = (maxy - miny)
    sz = (maxz - minz)

    mx = (maxx + minx) / 2.0
    my = (maxy + miny) / 2.0
    mz = (maxz + minz) / 2.0

    s = ''
    s += '<collision>\n'
    s += '<origin xyz="{0} {1} {2}" rpy="0 0 0"/>\n'.format(mx,my,mz)
    s += '<geometry>\n'
    s += '<box size="{0} {1} {2}"/>\n'.format(sx,sy,sz)
    s += '</geometry>\n'
    s += '</collision>\n'
    return s

def main():
    if len(sys.argv) > 1:
        ms = sys.argv[1]
        main_body = mesh.Mesh.from_file(ms)
        print as_box(*find_mins_maxs(main_body))

if __name__ == "__main__":
    main()
