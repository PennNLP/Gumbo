#!/usr/bin/env python

""" Convert maps from Lowell's JSON format to LTLMoP's .region format.
    Outputs resulting "calibration matrix" that LTLMoP will use for pose transformation.
"""

import os
import sys
import json
from numpy import *

# find LTLMoP repo
ltlmop_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "LTLMoP")
sys.path.append(os.path.join(ltlmop_root, "src", "lib"))

# load in the LTLMoP library
import regions

# load in map file
print "Loading '{}'...".format(sys.argv[1])
with open(sys.argv[1]) as f:
    data = json.load(f)

# create LTLMoP regions
rfi = regions.RegionFileInterface()

for region in data['WorldMap']['Regions']:
    newRegion = regions.Region(points=[])
    newRegion.name = str(region['name'])

    for pt in region['boundary']:
        newRegion.pointArray.append(regions.Point(pt[0], pt[1]))

    rfi.regions.append(newRegion)

# calculate transformation matrix (because LTLMoP regions are antiquated)
# flip over y-axis, scale up to make width be 1000, and topleft be (100,100)
leftMargin, topMargin, rightExtent, downExtent = rfi.getBoundingBox()
scale_factor = 1000.0/rightExtent
offset = regions.Point(100,100) - regions.Point(leftMargin, -(topMargin+downExtent))*scale_factor
T = matrix([[scale_factor, 0, offset.x],
            [0, -scale_factor, offset.y],
            [0, 0, 1]])

# apply transformation

# from lib/project.py
coordmap_lab2map = lambda pt: regions.Point(*((T * mat([pt.x, pt.y, 1]).T).T.tolist()[0][0:2]))

for region in rfi.regions:
    region.pointArray = map(coordmap_lab2map, region.pointArray)
    region.recalcBoundingBox()

# force topology (we avoid LTLMoP's automatic coincident edge detection)
rfi.transitions = [[[] for j in range(len(rfi.regions))] for i in range(len(rfi.regions))]
for path in data['WorldMap']['Topology']:
    r1 = rfi.indexOfRegionWithName(path[0])
    r2 = rfi.indexOfRegionWithName(path[1])
    fake_face = [frozenset((regions.Point(1,2), regions.Point(3,4)))]
    rfi.transitions[r1][r2] = fake_face
    rfi.transitions[r2][r1] = fake_face

# save file
output_filename = os.path.splitext(sys.argv[1])[0] + ".regions"
rfi.writeFile(output_filename)

print "Wrote to '{}'.".format(output_filename)

print "The calculated calibration matrix is: "
print T
print "You can just squirrel this into the appropriate .config file"
