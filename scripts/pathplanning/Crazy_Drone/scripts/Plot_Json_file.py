#!/usr/bin/env python
import math
import json
import matplotlib.pyplot as plt

# Load world JSON
with open('/home/maverick/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/awesome.world.json') as f:
	world = json.load(f)
#print world.keys()

air = world['airspace']
mark = world['markers']
wall = world['walls']
gate = world['gates']
print air,'\n\n',mark,'\n\n',wall,'\n\n',gate

