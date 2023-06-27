import sys
from math import pi , acos , sin , cos
from collections import deque
from heapq import heappush, heappop, heapify
def calcd(node1, node2):
   y1, x1 = node1
   y2, x2 = node2

   R   = 3958.76 # miles = 6371 km
   y1 *= pi/180.0
   x1 *= pi/180.0
   y2 *= pi/180.0
   x2 *= pi/180.0

   # approximate great circle distance with law of cosines
   return acos( sin(y1)*sin(y2) + cos(y1)*cos(y2)*cos(x2-x1) ) * R
def xyCoordinates(node1, node2):
   y1, x1 = node1
   y2, x2 = node2

   # approximate great circle distance with law of cosines
   return ([50 + 10 * (x1 + 130), 800 - (y1 * 10)], [50 + 10 * (x2 + 130), 800 - (y2 * 10)])
def heuristic(startstate, endstate, nodes):
    if startstate == endstate:
        return 0
    else:
        x2 = calcd((float(nodes[startstate][0]), float(nodes[startstate][1])), (float(nodes[endstate][0]), float(nodes[endstate][1])))
        return x2
def astar(start, end, nodes, backupStructure):
    closed = set()
    startnode = (heuristic(start, end, nodes), 0, start, [nodes[start]])     
    fringe = []
    heappush(fringe, startnode)
    while len(fringe) > 0:
        f, depth, v, path = heappop(fringe)
        if v == end:
            return path
        if v not in closed:
            closed.add(v)
            for c in backupStructure[v]:
                c2, d = c
                if c2 not in closed:
                    c_path = path.copy()
                    c_path.append(nodes[c2])
                    temp = (heuristic(c2, end, nodes) + depth + d, depth + d, c2, c_path)
                    heappush(fringe, temp)
try:
  a = sys.argv[1]
  b = sys.argv[2]
  # print("Hello")
  # print(sys.argv[3])
  c = sys.argv[3].split(", ")[:len(sys.argv[3].split(", "))-1]
  d2 = sys.argv[4].split(",")
  d = []
  for x in range(0, len(d2), 2): d.append([d2[x], d2[x+1]])
  e2 = sys.argv[5].split(",")
  e = []
  for x in range(0, len(e2), 4): e.append([[e2[x], e2[x+1]], [e2[x+2], e2[x+3]]])
  t1 = []
  coordDict = dict()
  for x in range(0, len(c)): coordDict[c[x]] = d[x]
  backupStructure = dict()
  for x in e:
    coord1 = [x[0][0], x[0][1]]
    coord2 = [x[1][0], x[1][1]]
    i1 = d.index(coord1)
    i2 = d.index(coord2)
    x2 = calcd((float(x[0][0]), float(x[0][1])), (float(x[1][0]), float(x[1][1])))
    t1.append([xyCoordinates((float(x[0][0]), float(x[0][1])), (float(x[1][0]), float(x[1][1]))), c[i1], c[i2]])
    if c[i1] not in backupStructure:
        x3 = set()
        x3.add((c[i2], x2))
        backupStructure[c[i1]] = x3
    else:
        backupStructure[c[i1]].add((c[i2], x2))
    if c[i2] not in backupStructure:
        x4 = set()
        x4.add((c[i1], x2))
        backupStructure[c[i2]] = x4
    else:
        backupStructure[c[i2]].add((c[i1], x2))
  #sys.stdout.write(astar(a, b, coordDict, backupStructure))
  toRet = ""
  a = astar(a, b, coordDict, backupStructure)
  for x in a:
    for y in x: toRet += y + ","
  sys.stdout.write(toRet[0:len(toRet)-1])
except:
  print("DATA_MISSING")