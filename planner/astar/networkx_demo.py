import timeit

import matplotlib.pyplot as plt
import networkx as nx
import numpy as np

from planner.astar import astar_grid48con
from tools import load_map

map = load_map('o.png')
print(map)
map = map[:, ::2]
print(map)
#print [list(i) for i in zip(*map)]

n = map.shape[1]
print map.shape

G = nx.grid_graph([n, n])
#print ("G: ", G)

start = (1, 7)
goal = (7, 1)
assert map[start] >= 0, "start in obstacle"
assert map[goal] >= 0, "goal in obstacle, %d" % map[goal]
#print ("Start: ", start)
#print ("End: ", goal)

def cost(a, b):
    if map[a] >= 0 and map[b] >= 0:  # no obstacle
      return np.linalg.norm(np.array(a)-np.array(b))
    else:
        #print ("else: ", np.Inf)
        return np.Inf

obstacle = []
for n in G.nodes():
    if not map[n] >= 0:  # obstacle
        obstacle.append(n)
print ("obstacle: ", obstacle)
G.remove_nodes_from(obstacle)

t = timeit.Timer()
t.timeit()

path = nx.astar_path(G, start, goal, cost)

print("Path::: ", path)
print("computation time:", t.repeat(), "s")

print("length: ", astar_grid48con.path_length(path))

fig, ax = plt.subplots()

ax.imshow(map.T, cmap='Greys', interpolation='nearest')
ax.set_title('astar path')
ax.axis([0, map.shape[0], 0, map.shape[1]])
ax.plot(
    np.array(np.matrix(path)[:, 0]).flatten(),
    np.array(np.matrix(path)[:, 1]).flatten(),
    c='b',
    lw=2
)

#n = G.number_of_nodes()
#if n < 500:
#    plt.figure()
#    pos = nx.spring_layout(G, iterations=1000, k=.1 / np.sqrt(n))
#    nx.draw(G, pos)
#
#plt.show()
