# All-in-one Mine Evacuation Simulation (Colab-ready)
# - Medium-complex branching mine (clean layout)
# - Dynamic CO sensors (noise, spikes, diffusion)
# - Cost = distance * (1 + safety_bias * normalized_CO) -> Dijkstra
# - Workers move smoothly along edges and replan periodically
# - Records frames and exposes interactive slider

import math, random, copy, time
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
from ipywidgets import interact, IntSlider

# ---------------------------
# Configuration (tweakable)
# ---------------------------
RANDOM_SEED = 42
NUM_BRANCHES = 6            # radial branches from hub
BRANCH_LEN_RANGE = (8, 14)  # Increased length slightly so we can see them run longer
INTERCONNECT_PROB = 0.15    # chance to create cross-links
NUM_WORKERS = 5
SAFETY_BIAS = 8.0           # Increased bias: Workers really hate CO now
TIMESTEPS = 200             # number of recorded frames
TIMESTEP_SEC = 0.5          # logical seconds per simulation step
WORKER_SPEED = 1.6          # units (graph distance) per second
SENSOR_NOISE_STD = 1.0      # Reduced noise so the "Event" is clearer
SPIKE_PROB = 0.01           # Low random spikes (we will use a manual spike)
SPIKE_MAG = (30, 100)       # spike magnitude in ppm
DIFFUSION_FACTOR = 0.14
REPLAN_INTERVAL = 5         # replan frequently to react to the gas

random.seed(RANDOM_SEED)
np.random.seed(RANDOM_SEED)

# ---------------------------
# 1) Generate medium-complex radial mine (clear layout)
# ---------------------------
def generate_mine_graph(num_branches=NUM_BRANCHES, branch_len_range=BRANCH_LEN_RANGE, interconnect_prob=INTERCONNECT_PROB):
    G = nx.Graph()
    hub = "hub"
    G.add_node(hub, pos=(0.0, 0.0), kind="hub")
    branch_ends = []
    for b in range(num_branches):
        prev = hub
        angle = (2*math.pi) * b / num_branches + random.uniform(-0.22, 0.22)
        length = random.randint(*branch_len_range)
        for i in range(length):
            r = (i+1) * 2.8 + random.uniform(-0.2, 0.2)
            x = r * math.cos(angle) + random.uniform(-0.15, 0.15)
            y = r * math.sin(angle) + random.uniform(-0.15, 0.15)
            name = f"B{b}_N{i}"
            G.add_node(name, pos=(x,y), kind="tunnel")
            dist = math.hypot(x - G.nodes[prev]['pos'][0], y - G.nodes[prev]['pos'][1])
            G.add_edge(prev, name, length=dist)
            prev = name
        # add an exit node slightly beyond
        exit_name = f"EXIT_{b}"
        angle2 = angle + random.uniform(-0.08, 0.08)
        x = (length + 1.15) * 2.8 * math.cos(angle2)
        y = (length + 1.15) * 2.8 * math.sin(angle2)
        G.add_node(exit_name, pos=(x,y), kind="exit")
        d = math.hypot(x - G.nodes[prev]['pos'][0], y - G.nodes[prev]['pos'][1])
        G.add_edge(prev, exit_name, length=d)
        branch_ends.append(exit_name)

    # add random interconnects between nodes to make navigation interesting
    nodes = [n for n in G.nodes() if n != 'hub']
    for i in range(len(nodes)):
        for j in range(i+1, len(nodes)):
            if random.random() < interconnect_prob:
                a, b = nodes[i], nodes[j]
                pa = G.nodes[a]['pos']; pb = G.nodes[b]['pos']
                d = math.hypot(pa[0]-pb[0], pa[1]-pb[1])
                if 0.8 < d < 9.0:
                    G.add_edge(a,b, length=d)

    # add a few dead-end pockets
    for _ in range(int(num_branches/2)):
        anchor = random.choice([n for n in G.nodes() if G.nodes[n]['kind']=='tunnel'])
        ax, ay = G.nodes[anchor]['pos']
        ang = random.uniform(0, 2*math.pi)
        r = random.uniform(0.6, 1.6)
        name = f"POCK_{random.randint(100,9999)}"
        G.add_node(name, pos=(ax + r*math.cos(ang), ay + r*math.sin(ang)), kind='pocket')
        d = math.hypot(G.nodes[name]['pos'][0]-ax, G.nodes[name]['pos'][1]-ay)
        G.add_edge(anchor, name, length=d)

    return G

G = generate_mine_graph()
positions = {n: d['pos'] for n,d in G.nodes(data=True)}
exit_nodes = [n for n,d in G.nodes(data=True) if d.get('kind') == 'exit']
print("Exits:", exit_nodes)

# ---------------------------
# 2) Sensors: one per node (initial)
# ---------------------------
def init_sensors(G, base_co=5.0):
    return {n: base_co + random.uniform(-0.5, 0.5) for n in G.nodes()}

sensors = init_sensors(G)

# ---------------------------
# 3) Helpers for CO -> edge weights
# ---------------------------
def get_node_co(sensors):
    return dict(sensors)

def compute_edge_weights(G, node_co, safety_bias=SAFETY_BIAS):
    vals = np.array(list(node_co.values()))
    minv, maxv = float(vals.min()), float(vals.max())
    denom = maxv - minv if maxv > minv else 1.0
    node_norm = {n: (v - minv)/denom for n,v in node_co.items()}
    weights = {}
    for u,v,data in G.edges(data=True):
        d = data.get('length', math.hypot(G.nodes[u]['pos'][0]-G.nodes[v]['pos'][0],
                                         G.nodes[u]['pos'][1]-G.nodes[v]['pos'][1]))
        co_avg = 0.5*(node_norm.get(u,0.0) + node_norm.get(v,0.0))
        
        # Non-linear penalty: if CO is high, weight explodes
        # This ensures they treat high CO as a 'wall' unless no other choice
        penalty = 1.0 + safety_bias * (co_avg ** 2) 
        
        w = d * penalty
        weights[(u,v)] = weights[(v,u)] = w
    return weights

# Dijkstra using weights dict
import heapq
def safest_shortest_path(G, weights, source, targets):
    pq = [(0.0, source)]
    seen = {}
    parent = {source: None}
    while pq:
        cost, node = heapq.heappop(pq)
        if node in seen:
            continue
        seen[node] = cost
        if node in targets:
            path = []
            cur = node
            while cur is not None:
                path.append(cur)
                cur = parent.get(cur, None)
            path.reverse()
            return path
        for nb in G.neighbors(node):
            if nb in seen: continue
            w = weights.get((node,nb), None)
            if w is None:
                p1 = G.nodes[node]['pos']; p2 = G.nodes[nb]['pos']
                w = math.hypot(p1[0]-p2[0], p1[1]-p2[1])
            new_cost = cost + w
            parent[nb] = node
            heapq.heappush(pq, (new_cost, nb))
    return None

# ---------------------------
# 4) Sensors update: noise + spikes + diffusion
# ---------------------------
def sensors_step(sensors, force_spike=False):
    # local noise
    for n in list(sensors.keys()):
        sensors[n] = max(0.0, sensors[n] + random.gauss(0, SENSOR_NOISE_STD))
        # Occasional random spikes (kept low to emphasize the main event)
        if not force_spike and random.random() < SPIKE_PROB:
            sensors[n] += random.uniform(*SPIKE_MAG)
            
    # diffusion
    new = sensors.copy()
    for n in G.nodes():
        neighs = list(G.neighbors(n))
        if not neighs: continue
        for nb in neighs:
            transfer = DIFFUSION_FACTOR * sensors[n] / max(1, len(neighs))
            new[nb] = max(0.0, new[nb] + transfer)
    # decay
    for n in new:
        new[n] = max(0.0, new[n] * 0.995)
    return new

# ---------------------------
# 5) Workers: FORCED START AT HUB
# ---------------------------
# Change: All workers start at the Hub to force them to navigate out
workers = {}
for i in range(NUM_WORKERS):
    start = "hub" 
    workers[f"W{i}"] = {
        "node": start,
        "pos": np.array(G.nodes[start]['pos'], dtype=float),
        "path": [],           
        "edge_progress": 0.0, 
        "evacuated": False
    }

# initial planning
def plan_all():
    node_co = get_node_co(sensors)
    weights = compute_edge_weights(G, node_co, safety_bias=SAFETY_BIAS)
    for wid, st in workers.items():
        if st['evacuated']: 
            st['path'] = []
            continue
        src = st['node']
        path = safest_shortest_path(G, weights, src, set(exit_nodes))
        st['path'] = path if path is not None else []

plan_all()

# ---------------------------
# 6) Movement along edges (strictly)
# ---------------------------
def move_worker_along_path(worker, dt):
    if worker['evacuated']: return
    path = worker['path']
    if not path or len(path) == 1:
        if path and G.nodes[path[0]].get('kind')=='exit':
            worker['evacuated'] = True
        return

    cur, nxt = path[0], path[1]
    pcur = np.array(G.nodes[cur]['pos'], dtype=float)
    pnext = np.array(G.nodes[nxt]['pos'], dtype=float)
    edge_len = np.linalg.norm(pnext - pcur)
    
    if edge_len < 1e-6:
        worker['pos'] = pnext.copy()
        worker['node'] = nxt
        worker['path'] = path[1:]
        worker['edge_progress'] = 0.0
        return

    prog = worker.get('edge_progress', 0.0)
    frac = (WORKER_SPEED * dt) / edge_len
    prog += frac
    if prog >= 1.0:
        worker['pos'] = pnext.copy()
        worker['node'] = nxt
        worker['path'] = path[1:]
        worker['edge_progress'] = 0.0
        if G.nodes[nxt].get('kind') == 'exit':
            worker['evacuated'] = True
    else:
        worker['pos'] = pcur + (pnext - pcur) * prog
        worker['edge_progress'] = prog

# ---------------------------
# 7) Simulation loop: THE DISASTER EVENT
# ---------------------------
history = []
last_replan_time = 0

print("Simulating... Watch for the event at step 15!")

for step in range(TIMESTEPS):
    # Snapshots
    snapshot = {
        'co': copy.deepcopy(sensors),
        'workers': {wid: {'pos': st['pos'].copy(), 'node': st['node'], 'path': list(st['path']), 'evacuated': st['evacuated']} for wid,st in workers.items()}
    }
    history.append(snapshot)

    # --- SCENARIO INJECTION ---
    # At step 15, simulate a massive leak in Branch 0, 1, and 5.
    # This blocks 50% of the exits, forcing workers who picked those to turn back.
    if step == 15:
        print("!!! GAS EXPLOSION IN BRANCHES 0, 1, 5 !!!")
        for n in G.nodes():
            # Target specific branches by name
            if "B0_" in n or "B1_" in n or "B5_" in n:
                sensors[n] += 400.0 # Massive spike
            if n == "hub":
                sensors[n] += 50.0 # Slight leak at hub to make it urgent

    # Normal dynamics
    sensors = sensors_step(sensors)

    # Replan logic
    if (step - last_replan_time) >= REPLAN_INTERVAL:
        plan_all()
        last_replan_time = step

    # Move
    dt = TIMESTEP_SEC
    for wid in workers:
        move_worker_along_path(workers[wid], dt)

# Final frame
history.append({
    'co': copy.deepcopy(sensors),
    'workers': {wid: {'pos': st['pos'].copy(), 'node': st['node'], 'path': list(st['path']), 'evacuated': st['evacuated']} for wid,st in workers.items()}
})

print(f"Done. Generated {len(history)} frames.")

# ---------------------------
# 8) Interactive slider viewer
# ---------------------------
all_co = []
for f in history:
    all_co.extend(list(f['co'].values()))
# Clamp vmax to visualize the 'danger' zones clearly without washing out low levels
vmin, vmax = 0, 150 
norm = Normalize(vmin=vmin, vmax=vmax)
cmap = plt.cm.inferno
sm = ScalarMappable(norm=norm, cmap=cmap)
sm.set_array([])

def draw_frame(frame_i):
    frame = history[frame_i]
    co_map = frame['co']
    workers_state = frame['workers']

    fig, ax = plt.subplots(figsize=(11,8))
    title_text = f"Step {frame_i}"
    if frame_i < 15: title_text += " (Normal)"
    elif frame_i == 15: title_text += " (EXPLOSION!)"
    else: title_text += " (Evacuation)"
    
    ax.set_title(title_text, fontsize=14)

    # Edges
    for u,v in G.edges():
        x1,y1 = G.nodes[u]['pos']; x2,y2 = G.nodes[v]['pos']
        ax.plot([x1,x2],[y1,y2], color='lightgray', linewidth=1.2, zorder=1)

    # Nodes
    mapped = [cmap(norm(co_map[n])) for n in G.nodes()]
    for n in G.nodes():
        x,y = G.nodes[n]['pos']
        kind = G.nodes[n].get('kind','tunnel')
        size = 160 if kind=='exit' else 80 if kind=='hub' else 40
        edgecol = 'black' if kind=='exit' else 'gray'
        # Highlight hub
        if kind == 'hub': edgecol = 'blue'
        
        ax.scatter([x],[y], s=size, color=mapped[list(G.nodes()).index(n)], edgecolor=edgecol, zorder=3)

    # Paths
    for wid, st in workers_state.items():
        path = st['path']
        if path and len(path) >= 2:
            xs = [G.nodes[n]['pos'][0] for n in path]
            ys = [G.nodes[n]['pos'][1] for n in path]
            ax.plot(xs, ys, linestyle='--', linewidth=2.0, color='lime', alpha=0.6, zorder=2)

    # Workers
    for wid, st in workers_state.items():
        px, py = st['pos']
        if st['evacuated']:
            ax.scatter([px],[py], s=200, marker='*', color='cyan', edgecolors='black', zorder=6)
        else:
            ax.scatter([px],[py], s=120, marker='o', color='dodgerblue', edgecolors='white', zorder=6)
            ax.text(px+0.15, py+0.15, wid, color='black', fontsize=9, fontweight='bold')

    # Exits
    for ex in exit_nodes:
        exx, exy = G.nodes[ex]['pos']
        ax.text(exx, exy+0.18, "EXIT", color='green', fontsize=9, ha='center')

    cbar = fig.colorbar(sm, ax=ax, fraction=0.04, pad=0.02)
    cbar.set_label("CO (ppm)")
    ax.set_aspect("equal")
    ax.set_xticks([]); ax.set_yticks([])
    plt.show()

slider = IntSlider(min=0, max=len(history)-1, step=1, value=0, description='frame')
interact(draw_frame, frame_i=slider)
