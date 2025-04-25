import pybullet as p
import pybullet_data
import time
import random
import numpy as np
import tkinter as tk

# --- PARAMETERS ---
GRID_SIZE          = 50        # terrain resolution
CELL_SIZE          = 1         # meters per grid cell
DRONE_RADIUS       = 0.5
START_CELL         = (0, 0)
END_CELL           = (GRID_SIZE-1, GRID_SIZE-1)
OBSTACLE_PROB      = 0.05      # static buildings
TREE_PROB          = 0.02      # static trees
CLEARANCE          = 2.0       # drone altitude above ground
LINE_WIDTH         = 2
STEPS_PER_SEGMENT  = 20
STEP_SLEEP         = 1/60

# moving spheres
NUM_SPHERES        = 5
SPHERE_RADIUS      = 0.5
SPHERE_SPEED       = 2.0       # m/s

# --- A* ON A 2D OCC GRID ---
def heuristic(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def astar(start, goal, occ):
    open_set = {start}
    came_from = {}
    g = {start: 0}
    f = {start: heuristic(start, goal)}
    while open_set:
        current = min(open_set, key=lambda x: f[x])
        if current == goal:
            # reconstruct
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        open_set.remove(current)
        x,y = current
        for dx,dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nb = (x+dx, y+dy)
            if 0<=nb[0]<GRID_SIZE and 0<=nb[1]<GRID_SIZE and not occ[nb]:
                tg = g[current] + 1
                if tg < g.get(nb, 1e9):
                    came_from[nb] = current
                    g[nb] = tg
                    f[nb] = tg + heuristic(nb, goal)
                    open_set.add(nb)
    return []

# --- HEIGHTFIELD GENERATOR ---
def make_heightfield():
    hf = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
    for _ in range(10):
        cx, cy = random.uniform(0, GRID_SIZE), random.uniform(0, GRID_SIZE)
        h = random.uniform(2,8)
        w = random.uniform(5,15)
        X, Y = np.meshgrid(np.arange(GRID_SIZE), np.arange(GRID_SIZE), indexing='ij')
        hf += h * np.exp(-((X-cx)**2 + (Y-cy)**2)/(2*w*w))
    hf = (hf / hf.max()) * 5.0
    return hf

# --- DYNAMIC SPHERES SETUP & UPDATE ---
def init_spheres(static_occ, hf):
    spheres = []
    for _ in range(NUM_SPHERES):
        # pick a random free cell
        while True:
            i = random.randrange(GRID_SIZE)
            j = random.randrange(GRID_SIZE)
            if not static_occ[i,j] and (i,j) not in (START_CELL, END_CELL):
                break
        x = (i+0.5)*CELL_SIZE
        y = (j+0.5)*CELL_SIZE
        z = hf[i,j] + CLEARANCE + random.uniform(-1,1)
        v = np.random.randn(2)
        v = (v/np.linalg.norm(v))*SPHERE_SPEED
        # sphere body
        col = p.createCollisionShape(p.GEOM_SPHERE, radius=SPHERE_RADIUS)
        vis = p.createVisualShape(p.GEOM_SPHERE, radius=SPHERE_RADIUS, rgbaColor=[1,1,0,1])
        body = p.createMultiBody(0, col, vis, [x,y,z])
        spheres.append({'id': body, 'pos': np.array([x,y,z]), 'vel': np.array([v[0],v[1],0])})
    return spheres

def update_spheres(spheres):
    for s in spheres:
        pos = s['pos'] + s['vel'] * STEP_SLEEP
        # bounce off the boundaries of the terrain
        for ax in (0,1):
            lim = GRID_SIZE*CELL_SIZE
            if pos[ax] < 0 or pos[ax] > lim:
                s['vel'][ax] *= -1
                pos[ax] = np.clip(pos[ax], 0, lim)
        s['pos'] = pos
        p.resetBasePositionAndOrientation(s['id'], pos.tolist(), [0,0,0,1])

def make_dynamic_occ(static_occ, spheres):
    occ = static_occ.copy()
    for s in spheres:
        i = int(s['pos'][0]//CELL_SIZE)
        j = int(s['pos'][1]//CELL_SIZE)
        # mark neighboring cells within SPHERE_RADIUS
        r = int(np.ceil(SPHERE_RADIUS/CELL_SIZE))
        for di in range(-r, r+1):
            for dj in range(-r, r+1):
                ii, jj = i+di, j+dj
                if 0<=ii<GRID_SIZE and 0<=jj<GRID_SIZE:
                    occ[ii,jj] = True
    return occ

# --- MAIN SIMULATION ---
def simulate():
    # PyBullet init
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)
    p.setGravity(0,0,-9.8)
    p.loadURDF("plane.urdf")

    # terrain
    hf = make_heightfield()
    terrain = p.createCollisionShape(
        p.GEOM_HEIGHTFIELD, meshScale=[CELL_SIZE]*2+[1],
        heightfieldTextureScaling=GRID_SIZE/2,
        heightfieldData=hf.flatten().tolist(),
        numHeightfieldRows=GRID_SIZE,
        numHeightfieldColumns=GRID_SIZE
    )
    p.createMultiBody(0, terrain)
    p.resetBasePositionAndOrientation(terrain,
        [GRID_SIZE*CELL_SIZE/2]*2+[0], [0,0,0,1])

    # static obstacles
    static_occ = np.zeros((GRID_SIZE, GRID_SIZE), bool)
    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            if (i,j) in (START_CELL, END_CELL): continue
            base_z = hf[i,j]
            x,y = i*CELL_SIZE, j*CELL_SIZE
            if random.random()<OBSTACLE_PROB:
                static_occ[i,j] = True
                h = random.uniform(3,8)
                vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.4]*2+[h/2],
                                          rgbaColor=[0.8,0.2,0.2,1])
                col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.4]*2+[h/2])
                p.createMultiBody(0, col, vis, [x, y, base_z+h/2])
            elif random.random()<TREE_PROB:
                # trees don’t block flight path
                trunk = p.createVisualShape(p.GEOM_CYLINDER, radius=0.1,
                                            length=1.0, rgbaColor=[0.4,0.2,0,1])
                leaf  = p.createVisualShape(p.GEOM_SPHERE, radius=0.5,
                                            rgbaColor=[0,0.6,0,1])
                col_tr = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.1, height=1.0)
                p.createMultiBody(0, col_tr, trunk, [x,y,base_z+0.5])
                p.createMultiBody(0, -1, leaf,  [x,y,base_z+1.5])

    # init dynamic spheres
    spheres = init_spheres(static_occ, hf)

    # create drone
    drone_vis = p.createVisualShape(p.GEOM_SPHERE,
        radius=DRONE_RADIUS, rgbaColor=[0,0,1,1])
    drone_col = p.createCollisionShape(p.GEOM_SPHERE, radius=DRONE_RADIUS)
    start_z = hf[START_CELL] + CLEARANCE
    drone = p.createMultiBody(1, drone_col, drone_vis,
        [START_CELL[0]*CELL_SIZE, START_CELL[1]*CELL_SIZE, start_z])

    # cinematic camera base
    base_dist, base_yaw, base_pitch = 18, 0, -20
    total_steps = 0  # will count frames

    # runtime replanning loop
    current = START_CELL
    while current != END_CELL:
        # update occupancy with moving spheres
        dyn_occ = make_dynamic_occ(static_occ, spheres)
        # plan from current → goal
        path = astar(current, END_CELL, dyn_occ)
        if not path:
            print("No safe path!")
            return
        # draw new path
        p.removeAllUserDebugItems()
        for a,b in zip(path, path[1:]):
            z1 = hf[a] + CLEARANCE
            z2 = hf[b] + CLEARANCE
            p.addUserDebugLine(
                [a[0]*CELL_SIZE, a[1]*CELL_SIZE, z1],
                [b[0]*CELL_SIZE, b[1]*CELL_SIZE, z2],
                [0,1,0], LINE_WIDTH
            )

        # follow just the first segment
        nxt = path[1]
        p1 = np.array([current[0]*CELL_SIZE, current[1]*CELL_SIZE,
                       hf[current]+CLEARANCE])
        p2 = np.array([nxt[0]*CELL_SIZE,    nxt[1]*CELL_SIZE,
                       hf[nxt]   +CLEARANCE])

        # animate this segment
        for t in np.linspace(0,1,STEPS_PER_SEGMENT):
            # move spheres
            update_spheres(spheres)
            # interpolate drone
            pos = (1-t)*p1 + t*p2
            p.resetBasePositionAndOrientation(drone, pos.tolist(), [0,0,0,1])
            p.stepSimulation()

            # cinematic camera
            total_steps += 1
            yaw   = base_yaw   + (total_steps/500)*360
            pitch = base_pitch + 10*np.sin(total_steps/50)
            dist  = base_dist  + 3*np.sin(total_steps/30)
            p.resetDebugVisualizerCamera(dist, yaw, pitch, pos.tolist())

            time.sleep(STEP_SLEEP)

        current = nxt

    print("Mission complete!")

# --- GUI ---
def run_gui():
    root = tk.Tk()
    root.title("Drone with Dynamic Obstacles")
    tk.Button(root, text="Run Simulation", command=simulate).pack(padx=20, pady=20)
    root.mainloop()

if __name__ == "__main__":
    run_gui()
