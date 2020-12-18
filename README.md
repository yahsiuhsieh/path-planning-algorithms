# A* Path Planning

A* path planning in various 3D environments

<p align="center">
  <img width="640" height="480" src="https://github.com/arthur960304/astar-path-planning/blob/main/results/flappy_bird.png"/>
</p>

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Built With

* Python 3.6.10

* numpy >= 1.16.2

* matplotlib >= 3.1.1

## Code Organization

```
.
├── src                # Scripts
│   ├── maps           # Folder contains all envs
│   ├── main.py        # Code to create env and call the planner
│   └── Planner.py     # Main A* algorithm
├── results            # Path visualization in .png
├── doc                # Detailed info
└── README.md
```

## How to Run

There are 7 3D environments that you can try, as shown in [The Envs section](#the-envs)

Run the following commands to excecute the A* algorithm.

```
python main.py
```

## The Envs

### Cube

In the `main.py` file
```python
test_single_cube(True)
```

<p align="center">
  <img width="480" height="360" src="https://github.com/arthur960304/astar-path-planning/blob/main/results/cube.png"/>
</p>

### Flappy Bird

In the `main.py` file
```python
test_flappy_bird(True)
```

<p align="center">
  <img width="480" height="360" src="https://github.com/arthur960304/astar-path-planning/blob/main/results/flappy_bird.png"/>
</p>

### Maze

In the `main.py` file
```python
test_maze(True)
```

<p align="center">
  <img width="480" height="360" src="https://github.com/arthur960304/astar-path-planning/blob/main/results/maze.png"/>
</p>

### Monza

In the `main.py` file
```python
test_monza(True)
```

<p align="center">
  <img width="480" height="360" src="https://github.com/arthur960304/astar-path-planning/blob/main/results/monza.png"/>
</p>

### Room

In the `main.py` file
```python
test_room(True)
```

<p align="center">
  <img width="480" height="360" src="https://github.com/arthur960304/astar-path-planning/blob/main/results/room.png"/>
</p>

### Tower

In the `main.py` file
```python
test_tower(True)
```

<p align="center">
  <img width="480" height="360" src="https://github.com/arthur960304/astar-path-planning/blob/main/results/tower.png"/>
</p>

### Window

In the `main.py` file
```python
test_window(True)
```

<p align="center">
  <img width="480" height="360" src="https://github.com/arthur960304/astar-path-planning/blob/main/results/window.png"/>
</p>
