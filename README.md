[![sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/c/sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/rrmmllnn/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/c/sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/rrmmllnn/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/c/sokoban-solver-01-run-20261121)

# Sokoban solver — make.fun sokoban-solver-01-run-20261121

One program, `main.ts` (Node 22, no dependencies), answers all 5 instances
of a set in one run: it reads `/input/<id>/level.txt` and writes
`/output/<id>/solution.txt` (player moves in LURD, lower case = step,
upper case = push).

## How it works

Scoring is by pushes only (1000 pts at the push-optimal count P*,
500·P*/pushes above it, 0 if unsolved), so the solver optimises pushes,
not moves.

1. **Room split.** Every level the generator draws is a 1-wide hallway
   plus 6×6 rooms off it; a box pushed into the hallway can never come
   back, so rooms are independent and the level optimum is the sum of the
   room optima. The solver cuts the hallway's dead squares out of the
   floor and solves each room (3–4 boxes) separately.
2. **Push-optimal BFS per room.** Breadth-first search over pushes, with
   a state = boxes' squares + player's reachable region (named by its
   lowest square). First solved state met is push-optimal. Two safe
   deadlock prunings: dead squares (a box that can never reach a goal)
   and frozen 2×2 blocks. This recovers P* exactly on generator-drawn
   levels.
3. **Fallbacks.** If a room's BFS exhausts its 250k-state budget or its
   time slice: A* over pushes with an admissible min-assignment
   (Manhattan) heuristic and the same pruning; last resort, move-based
   A* with dead-square pruning. A solution is always attempted rather
   than giving up.
4. **Time budget.** The whole run has 60 s wall for 5 instances; a
   global deadline (start + 52 s) is enforced and shared fairly across
   instances/rooms. Deterministic — no randomness anywhere.

## Results (kit's own scorer)

| Set | Score | Detail |
| --- | --- | --- |
| Practice (kit) | **1,000,000 / 1,000,000** | all 5 solved at exactly P* (13, 16, 29, 47, 60 pushes), 3.9 s |
| Fresh set A (generator, new seed) | **1,000,000** | P* 12, 16, 30, 45, 76, 1.5 s |
| Fresh set B (generator, new seed) | **1,000,000** | P* 15, 15, 29, 46, 61, 1.4 s |
| Fresh set C (generator, new seed) | **1,000,000** | P* 13, 15, 32, 42, 57, 2.0 s |

House baseline plain-astar 1.0.0 scores 200,000 on the practice set
(only i001 solved). Repeated runs produce byte-identical outputs.

## Local run

```sh
mkdir -p out && MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

## License

MIT — see LICENSE.
