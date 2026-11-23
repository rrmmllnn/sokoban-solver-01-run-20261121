// Sokoban solver for make.fun sokoban-solver-01-run-20261121 (Node 22, single file, no deps).
//
// Strategy (scoring is by PUSHES only: 1000 pts at the push-optimal count P*,
// 500*P*/pushes above it, 0 if unsolved):
//   1. Split the level into rooms (the generator's structure: a 1-wide hallway
//      plus 6x6 rooms off it; a box pushed into the hallway can never come back,
//      so each room is independent and the level optimum is the sum of the room
//      optima). Each room (3-4 boxes) is solved by breadth-first search over
//      PUSHES, which is push-optimal by construction, with two safe deadlock
//      prunings: dead squares (a box that can never reach a goal) and frozen
//      2x2 blocks. This mirrors the generator's own reference solver, so on
//      any level drawn by this generator it recovers P* exactly.
//   2. If a room's BFS exhausts its state budget or its time slice, fall back
//      to A* over pushes with an admissible min-assignment (Manhattan) heuristic
//      and the same deadlock pruning — near-optimal, still a valid solution.
//   3. Last resort: move-based A* with dead-square pruning, so an instance
//      still gets a (lower-scoring) solution instead of nothing.
// Time budget: the whole run has 60 s wall for 5 instances. A global deadline
// (start + 52 s) is enforced; time left is shared fairly across the instances
// and rooms still to solve, and every search checks the clock regularly.
// Deterministic: BFS ties break by square order, no randomness anywhere.
//
// Local run: mkdir -p out && MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

/** Wall-clock budget for the whole run (limit is 60 s; keep an 8 s margin). */
const START = Date.now();
const GLOBAL_DEADLINE = START + 52_000;
/** BFS state cap per room (the generator only keeps rooms its own 250k-state BFS solves). */
const BFS_STATES = 250_000;
/** Push-A* expansion cap per room (fallback only). */
const ASTAR_STATES = 400_000;

type Level = { w: number; h: number; wall: Uint8Array; goals: number[]; boxes: number[]; player: number };

const DIRS: [number, number, string, string][] = [
  [0, -1, 'u', 'U'],
  [0, 1, 'd', 'D'],
  [-1, 0, 'l', 'L'],
  [1, 0, 'r', 'R'],
];

function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x]!;
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  for (let y = 0; y < h; y++)
    for (let x = 0; x < w; x++)
      if (x === 0 || y === 0 || x === w - 1 || y === h - 1) wall[y * w + x] = 1;
  return { w, h, wall, goals, boxes: boxes.sort((a, b) => a - b), player };
}

/** Squares from which a box alone can be pushed onto some goal (pulled back from the goals). */
function liveSquares(level: Level): Uint8Array {
  const { w, wall, goals } = level;
  const live = new Uint8Array(wall.length);
  const queue = [...goals];
  for (const g of goals) live[g] = 1;
  let qi = 0;
  while (qi < queue.length) {
    const x = queue[qi++]!;
    for (const [dx, dy] of DIRS) {
      const d = dy * w + dx;
      const y = x - d;
      if (!wall[y] && !wall[y - d] && !live[y]) {
        live[y] = 1;
        queue.push(y);
      }
    }
  }
  return live;
}

/** Lowest square the player reaches from `from` with `occupied` in the way; marks `seen` with `stamp`. */
function reach(level: Level, occupied: Uint8Array, from: number, seen: Int32Array, stamp: number, stack: Int32Array): number {
  const { w, wall } = level;
  let low = from;
  let n = 0;
  stack[n++] = from;
  seen[from] = stamp;
  while (n > 0) {
    const p = stack[--n]!;
    if (p < low) low = p;
    const up = p - w, dn = p + w, lf = p - 1, rt = p + 1;
    if (wall[up] === 0 && occupied[up] === 0 && seen[up] !== stamp) { seen[up] = stamp; stack[n++] = up; }
    if (wall[dn] === 0 && occupied[dn] === 0 && seen[dn] !== stamp) { seen[dn] = stamp; stack[n++] = dn; }
    if (wall[lf] === 0 && occupied[lf] === 0 && seen[lf] !== stamp) { seen[lf] = stamp; stack[n++] = lf; }
    if (wall[rt] === 0 && occupied[rt] === 0 && seen[rt] !== stamp) { seen[rt] = stamp; stack[n++] = rt; }
  }
  return low;
}

/** Whether the box just pushed to `t` sits in a frozen 2x2 block (walls/boxes, a box off its goal). */
function frozenBlock(level: Level, occupied: Uint8Array, goal: Uint8Array, t: number): boolean {
  const { w, wall } = level;
  for (const a of [t, t - 1, t - w, t - w - 1]) {
    const b = a + 1, c = a + w, d = c + 1;
    if (!(wall[a] || occupied[a]) || !(wall[b] || occupied[b]) || !(wall[c] || occupied[c]) || !(wall[d] || occupied[d])) continue;
    if ((occupied[a] && !goal[a]) || (occupied[b] && !goal[b]) || (occupied[c] && !goal[c]) || (occupied[d] && !goal[d])) return true;
  }
  return false;
}

function keyOf(boxes: number[], low: number, size: number): number {
  let k = low;
  for (const b of boxes) k = k * size + b;
  return k;
}

/** Shortest walk from `from` to `to` around walls and boxes (lower-case LURD letters). */
function walkTo(level: Level, occupied: Uint8Array, from: number, to: number): string {
  const { w, wall } = level;
  if (from === to) return '';
  const prev = new Int32Array(wall.length).fill(-1);
  const how = new Array<string>(wall.length);
  const queue = [from];
  prev[from] = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi]!;
    if (p === to) break;
    for (const [dx, dy, letter] of DIRS) {
      const q = p + dy * w + dx;
      if (wall[q] || occupied[q] || prev[q] !== -1) continue;
      prev[q] = p;
      how[q] = letter;
      queue.push(q);
    }
  }
  if (prev[to] === -1) throw new Error('walkTo: push square out of reach');
  let s = '';
  for (let p = to; p !== from; p = prev[p]!) s = how[p]! + s;
  return s;
}

/** Rebuild the move string from a BFS/A* parent chain of push-states. */
function reconstruct(level: Level, statesBoxes: number[][], parent: number[], found: number): string {
  const path: number[] = [];
  for (let s = found; s > 0; s = parent[s]!) path.push(s);
  path.reverse();
  const occupied = new Uint8Array(level.wall.length);
  let moves = '';
  let player = level.player;
  let boxes = level.boxes;
  for (const s of path) {
    const nextBoxes = statesBoxes[s]!;
    const from = boxes.find((b) => !nextBoxes.includes(b))!;
    const to = nextBoxes.find((b) => !boxes.includes(b))!;
    const d = to - from;
    const dir = DIRS.find(([dx, dy]) => dy * level.w + dx === d)!;
    for (const b of boxes) occupied[b] = 1;
    moves += walkTo(level, occupied, player, from - d);
    for (const b of boxes) occupied[b] = 0;
    moves += dir[3];
    player = from;
    boxes = nextBoxes;
  }
  return moves;
}

/**
 * Push-optimal BFS over pushes for one (sub-)level: { pushes, moves } or null
 * when it gives up (state cap / deadline) or the level is unsolvable.
 * A state is the boxes' squares + the player's reachable region (named by its
 * lowest square); states expand in push order, so the first solved state met
 * is push-optimal. Dead squares and frozen 2x2 blocks are pruned (safe).
 */
function solvePushOptimal(level: Level, maxStates: number, deadline: number): { pushes: number; moves: string } | null {
  const { w, wall, goals } = level;
  const size = wall.length;
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const live = liveSquares(level);
  if (level.boxes.some((b) => !live[b])) return null;
  const occupied = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;
  const solved = (boxes: number[]): boolean => boxes.every((b) => goal[b]);
  const statesBoxes: number[][] = [level.boxes];
  const statesPlayer: number[] = [level.player];
  const parent: number[] = [-1];
  const index = new Map<number, number>();
  for (const b of level.boxes) occupied[b] = 1;
  index.set(keyOf(level.boxes, reach(level, occupied, level.player, seen, ++stamp, stack), size), 0);
  for (const b of level.boxes) occupied[b] = 0;
  let found = solved(level.boxes) ? 0 : -1;
  let expansions = 0;
  for (let head = 0; found < 0 && head < statesBoxes.length; head++) {
    if ((++expansions & 1023) === 0 && Date.now() > deadline) return null;
    const boxes = statesBoxes[head]!;
    for (const b of boxes) occupied[b] = 1;
    reach(level, occupied, statesPlayer[head]!, seen, ++stamp, stack);
    const here = stamp;
    for (let bi = 0; bi < boxes.length && found < 0; bi++) {
      const b = boxes[bi]!;
      for (const [dx, dy] of DIRS) {
        const d = dy * w + dx;
        const t = b + d;
        if (seen[b - d] !== here || wall[t] || occupied[t] || !live[t]) continue;
        occupied[b] = 0;
        occupied[t] = 1;
        const frozen = frozenBlock(level, occupied, goal, t);
        let key = -1;
        let next: number[] | null = null;
        if (!frozen) {
          next = boxes.slice();
          next[bi] = t;
          next.sort((x, y) => x - y);
          key = keyOf(next, reach(level, occupied, b, mark, ++stamp, stack), size);
        }
        occupied[t] = 0;
        occupied[b] = 1;
        if (frozen || index.has(key)) continue;
        if (statesBoxes.length >= maxStates) {
          for (const x of boxes) occupied[x] = 0;
          return null;
        }
        index.set(key, statesBoxes.length);
        statesBoxes.push(next!);
        statesPlayer.push(b);
        parent.push(head);
        if (solved(next!)) { found = statesBoxes.length - 1; break; }
      }
    }
    for (const x of boxes) occupied[x] = 0;
  }
  if (found < 0) return null;
  return { pushes: parentChainLength(parent, found), moves: reconstruct(level, statesBoxes, parent, found) };
}

function parentChainLength(parent: number[], found: number): number {
  let n = 0;
  for (let s = found; s > 0; s = parent[s]!) n++;
  return n;
}

/** Min-cost assignment of boxes to goals by Manhattan distance (admissible push heuristic). */
function matchingHeuristic(boxes: number[], goals: number[], w: number): number {
  const n = boxes.length;
  // Permute goals (n <= 4 in every room this generator draws; guard anyway).
  const idx = goals.map((_, i) => i);
  let best = Infinity;
  const permute = (k: number): void => {
    if (k === n) {
      let s = 0;
      for (let i = 0; i < n; i++) {
        const b = boxes[i]!, g = goals[idx[i]!]!;
        s += Math.abs((g % w) - (b % w)) + Math.abs(Math.floor(g / w) - Math.floor(b / w));
      }
      if (s < best) best = s;
      return;
    }
    for (let i = k; i < n; i++) {
      [idx[k], idx[i]] = [idx[i]!, idx[k]!];
      permute(k + 1);
      [idx[k], idx[i]] = [idx[i]!, idx[k]!];
    }
  };
  if (n <= 6) permute(0);
  else {
    // Greedy fallback for unexpectedly large rooms (still admissible? no — use
    // per-box min instead, which is admissible).
    best = 0;
    for (const b of boxes) {
      let m = Infinity;
      for (const g of goals) m = Math.min(m, Math.abs((g % w) - (b % w)) + Math.abs(Math.floor(g / w) - Math.floor(b / w)));
      best += m;
    }
  }
  return best;
}

/**
 * A* over pushes (fallback when BFS gives up): same state space and deadlock
 * pruning, f = pushes + min-assignment Manhattan heuristic. Usually optimal
 * on these small rooms; always returns a valid solution when it finds one.
 */
function solvePushAStar(level: Level, maxStates: number, deadline: number): { pushes: number; moves: string } | null {
  const { w, wall, goals } = level;
  const size = wall.length;
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const live = liveSquares(level);
  if (level.boxes.some((b) => !live[b])) return null;
  const occupied = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;
  const solved = (boxes: number[]): boolean => boxes.every((b) => goal[b]);
  const statesBoxes: number[][] = [level.boxes];
  const statesPlayer: number[] = [level.player];
  const parent: number[] = [-1];
  const gScore: number[] = [0];
  const best = new Map<number, number>();
  for (const b of level.boxes) occupied[b] = 1;
  const startKey = keyOf(level.boxes, reach(level, occupied, level.player, seen, ++stamp, stack), size);
  for (const b of level.boxes) occupied[b] = 0;
  best.set(startKey, 0);
  // Binary heap on [f, g, state].
  const heap: [number, number, number][] = [[matchingHeuristic(level.boxes, goals, w), 0, 0]];
  const less = (a: [number, number, number], b: [number, number, number]): boolean =>
    a[0] < b[0] || (a[0] === b[0] && (a[1] < b[1] || (a[1] === b[1] && a[2] < b[2])));
  const hpush = (e: [number, number, number]): void => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i]!, heap[p]!)) break;
      [heap[i], heap[p]] = [heap[p]!, heap[i]!];
      i = p;
    }
  };
  const hpop = (): [number, number, number] => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1]!, heap[c]!)) c++;
        if (!less(heap[c]!, heap[i]!)) break;
        [heap[i], heap[c]] = [heap[c]!, heap[i]!];
        i = c;
      }
    }
    return top;
  };
  let expansions = 0;
  while (heap.length > 0) {
    if ((++expansions & 1023) === 0 && Date.now() > deadline) return null;
    const [, g, s] = hpop();
    if (g !== gScore[s]) continue; // stale heap entry
    const boxes = statesBoxes[s]!;
    if (solved(boxes)) return { pushes: g, moves: reconstruct(level, statesBoxes, parent, s) };
    if (statesBoxes.length >= maxStates) return null;
    for (const b of boxes) occupied[b] = 1;
    reach(level, occupied, statesPlayer[s]!, seen, ++stamp, stack);
    const here = stamp;
    for (let bi = 0; bi < boxes.length; bi++) {
      const b = boxes[bi]!;
      for (const [dx, dy] of DIRS) {
        const d = dy * w + dx;
        const t = b + d;
        if (seen[b - d] !== here || wall[t] || occupied[t] || !live[t]) continue;
        occupied[b] = 0;
        occupied[t] = 1;
        const frozen = frozenBlock(level, occupied, goal, t);
        let key = -1;
        let next: number[] | null = null;
        if (!frozen) {
          next = boxes.slice();
          next[bi] = t;
          next.sort((x, y) => x - y);
          key = keyOf(next, reach(level, occupied, b, mark, ++stamp, stack), size);
        }
        occupied[t] = 0;
        occupied[b] = 1;
        if (frozen) continue;
        const seenG = best.get(key);
        if (seenG !== undefined && seenG <= g + 1) continue;
        best.set(key, g + 1);
        const ns = statesBoxes.length;
        statesBoxes.push(next!);
        statesPlayer.push(b);
        parent.push(s);
        gScore.push(g + 1);
        hpush([g + 1 + matchingHeuristic(next!, goals, w), g + 1, ns]);
      }
    }
    for (const x of boxes) occupied[x] = 0;
  }
  return null;
}

/**
 * Last-resort move-based A* (fewest moves, like the house baseline, but with
 * dead-square pruning so it does not wander into hopeless positions). Only
 * used when both push searches fail for a room.
 */
function solveMoveAStar(level: Level, maxStates: number, deadline: number): { pushes: number; moves: string } | null {
  const { w, wall, goals } = level;
  const goal = new Uint8Array(wall.length);
  for (const g of goals) goal[g] = 1;
  const live = liveSquares(level);
  if (level.boxes.some((b) => !live[b])) return null;
  const dist = (b: number): number => {
    let m = Infinity;
    for (const g of goals) m = Math.min(m, Math.abs((g % w) - (b % w)) + Math.abs(Math.floor(g / w) - Math.floor(b / w)));
    return m;
  };
  const est = (bs: number[]): number => bs.reduce((a, b) => a + dist(b), 0);
  const key = (p: number, bs: number[]): string => String.fromCharCode(p, ...bs);
  const sp: number[] = [level.player];
  const sb: number[][] = [level.boxes.slice()];
  const sg: number[] = [0];
  const parent: number[] = [-1];
  const step: string[] = [''];
  const best = new Map<string, number>([[key(level.player, level.boxes), 0]]);
  const heap: [number, number, number][] = [[est(level.boxes), 0, 0]];
  const less = (a: [number, number, number], b: [number, number, number]): boolean =>
    a[0] < b[0] || (a[0] === b[0] && (a[1] > b[1] || (a[1] === b[1] && a[2] < b[2])));
  const hpush = (e: [number, number, number]): void => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i]!, heap[p]!)) break;
      [heap[i], heap[p]] = [heap[p]!, heap[i]!];
      i = p;
    }
  };
  const hpop = (): [number, number, number] => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1]!, heap[c]!)) c++;
        if (!less(heap[c]!, heap[i]!)) break;
        [heap[i], heap[c]] = [heap[c]!, heap[i]!];
        i = c;
      }
    }
    return top;
  };
  let expanded = 0;
  while (heap.length > 0 && expanded < maxStates) {
    if ((++expanded & 1023) === 0 && Date.now() > deadline) return null;
    const [, g, s] = hpop();
    if (best.get(key(sp[s]!, sb[s]!)) !== s) continue;
    const bs = sb[s]!;
    if (bs.every((b) => goal[b])) {
      let moves = '';
      let pushes = 0;
      for (let t = s; t > 0; t = parent[t]!) {
        moves = step[t]! + moves;
        if (step[t]! === step[t]!.toUpperCase()) pushes++;
      }
      return { pushes, moves };
    }
    const p = sp[s]!;
    for (const [dx, dy, letter] of DIRS) {
      const d = dy * w + dx;
      const q = p + d;
      if (wall[q]) continue;
      let nb = bs;
      let move = letter;
      const at = bs.indexOf(q);
      if (at >= 0) {
        if (wall[q + d] || bs.includes(q + d) || !live[q + d]) continue;
        nb = bs.slice();
        nb[at] = q + d;
        nb.sort((a, b) => a - b);
        move = letter.toUpperCase();
      }
      const k = key(q, nb);
      const seenS = best.get(k);
      if (seenS !== undefined && sg[seenS]! <= g + 1) continue;
      const t = sp.length;
      sp.push(q);
      sb.push(nb);
      sg.push(g + 1);
      parent.push(s);
      step.push(move);
      best.set(k, t);
      hpush([g + 1 + est(nb), g + 1, t]);
    }
  }
  return null;
}

/** The level's rooms: cut the hallway's dead squares out of the floor; each remaining part holding boxes is a room. */
function roomsOf(level: Level): { squares: number[]; boxes: number[]; goals: number[]; done?: boolean }[] | null {
  const { w, wall, goals, boxes, player } = level;
  const live = liveSquares(level);
  const row = Math.floor(player / w);
  const cut = new Uint8Array(wall.length);
  for (let i = row * w; i < (row + 1) * w; i++) if (!wall[i] && !live[i]) cut[i] = 1;
  if (boxes.some((b) => cut[b])) return null;
  const part = new Int32Array(wall.length).fill(-1);
  const rooms: { squares: number[]; boxes: number[]; goals: number[]; done?: boolean }[] = [];
  for (let s = 0; s < wall.length; s++) {
    if (wall[s] || cut[s] || part[s] >= 0) continue;
    const squares = [s];
    part[s] = rooms.length;
    for (let i = 0; i < squares.length; i++) {
      for (const [dx, dy] of DIRS) {
        const q = squares[i]! + dy * w + dx;
        if (!wall[q] && !cut[q] && part[q] < 0) {
          part[q] = rooms.length;
          squares.push(q);
        }
      }
    }
    rooms.push({ squares, boxes: [], goals: [] });
  }
  for (const b of boxes) rooms[part[b]!]!.boxes.push(b);
  for (const g of goals) if (part[g]! >= 0) rooms[part[g]!]!.goals.push(g);
  if (goals.some((g) => part[g]! < 0) || rooms.some((r) => r.boxes.length !== r.goals.length)) return null;
  return rooms.filter((r) => r.boxes.length > 0);
}

function endOf(w: number, from: number, moves: string): number {
  let p = from;
  for (const ch of moves.toLowerCase()) p += ch === 'u' ? -w : ch === 'd' ? w : ch === 'l' ? -1 : 1;
  return p;
}

/** Solve one room with the fallback chain (BFS optimal -> push A* -> move A*). */
function solveRoom(sub: Level, deadline: number): { pushes: number; moves: string } | null {
  return (
    solvePushOptimal(sub, BFS_STATES, deadline) ??
    solvePushAStar(sub, ASTAR_STATES, deadline) ??
    solveMoveAStar(sub, 200_000, deadline)
  );
}

/** Solve a whole level room by room; falls back to whole-level search if it has no room structure. */
function solveLevel(level: Level, deadline: number): { pushes: number; moves: string } | null {
  const rooms = roomsOf(level);
  if (rooms === null) return solveRoom(level, deadline);
  let player = level.player;
  let pushes = 0;
  let moves = '';
  for (const room of rooms) {
    const wall = level.wall.slice();
    for (const other of rooms)
      if (other !== room) for (const b of other.done ? other.goals : other.boxes) wall[b] = 1;
    const sub: Level = {
      w: level.w,
      h: level.h,
      wall,
      goals: room.goals.slice().sort((a, b) => a - b),
      boxes: room.boxes.slice().sort((a, b) => a - b),
      player,
    };
    const solved = solveRoom(sub, deadline);
    if (solved === null) return null;
    pushes += solved.pushes;
    moves += solved.moves;
    player = endOf(level.w, player, solved.moves);
    room.done = true;
  }
  return { pushes, moves };
}

// ---- main: one run answers every instance in /input, writing /output/<id>/solution.txt ----
const ids = readdirSync(I).sort();
for (let k = 0; k < ids.length; k++) {
  const id = ids[k]!;
  const level = parseLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  // Fair share of the remaining time for the instances still to solve.
  const deadline = Math.min(GLOBAL_DEADLINE, Date.now() + Math.max(1000, Math.floor((GLOBAL_DEADLINE - Date.now()) / (ids.length - k))));
  let solved: { pushes: number; moves: string } | null = null;
  try {
    solved = solveLevel(level, deadline);
  } catch {
    solved = null;
  }
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${solved?.moves ?? ''}\n`);
}
