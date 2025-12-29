#!/usr/bin/env python3
"""
map_semantic_extractor (ROS 2)

Suscribe a un OccupancyGrid (p.ej. /map de slam_toolbox), ejecuta el pipeline
(Voronoi/skeleton → grafo LOS + frontiers) y publica dos JSON en topics distintos:

- /map_semantic/graph_json     (std_msgs/String): grafo navegable LOS en JSON
- /map_semantic/frontiers_json (std_msgs/String): frontiers en JSON

Diseño:
- Callback mínimo: solo llamadas a funciones puras + publish.
- Parámetros mínimos en el nodo: topics (opcional). El resto son constantes en este archivo.
"""

import json
import math
from typing import Any, Dict, List, Optional, Set, Tuple

import cv2
import networkx as nx
import numpy as np
from skimage.morphology import skeletonize

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String


# ============================================================
# CONSTANTES (ajusta aquí, no en parámetros del nodo)
# ============================================================

MAP_TOPIC = "/map"
GRAPH_TOPIC = "/map_semantic/graph_json"
FRONTIERS_TOPIC = "/map_semantic/frontiers_json"

FRAME_ID_FALLBACK = "map"

# Conversión OccupancyGrid -> gris (compatibilidad con tu lógica de thresholds)
# Convención:
#   free (0)      -> 255
#   occ (100)     -> 0
#   unknown (-1)  -> 127
UNKNOWN_GRAY = 127
OCC_MAX = 100

# Segmentación estilo PGM (si mantienes thresholds)
FREE_THRESHOLD = 250
OCC_THRESHOLD = 50
MID_IS_UNKNOWN = True
UNKNOWN_IS_OBSTACLE = False  

# Limpieza máscara (sobre free_mask)
OPEN_ITERS = 1
CLOSE_ITERS = 1
CLEAN_KERNEL = 3
KEEP_LARGEST = True
USE_KNOWN_ONLY_FOR_GRAPH = True

# Skeleton / grafo
SKELETON_MIN_CLEARANCE_CELLS = 0.0
JUNCTION_DEGREE = 3
INCLUDE_ENDPOINTS = True
LOS_MIN_CLEARANCE_M = 0.20
SNAP_RADIUS_CELLS = 3

# Frontiers
FRONTIER_CONNECTIVITY8 = True
FRONTIER_MIN_SIZE_CELLS = 20
TOP_K_FRONTIERS = 15


# ============================================================
# CONVERSIÓN OccupancyGrid -> np.ndarray
# ============================================================

def occupancygrid_to_numpy(msg: OccupancyGrid) -> np.ndarray:
    """
    Convierte msg.data (int8) a una matriz (H, W) de int16 para operar con máscaras.

    msg.data:
      -1 = unknown
       0 = free
     100 = occupied (o valores intermedios, según origen)
    """
    h = int(msg.info.height)
    w = int(msg.info.width)
    data = np.array(msg.data, dtype=np.int16).reshape((h, w))
    return data


def occupancygrid_to_gray(occ: np.ndarray) -> np.ndarray:
    """
    Normaliza OccupancyGrid a una imagen en gris uint8 (H, W) compatible con tu pipeline PGM:
      free (0) -> 255
      occ (100) -> 0
      unknown (-1) -> UNKNOWN_GRAY
    """
    gray = np.empty_like(occ, dtype=np.uint8)

    unknown = occ < 0
    known = ~unknown

    gray[unknown] = np.uint8(UNKNOWN_GRAY)

    # Escala lineal 0..100 → 255..0
    v = np.clip(occ[known], 0, OCC_MAX).astype(np.float32)
    gray_known = (255.0 * (1.0 - (v / float(OCC_MAX)))).astype(np.uint8)
    gray[known] = gray_known

    return gray


# ============================================================
# MÁSCARAS + LIMPIEZA
# ============================================================

def pgm_to_masks(
    img: np.ndarray,
    *,
    free_threshold: int,
    occ_threshold: int,
    mid_is_unknown: bool,
    unknown_is_obstacle: bool,
) -> Dict[str, np.ndarray]:
    """
    Segmenta un mapa en gris en máscaras booleanas: free / obstacle / unknown.

    - free:       img >= free_threshold
    - obstacle:   img <= occ_threshold
    - unknown:    resto (si mid_is_unknown=True)
    """
    free = img >= free_threshold
    obstacle = img <= occ_threshold
    unknown = (~(free | obstacle)) if mid_is_unknown else np.zeros_like(free, bool)

    if unknown_is_obstacle:
        obstacle = obstacle | unknown
        free = ~obstacle

    return {"free": free, "obstacle": obstacle, "unknown": unknown}


def clean_mask(mask: np.ndarray, *, open_iters: int, close_iters: int, kernel: int) -> np.ndarray:
    """
    Aplica morfología (open + close) para eliminar ruido en máscara binaria.
    """
    k = np.ones((kernel, kernel), np.uint8)
    x = (mask.astype(np.uint8) * 255)
    if open_iters > 0:
        x = cv2.morphologyEx(x, cv2.MORPH_OPEN, k, iterations=open_iters)
    if close_iters > 0:
        x = cv2.morphologyEx(x, cv2.MORPH_CLOSE, k, iterations=close_iters)
    return x > 0


def keep_largest_component(mask: np.ndarray) -> np.ndarray:
    """
    Conserva solo el componente conexo más grande (reduce islas y artefactos).
    """
    x = mask.astype(np.uint8)
    n, labels = cv2.connectedComponents(x)
    if n <= 1:
        return mask
    counts = np.bincount(labels.reshape(-1))
    counts[0] = 0
    keep = counts.argmax()
    return labels == keep


def distance_transform(free_mask: np.ndarray) -> np.ndarray:
    """
    Distance Transform L2 (en celdas) sobre el espacio libre.
    """
    src = free_mask.astype(np.uint8) * 255
    return cv2.distanceTransform(src, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)


def skeleton_free(free_mask: np.ndarray) -> np.ndarray:
    """
    Skeleton del espacio libre (bool).
    """
    return skeletonize(free_mask).astype(bool, copy=False)


def filter_skeleton_by_clearance(skel: np.ndarray, dist_cells: np.ndarray, *, min_clearance_cells: float) -> np.ndarray:
    """
    Filtra el skeleton eliminando puntos con clearance (distancia a obstáculo) menor que el umbral.
    """
    return skel & (dist_cells >= min_clearance_cells)


# ============================================================
# GRAFO (skeleton → topología → LOS)
# ============================================================

def skeleton_degree(skel: np.ndarray) -> np.ndarray:
    """
    Calcula el grado 8-conectado de cada celda del skeleton.
    """
    k = np.array([[1, 1, 1],
                  [1, 0, 1],
                  [1, 1, 1]], dtype=np.uint8)
    return cv2.filter2D(skel.astype(np.uint8), -1, k, borderType=cv2.BORDER_CONSTANT)


def cell_to_xy(cell: Tuple[int, int], resolution_m: float, origin_xy_m: Tuple[float, float]) -> Tuple[float, float]:
    """
    Convierte celda (r,c) a coordenadas métricas (x,y).
    """
    r, c = cell
    x0, y0 = origin_xy_m
    return (x0 + c * resolution_m, y0 + r * resolution_m)


def neighbors8(cell: Tuple[int, int], skeleton: np.ndarray) -> List[Tuple[int, int]]:
    """
    Vecinos 8-conectados dentro del skeleton.
    """
    h, w = skeleton.shape
    r, c = cell
    out: List[Tuple[int, int]] = []
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            rr, cc = r + dr, c + dc
            if 0 <= rr < h and 0 <= cc < w and skeleton[rr, cc]:
                out.append((rr, cc))
    return out


def skeleton_node_cells(
    skeleton: np.ndarray,
    deg: np.ndarray,
    *,
    junction_degree: int,
    include_endpoints: bool,
) -> List[Tuple[int, int]]:
    """
    Selecciona celdas que serán nodos:
    - endpoints (grado=1) si include_endpoints=True
    - junctions (grado>=junction_degree)
    """
    ys, xs = np.where(skeleton)
    out: List[Tuple[int, int]] = []
    for r, c in zip(ys.tolist(), xs.tolist()):
        d = int(deg[r, c])
        if (include_endpoints and d == 1) or (d >= junction_degree):
            out.append((r, c))
    return out


def node_id_map(node_cells: List[Tuple[int, int]], *, prefix: str = "n") -> Dict[Tuple[int, int], str]:
    """
    Asigna IDs estables a las celdas de nodo.
    """
    return {cell: f"{prefix}{i}" for i, cell in enumerate(node_cells)}


def init_graph(frame_id: str, resolution_m: float, origin_xy_m: Tuple[float, float]) -> nx.Graph:
    """
    Grafo base con metadatos de frame/resolución/origen.
    """
    return nx.Graph(frame_id=frame_id, resolution_m=resolution_m, origin_xy_m=origin_xy_m)


def add_nodes_from_cells(
    g: nx.Graph,
    node_id: Dict[Tuple[int, int], str],
    deg: np.ndarray,
    *,
    resolution_m: float,
    origin_xy_m: Tuple[float, float],
    junction_degree: int,
) -> None:
    """
    Inserta nodos al grafo con atributos (cell, xy_m, type, labels).
    """
    for cell, nid in node_id.items():
        d = int(deg[cell[0], cell[1]])
        g.add_node(
            nid,
            cell=cell,
            xy_m=cell_to_xy(cell, resolution_m, origin_xy_m),
            type=("junction" if d >= junction_degree else "endpoint"),
            labels={},
        )


def trace_edge_path(
    start_node: Tuple[int, int],
    first_step: Tuple[int, int],
    skeleton: np.ndarray,
    node_set: Set[Tuple[int, int]],
) -> List[Tuple[int, int]]:
    """
    Traza una arista siguiendo el skeleton desde un nodo hasta encontrar otro nodo.
    """
    path = [start_node]
    prev, cur = start_node, first_step
    while True:
        path.append(cur)
        if cur in node_set:
            return path
        nxt = None
        for cand in neighbors8(cur, skeleton):
            if cand != prev:
                nxt = cand
                break
        if nxt is None:
            return path
        prev, cur = cur, nxt


def add_edges_from_skeleton(g: nx.Graph, node_id: Dict[Tuple[int, int], str], skeleton: np.ndarray) -> None:
    """
    Inserta aristas topológicas entre nodos siguiendo el skeleton.
    """
    node_set = set(node_id.keys())
    visited: Set[Tuple[Tuple[int, int], Tuple[int, int]]] = set()

    for u_cell, u in node_id.items():
        for v_cell in neighbors8(u_cell, skeleton):
            if (u_cell, v_cell) in visited:
                continue
            visited.add((u_cell, v_cell))

            path = trace_edge_path(u_cell, v_cell, skeleton, node_set)
            end_cell = path[-1]
            if end_cell not in node_set:
                continue

            v = node_id[end_cell]
            if u == v or g.has_edge(u, v):
                continue

            g.add_edge(u, v, path_cells=path, labels={}, metrics={}, enabled=True)


def skeleton_to_graph(
    skel: np.ndarray,
    *,
    resolution_m: float,
    origin_xy_m: Tuple[float, float],
    frame_id: str,
    junction_degree: int,
    include_endpoints: bool,
) -> nx.Graph:
    """
    Convierte skeleton a grafo topológico (nodos en junctions/endpoints + aristas por trazado).
    """
    skeleton = skel.astype(bool)
    deg = skeleton_degree(skeleton)

    nodes = skeleton_node_cells(
        skeleton=skeleton,
        deg=deg,
        junction_degree=junction_degree,
        include_endpoints=include_endpoints,
    )
    nid = node_id_map(nodes)

    g = init_graph(frame_id=frame_id, resolution_m=resolution_m, origin_xy_m=origin_xy_m)
    add_nodes_from_cells(
        g=g,
        node_id=nid,
        deg=deg,
        resolution_m=resolution_m,
        origin_xy_m=origin_xy_m,
        junction_degree=junction_degree,
    )
    add_edges_from_skeleton(g=g, node_id=nid, skeleton=skeleton)
    return g


def bresenham(a: Tuple[int, int], b: Tuple[int, int]):
    """
    Iterador de celdas en el segmento discreto a->b.
    """
    r0, c0 = a
    r1, c1 = b
    dr, dc = abs(r1 - r0), abs(c1 - c0)
    sr, sc = (1 if r0 < r1 else -1), (1 if c0 < c1 else -1)
    err = dr - dc
    r, c = r0, c0
    while True:
        yield r, c
        if (r, c) == (r1, c1):
            break
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc


def segment_ok(
    free_mask: np.ndarray,
    a: Tuple[int, int],
    b: Tuple[int, int],
    *,
    dist_cells: Optional[np.ndarray],
    min_clearance_cells: float,
) -> bool:
    """
    Valida que el segmento a->b atraviesa solo espacio libre y (opcionalmente) cumple clearance.
    """
    for r, c in bresenham(a, b):
        if not free_mask[r, c]:
            return False
        if dist_cells is not None and min_clearance_cells > 0 and dist_cells[r, c] < min_clearance_cells:
            return False
    return True


def path_to_los_anchors(
    free_mask: np.ndarray,
    path_cells: List[Tuple[int, int]],
    *,
    dist_cells: Optional[np.ndarray],
    min_clearance_cells: float,
) -> List[Tuple[int, int]]:
    """
    Reduce una polilínea (path_cells) a anclas con visión directa (LOS) respetando clearance.
    """
    if len(path_cells) < 2:
        return path_cells

    anchors = [path_cells[0]]
    i, n = 0, len(path_cells)

    while i < n - 1:
        best = i + 1
        j = i + 1
        while j < n and segment_ok(free_mask, path_cells[i], path_cells[j], dist_cells=dist_cells, min_clearance_cells=min_clearance_cells):
            best = j
            j += 1
        anchors.append(path_cells[best])
        i = best

    return anchors


def edges_to_los_chains(
    g: nx.Graph,
    free_mask: np.ndarray,
    dist_cells: Optional[np.ndarray],
    *,
    min_clearance_m: float,
) -> nx.Graph:
    """
    Convierte aristas topológicas en cadenas de segmentos rectos (LOS) con geometría y métricas.
    """
    res = float(g.graph.get("resolution_m", 1.0))
    origin = tuple(g.graph.get("origin_xy_m", (0.0, 0.0)))
    frame_id = g.graph.get("frame_id", FRAME_ID_FALLBACK)
    min_clearance_cells = (min_clearance_m / res) if min_clearance_m > 0 else 0.0

    def xy(cell: Tuple[int, int]) -> Tuple[float, float]:
        r, c = cell
        x0, y0 = origin
        return (x0 + c * res, y0 + r * res)

    out = nx.Graph(frame_id=frame_id, resolution_m=res, origin_xy_m=origin)
    for nid, nd in g.nodes(data=True):
        out.add_node(nid, **nd)

    turn_idx = 0
    def new_turn_id():
        nonlocal turn_idx
        tid = f"t{turn_idx}"
        turn_idx += 1
        return tid

    for u, v, ed in g.edges(data=True):
        if ed.get("enabled") is False:
            continue
        path = ed.get("path_cells")
        if not path or len(path) < 2:
            continue

        anchors = path_to_los_anchors(free_mask, path, dist_cells=dist_cells, min_clearance_cells=min_clearance_cells)
        chain = [u]
        for cell in anchors[1:-1]:
            tid = new_turn_id()
            out.add_node(tid, cell=cell, xy_m=xy(cell), type="turn", labels={})
            chain.append(tid)
        chain.append(v)

        for a, b in zip(chain[:-1], chain[1:]):
            ax, ay = out.nodes[a]["xy_m"]
            bx, by = out.nodes[b]["xy_m"]
            out.add_edge(
                a, b,
                enabled=True,
                is_straight=True,
                geometry={"segment_xy_m": [(ax, ay), (bx, by)]},
                labels={},
                metrics={"length_m": float(math.hypot(bx - ax, by - ay))}
            )

    return out


def snap_merge_nodes(g: nx.Graph, *, snap_radius_cells: int) -> nx.Graph:
    """
    Fusiona nodos cercanos por buckets (reduce clusters y estabiliza el grafo).
    """
    out = nx.Graph(**g.graph)

    buckets: Dict[Tuple[int, int], List[str]] = {}
    for nid, nd in g.nodes(data=True):
        r, c = nd["cell"]
        key = (int(r // snap_radius_cells), int(c // snap_radius_cells))
        buckets.setdefault(key, []).append(nid)

    representative: Dict[str, str] = {}
    for key, nodes in buckets.items():
        if len(nodes) == 1:
            nid = nodes[0]
            out.add_node(nid, **g.nodes[nid])
            representative[nid] = nid
            continue

        rs = [g.nodes[n]["cell"][0] for n in nodes]
        cs = [g.nodes[n]["cell"][1] for n in nodes]
        r0 = int(round(float(np.mean(rs))))
        c0 = int(round(float(np.mean(cs))))
        rep = f"m{key[0]}_{key[1]}"

        base = dict(g.nodes[nodes[0]])
        base["cell"] = (r0, c0)
        out.add_node(rep, **base)

        for n in nodes:
            representative[n] = rep

    for u, v, ed in g.edges(data=True):
        uu, vv = representative[u], representative[v]
        if uu == vv:
            continue
        if out.has_edge(uu, vv):
            continue
        out.add_edge(uu, vv, **ed)

    return out


def graph_to_llm_json(g: nx.Graph, *, include_disabled_edges: bool) -> str:
    """
    Serializa el grafo LOS a JSON (nodos/aristas) para consumo por LLM/planificador.
    """
    payload = {
        "frame": g.graph.get("frame_id", FRAME_ID_FALLBACK),
        "resolution_m": g.graph.get("resolution_m", 1.0),
        "origin_xy_m": list(g.graph.get("origin_xy_m", (0.0, 0.0))),
        "nodes": [],
        "edges": [],
    }

    for nid, nd in g.nodes(data=True):
        node = {"id": nid, **nd}
        r, c = node["cell"]
        node["cell"] = [int(r), int(c)]
        x, y = node["xy_m"]
        node["xy_m"] = [float(x), float(y)]
        payload["nodes"].append(node)

    for u, v, ed in g.edges(data=True):
        if not include_disabled_edges and ed.get("enabled") is False:
            continue
        payload["edges"].append({"from": u, "to": v, **ed})

    return json.dumps(payload, ensure_ascii=False, indent=2)


# ============================================================
# FRONTIERS (mask → clusters → ranking simple)
# ============================================================

def frontier_mask(free: np.ndarray, unknown: np.ndarray, *, connectivity8: bool) -> np.ndarray:
    """
    Frontier = celda libre adyacente a unknown (8-conectividad por defecto).
    """
    k = np.ones((3, 3), np.uint8) if connectivity8 else np.array([[0, 1, 0],
                                                                  [1, 1, 1],
                                                                  [0, 1, 0]], np.uint8)
    unk_nb = cv2.dilate(unknown.astype(np.uint8), k, iterations=1) > 0
    return free & unk_nb


def cluster_frontiers(frontier: np.ndarray, *, min_size_cells: int) -> List[Dict[str, Any]]:
    """
    Agrupa frontier cells en regiones conexas y devuelve centroides + bbox.
    """
    x = (frontier.astype(np.uint8) * 255)
    n, _, stats, centroids = cv2.connectedComponentsWithStats(x, connectivity=8)

    out: List[Dict[str, Any]] = []
    for i in range(1, n):
        area = int(stats[i, cv2.CC_STAT_AREA])
        if area < min_size_cells:
            continue
        cx, cy = centroids[i]  # (x=col, y=row)
        out.append({
            "id": f"f{i}",
            "size_cells": area,
            "centroid_rc": (int(round(cy)), int(round(cx))),
            "bbox_rcwh": (
                int(stats[i, cv2.CC_STAT_TOP]),
                int(stats[i, cv2.CC_STAT_LEFT]),
                int(stats[i, cv2.CC_STAT_HEIGHT]),
                int(stats[i, cv2.CC_STAT_WIDTH]),
            ),
        })
    return out


def rank_frontiers_by_size(frontiers: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """
    Ranking mínimo: mayor tamaño primero.
    """
    return sorted(frontiers, key=lambda f: f["size_cells"], reverse=True)


def frontiers_to_llm_json(
    frontiers_scored: List[Dict[str, Any]],
    *,
    frame_id: str,
    resolution_m: float,
    origin_xy_m: Tuple[float, float],
    top_k: int,
) -> str:
    """
    Serializa frontiers a JSON independiente del grafo.
    """
    payload = {
        "frame": frame_id,
        "resolution_m": float(resolution_m),
        "origin_xy_m": [float(origin_xy_m[0]), float(origin_xy_m[1])],
        "frontiers": [],
    }

    for f in frontiers_scored[:top_k]:
        item = dict(f)

        if isinstance(item.get("centroid_rc"), tuple):
            r, c = item["centroid_rc"]
            item["centroid_rc"] = [int(r), int(c)]

        if isinstance(item.get("bbox_rcwh"), tuple):
            t, l, h, w = item["bbox_rcwh"]
            item["bbox_rcwh"] = [int(t), int(l), int(h), int(w)]

        payload["frontiers"].append(item)

    return json.dumps(payload, ensure_ascii=False, indent=2)


# ============================================================
# PIPELINES (separados): grafo y frontiers
# ============================================================

def build_graph_json_from_occupancy(
    occ: np.ndarray,
    *,
    resolution_m: float,
    origin_xy_m: Tuple[float, float],
    frame_id: str,
) -> str:
    """
    Pipeline grafo: OccupancyGrid (np) → gris → máscaras → limpieza → skeleton → grafo topo → LOS → merge → JSON.
    """
    img = occupancygrid_to_gray(occ)
    masks = pgm_to_masks(
        img,
        free_threshold=FREE_THRESHOLD,
        occ_threshold=OCC_THRESHOLD,
        mid_is_unknown=MID_IS_UNKNOWN,
        unknown_is_obstacle=UNKNOWN_IS_OBSTACLE,
    )

    free_mask = masks["free"]
    if USE_KNOWN_ONLY_FOR_GRAPH:
        free_mask = free_mask & (~masks["unknown"])

    free_mask = clean_mask(free_mask, open_iters=OPEN_ITERS, close_iters=CLOSE_ITERS, kernel=CLEAN_KERNEL)
    if KEEP_LARGEST:
        free_mask = keep_largest_component(free_mask)

    dist = distance_transform(free_mask)
    skel = skeleton_free(free_mask)
    if SKELETON_MIN_CLEARANCE_CELLS > 0:
        skel = filter_skeleton_by_clearance(skel, dist, min_clearance_cells=SKELETON_MIN_CLEARANCE_CELLS)

    g_topo = skeleton_to_graph(
        skel,
        resolution_m=resolution_m,
        origin_xy_m=origin_xy_m,
        frame_id=frame_id,
        junction_degree=JUNCTION_DEGREE,
        include_endpoints=INCLUDE_ENDPOINTS,
    )

    g_los = edges_to_los_chains(g_topo, free_mask, dist, min_clearance_m=LOS_MIN_CLEARANCE_M)
    g_los = snap_merge_nodes(g_los, snap_radius_cells=SNAP_RADIUS_CELLS)

    return graph_to_llm_json(g_los, include_disabled_edges=False)


def build_frontiers_json_from_occupancy(
    occ: np.ndarray,
    *,
    resolution_m: float,
    origin_xy_m: Tuple[float, float],
    frame_id: str,
) -> str:
    """
    Pipeline frontiers: OccupancyGrid (np) → gris → máscaras → frontier_mask → clusters → rank → JSON.
    """
    img = occupancygrid_to_gray(occ)
    masks = pgm_to_masks(
        img,
        free_threshold=FREE_THRESHOLD,
        occ_threshold=OCC_THRESHOLD,
        mid_is_unknown=MID_IS_UNKNOWN,
        unknown_is_obstacle=UNKNOWN_IS_OBSTACLE,
    )

    fm = frontier_mask(masks["free"], masks["unknown"], connectivity8=FRONTIER_CONNECTIVITY8)
    clusters = cluster_frontiers(fm, min_size_cells=FRONTIER_MIN_SIZE_CELLS)
    ranked = rank_frontiers_by_size(clusters)

    return frontiers_to_llm_json(
        ranked,
        frame_id=frame_id,
        resolution_m=resolution_m,
        origin_xy_m=origin_xy_m,
        top_k=TOP_K_FRONTIERS,
    )


# ============================================================
# ROS 2 NODE
# ============================================================

class MapSemanticExtractor(Node):
    """
    Nodo ROS2:
    - subscribe: OccupancyGrid
    - publish: graph_json y frontiers_json (std_msgs/String)
    """

    def __init__(self):
        super().__init__("map_semantic_extractor")

        self.declare_parameter("map_topic", MAP_TOPIC)
        self.declare_parameter("graph_topic", GRAPH_TOPIC)
        self.declare_parameter("frontiers_topic", FRONTIERS_TOPIC)

        self._map_topic = self.get_parameter("map_topic").value
        self._graph_topic = self.get_parameter("graph_topic").value
        self._frontiers_topic = self.get_parameter("frontiers_topic").value

        self._pub_graph = self.create_publisher(String, self._graph_topic, 10)
        self._pub_frontiers = self.create_publisher(String, self._frontiers_topic, 10)

        self._sub = self.create_subscription(OccupancyGrid, self._map_topic, self._on_map, 10)

        self.get_logger().info(f"Subscribed: {self._map_topic}")
        self.get_logger().info(f"Publishing graph: {self._graph_topic}")
        self.get_logger().info(f"Publishing frontiers: {self._frontiers_topic}")

    def _on_map(self, msg: OccupancyGrid) -> None:
        """
        Callback mínimo: convertir → pipelines → publish.
        """
        occ = occupancygrid_to_numpy(msg)
        res = float(msg.info.resolution)

        origin_xy_m = (
            float(msg.info.origin.position.x),
            float(msg.info.origin.position.y),
        )

        frame_id = msg.header.frame_id or FRAME_ID_FALLBACK

        graph_json, frontiers_json = self._process_map(occ, res, origin_xy_m, frame_id)
        self._publish_json(graph_json, frontiers_json)

    def _process_map(
        self,
        occ: np.ndarray,
        resolution_m: float,
        origin_xy_m: Tuple[float, float],
        frame_id: str,
    ) -> Tuple[str, str]:
        """
        Encapsula el trabajo pesado fuera del callback.
        """
        graph_json = build_graph_json_from_occupancy(
            occ,
            resolution_m=resolution_m,
            origin_xy_m=origin_xy_m,
            frame_id=frame_id,
        )

        frontiers_json = build_frontiers_json_from_occupancy(
            occ,
            resolution_m=resolution_m,
            origin_xy_m=origin_xy_m,
            frame_id=frame_id,
        )

        return graph_json, frontiers_json

    def _publish_json(self, graph_json: str, frontiers_json: str) -> None:
        """
        Publica ambos JSON en topics separados.
        """
        m1 = String()
        m1.data = graph_json
        self._pub_graph.publish(m1)

        m2 = String()
        m2.data = frontiers_json
        self._pub_frontiers.publish(m2)


def main(args=None) -> None:
    """
    Entry point ROS2.
    """
    rclpy.init(args=args)
    node = MapSemanticExtractor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
