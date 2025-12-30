#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import numpy as np
import cv2
import networkx as nx
from typing import Dict, List, Tuple, Set, Optional, Any
from skimage.morphology import skeletonize
import math


class MapSemanticExtractorNode(Node):
    def __init__(self):
        super().__init__('map_semantic_extractor_node')
        
        # Parámetros
        self.declare_parameter('free_threshold', 65)
        self.declare_parameter('occ_threshold', 25)
        self.declare_parameter('min_clearance_m', 0.3)
        self.declare_parameter('snap_radius_cells', 3)
        self.declare_parameter('junction_degree', 3)
        self.declare_parameter('morph_open_iters', 0)   # Nuevo: limpieza open
        self.declare_parameter('morph_close_iters', 1)  # Nuevo: limpieza close
        self.declare_parameter('morph_kernel_size', 3)  # Nuevo: tamaño kernel
        
        # Suscriptor al mapa
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # Publishers para markers
        self.marker_pub = self.create_publisher(MarkerArray, '/topological_graph_markers', 10)
        
        self.graph = None
        self.last_map_data = None
        
        self.get_logger().info('Map Semantic Extractor Node iniciado')

    def occupancy_grid_to_masks(
        self,
        grid: OccupancyGrid,
        free_threshold: int = 65,
        occ_threshold: int = 25,
    ) -> Dict[str, np.ndarray]:
        """
        Convierte OccupancyGrid a máscaras numpy.
        OccupancyGrid: valores 0-100 (probabilidad de ocupación), -1 = desconocido
        """
        width = grid.info.width
        height = grid.info.height
        
        # Convertir datos a array 2D
        # IMPORTANTE: OccupancyGrid usa row-major order, origen en esquina inferior izquierda
        data = np.array(grid.data, dtype=np.int8).reshape((height, width))
        
        # Voltear verticalmente porque OccupancyGrid tiene origen abajo-izquierda
        # y OpenCV/numpy usan origen arriba-izquierda
        data = np.flipud(data)
        
        # Crear máscaras
        unknown = (data == -1)
        free = (data >= 0) & (data <= free_threshold)  # Baja probabilidad = libre
        obstacle = (data > occ_threshold) & (data <= 100)  # Alta probabilidad = ocupado
        
        self.get_logger().info(
            f'Máscaras creadas - Free: {free.sum()}, Obstacle: {obstacle.sum()}, Unknown: {unknown.sum()}'
        )
        
        return {
            "free": free,
            "obstacle": obstacle,
            "unknown": unknown,
            "resolution_m": grid.info.resolution,
            "origin_xy_m": (grid.info.origin.position.x, grid.info.origin.position.y),
            "frame_id": grid.header.frame_id,
            "height": height,  # Guardamos altura para conversión correcta
        }

    def map_callback(self, msg: OccupancyGrid):
        """Callback cuando llega un nuevo mapa"""
        try:
            self.get_logger().info('Procesando nuevo mapa...')
            
            # Convertir a máscaras
            masks = self.occupancy_grid_to_masks(
                msg,
                self.get_parameter('free_threshold').value,
                self.get_parameter('occ_threshold').value
            )
            
            free_mask = masks["free"]
            obstacle_mask = masks["obstacle"]
            unknown_mask = masks["unknown"]
            resolution_m = masks["resolution_m"]
            origin_xy_m = masks["origin_xy_m"]
            frame_id = masks["frame_id"]
            height = masks["height"]
            
            # Limpiar máscara libre (ahora configurable)
            free_clean = self.clean_mask(
                free_mask,
                open_iters=self.get_parameter('morph_open_iters').value,
                close_iters=self.get_parameter('morph_close_iters').value,
                kernel=self.get_parameter('morph_kernel_size').value
            )
            free_clean = self.keep_largest_component(free_clean)
            
            # Calcular distancia de obstáculos
            dist_cells = self.distance_transform(free_clean)
            
            # Skeletonizar
            skel = self.skeleton_free(free_clean)
            
            # Filtrar por clearance
            min_clearance_m = self.get_parameter('min_clearance_m').value
            min_clearance_cells = min_clearance_m / resolution_m
            skel_filtered = self.filter_skeleton_by_clearance(skel, dist_cells, min_clearance_cells)
            
            self.get_logger().info(f'Skeleton pixels: {skel_filtered.sum()}')
            
            # Convertir a grafo
            graph = self.skeleton_to_graph(
                skel_filtered,
                resolution_m=resolution_m,
                origin_xy_m=origin_xy_m,
                frame_id=frame_id,
                height=height,
                junction_degree=self.get_parameter('junction_degree').value,
                include_endpoints=True
            )
            
            # Simplificar a líneas rectas (LOS)
            graph = self.edges_to_los_chains(graph, free_clean, dist_cells, min_clearance_m)
            
            # Fusionar nodos cercanos
            snap_radius = self.get_parameter('snap_radius_cells').value
            graph = self.snap_merge_nodes(graph, snap_radius_cells=snap_radius)
            
            self.graph = graph
            self.last_map_data = masks
            
            self.get_logger().info(
                f'Grafo generado: {graph.number_of_nodes()} nodos, {graph.number_of_edges()} aristas'
            )
            
            # Publicar markers
            self.publish_markers(graph, frame_id)
            
        except Exception as e:
            self.get_logger().error(f'Error procesando mapa: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def publish_markers(self, graph: nx.Graph, frame_id: str):
        """Publica markers de visualización para RViz2"""
        marker_array = MarkerArray()
        
        # Marker para nodos (esferas)
        node_marker = Marker()
        node_marker.header.frame_id = frame_id
        node_marker.header.stamp = self.get_clock().now().to_msg()
        node_marker.ns = "nodes"
        node_marker.id = 0
        node_marker.type = Marker.SPHERE_LIST
        node_marker.action = Marker.ADD
        node_marker.scale.x = 0.2
        node_marker.scale.y = 0.2
        node_marker.scale.z = 0.2
        node_marker.pose.orientation.w = 1.0
        
        for nid, nd in graph.nodes(data=True):
            x, y = nd["xy_m"]
            point = Point()
            point.x = float(x)
            point.y = float(y)
            point.z = 0.0
            node_marker.points.append(point)
            
            # Color según tipo de nodo
            color = ColorRGBA()
            if nd.get("type") == "junction":
                color.r, color.g, color.b = 1.0, 0.0, 0.0  # Rojo para junctions
            elif nd.get("type") == "turn":
                color.r, color.g, color.b = 0.0, 1.0, 1.0  # Cyan para turns
            else:
                color.r, color.g, color.b = 0.0, 1.0, 0.0  # Verde para endpoints
            color.a = 1.0
            node_marker.colors.append(color)
        
        marker_array.markers.append(node_marker)
        
        # Marker para aristas (líneas)
        edge_marker = Marker()
        edge_marker.header.frame_id = frame_id
        edge_marker.header.stamp = self.get_clock().now().to_msg()
        edge_marker.ns = "edges"
        edge_marker.id = 1
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        edge_marker.scale.x = 0.05
        edge_marker.pose.orientation.w = 1.0
        edge_marker.color.r = 0.0
        edge_marker.color.g = 0.5
        edge_marker.color.b = 1.0
        edge_marker.color.a = 0.8
        
        for u, v, ed in graph.edges(data=True):
            if not ed.get("enabled", True):
                continue
            
            u_x, u_y = graph.nodes[u]["xy_m"]
            v_x, v_y = graph.nodes[v]["xy_m"]
            
            p1 = Point()
            p1.x, p1.y, p1.z = float(u_x), float(u_y), 0.0
            p2 = Point()
            p2.x, p2.y, p2.z = float(v_x), float(v_y), 0.0
            
            edge_marker.points.extend([p1, p2])
        
        marker_array.markers.append(edge_marker)
        
        # Marker de texto para IDs de nodos
        for idx, (nid, nd) in enumerate(graph.nodes(data=True)):
            text_marker = Marker()
            text_marker.header.frame_id = frame_id
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "node_labels"
            text_marker.id = idx + 100
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            x, y = nd["xy_m"]
            text_marker.pose.position.x = float(x)
            text_marker.pose.position.y = float(y)
            text_marker.pose.position.z = 0.3
            text_marker.pose.orientation.w = 1.0
            
            text_marker.text = nid
            text_marker.scale.z = 0.15
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f'Publicados {len(marker_array.markers)} markers')

    # ==================== FUNCIONES DE PROCESAMIENTO ====================
    # (Copiadas del documento original con ajustes mínimos)
    
    def clean_mask(self, mask: np.ndarray, open_iters=0, close_iters=1, kernel=3) -> np.ndarray:
        k = np.ones((kernel, kernel), np.uint8)
        x = (mask.astype(np.uint8) * 255)
        x = cv2.morphologyEx(x, cv2.MORPH_OPEN, k, iterations=open_iters)
        x = cv2.morphologyEx(x, cv2.MORPH_CLOSE, k, iterations=close_iters)
        return x > 0

    def keep_largest_component(self, mask: np.ndarray) -> np.ndarray:
        x = mask.astype(np.uint8)
        n, labels = cv2.connectedComponents(x)
        if n <= 1:
            return mask
        counts = np.bincount(labels.reshape(-1))
        counts[0] = 0
        keep = counts.argmax()
        return labels == keep

    def distance_transform(self, free_mask: np.ndarray):
        src = free_mask.astype(np.uint8) * 255
        return cv2.distanceTransform(src, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)

    def skeleton_free(self, free_mask: np.ndarray) -> np.ndarray:
        return skeletonize(free_mask).astype(bool, copy=False)

    def filter_skeleton_by_clearance(self, skel: np.ndarray, dist_cells: np.ndarray, min_clearance_cells: float) -> np.ndarray:
        return skel & (dist_cells >= min_clearance_cells)

    def skeleton_degree(self, skel: np.ndarray) -> np.ndarray:
        k = np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]], dtype=np.uint8)
        return cv2.filter2D(skel.astype(np.uint8), -1, k, borderType=cv2.BORDER_CONSTANT)

    def cell_to_xy(self, cell: Tuple[int, int], resolution_m: float, origin_xy_m: Tuple[float, float], height: int) -> Tuple[float, float]:
        """
        Convierte coordenadas de celda (row, col) a coordenadas del mundo (x, y).
        IMPORTANTE: Después de flipud(), row=0 está ARRIBA en la imagen numpy,
        pero corresponde al Y MÁS ALTO en el mundo real.
        """
        r, c = cell
        x0, y0 = origin_xy_m
        # X: directamente de columnas (sin cambio)
        x = x0 + c * resolution_m
        # Y: invertir porque row=0 (arriba en imagen) = y_max (arriba en mundo)
        # Formula: y = y0 + (height - 1 - r) * resolution
        y = y0 + (height - 1 - r) * resolution_m
        return (x, y)

    def neighbors8(self, cell: Tuple[int, int], skeleton: np.ndarray) -> List[Tuple[int, int]]:
        h, w = skeleton.shape
        r, c = cell
        out = []
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                rr, cc = r + dr, c + dc
                if 0 <= rr < h and 0 <= cc < w and skeleton[rr, cc]:
                    out.append((rr, cc))
        return out

    def skeleton_node_cells(self, skeleton: np.ndarray, deg: np.ndarray, junction_degree: int, include_endpoints: bool) -> List[Tuple[int, int]]:
        ys, xs = np.where(skeleton)
        out = []
        for r, c in zip(ys.tolist(), xs.tolist()):
            d = int(deg[r, c])
            if (include_endpoints and d == 1) or (d >= junction_degree):
                out.append((r, c))
        return out

    def node_id_map(self, node_cells: List[Tuple[int, int]], prefix: str = "n") -> Dict[Tuple[int, int], str]:
        return {cell: f"{prefix}{i}" for i, cell in enumerate(node_cells)}

    def init_graph(self, frame_id: str, resolution_m: float, origin_xy_m: Tuple[float, float], height: int) -> nx.Graph:
        return nx.Graph(frame_id=frame_id, resolution_m=resolution_m, origin_xy_m=origin_xy_m, height=height)

    def add_nodes_from_cells(self, g: nx.Graph, node_id: Dict[Tuple[int, int], str], deg: np.ndarray, 
                            resolution_m: float, origin_xy_m: Tuple[float, float], height: int, junction_degree: int) -> None:
        for cell, nid in node_id.items():
            d = int(deg[cell[0], cell[1]])
            g.add_node(
                nid,
                cell=cell,
                xy_m=self.cell_to_xy(cell, resolution_m, origin_xy_m, height),
                type=("junction" if d >= junction_degree else "endpoint"),
                labels={},
            )

    def trace_edge_path(self, start_node: Tuple[int, int], first_step: Tuple[int, int], 
                       skeleton: np.ndarray, node_set: Set[Tuple[int, int]]) -> List[Tuple[int, int]]:
        path = [start_node]
        prev, cur = start_node, first_step
        while True:
            path.append(cur)
            if cur in node_set:
                return path
            nxt = None
            for cand in self.neighbors8(cur, skeleton):
                if cand != prev:
                    nxt = cand
                    break
            if nxt is None:
                return path
            prev, cur = cur, nxt

    def add_edges_from_skeleton(self, g: nx.Graph, node_id: Dict[Tuple[int, int], str], skeleton: np.ndarray) -> None:
        node_set = set(node_id.keys())
        visited = set()

        for u_cell, u in node_id.items():
            for v_cell in self.neighbors8(u_cell, skeleton):
                if (u_cell, v_cell) in visited:
                    continue
                visited.add((u_cell, v_cell))

                path = self.trace_edge_path(u_cell, v_cell, skeleton, node_set)
                end_cell = path[-1]
                if end_cell not in node_set:
                    continue

                v = node_id[end_cell]
                if u == v or g.has_edge(u, v):
                    continue

                g.add_edge(u, v, path_cells=path, labels={}, metrics={}, enabled=True)

    def skeleton_to_graph(self, skel: np.ndarray, resolution_m: float, origin_xy_m: Tuple[float, float] = (0.0, 0.0),
                         frame_id: str = "map", height: int = 0, junction_degree: int = 3, include_endpoints: bool = True) -> nx.Graph:
        skeleton = skel.astype(bool)
        deg = self.skeleton_degree(skeleton)
        
        # Si no se proporciona height, usar la altura de la imagen
        if height == 0:
            height = skeleton.shape[0]

        nodes = self.skeleton_node_cells(skeleton=skeleton, deg=deg, junction_degree=junction_degree, include_endpoints=include_endpoints)
        nid = self.node_id_map(nodes)

        g = self.init_graph(frame_id=frame_id, resolution_m=resolution_m, origin_xy_m=origin_xy_m, height=height)
        self.add_nodes_from_cells(g=g, node_id=nid, deg=deg, resolution_m=resolution_m, origin_xy_m=origin_xy_m, height=height, junction_degree=junction_degree)
        self.add_edges_from_skeleton(g=g, node_id=nid, skeleton=skeleton)
        return g

    def bresenham(self, a: Tuple[int, int], b: Tuple[int, int]):
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

    def segment_ok(self, free_mask: np.ndarray, a: Tuple[int, int], b: Tuple[int, int],
                   dist_cells: Optional[np.ndarray] = None, min_clearance_cells: float = 0.0) -> bool:
        if min_clearance_cells > 0 and dist_cells is None:
            raise ValueError("dist_cells required when min_clearance_cells > 0")
        for r, c in self.bresenham(a, b):
            if not free_mask[r, c]:
                return False
            if dist_cells is not None and dist_cells[r, c] < min_clearance_cells:
                return False
        return True

    def path_to_los_anchors(self, free_mask: np.ndarray, path_cells: List[Tuple[int, int]],
                           dist_cells: Optional[np.ndarray], min_clearance_cells: float) -> List[Tuple[int, int]]:
        if len(path_cells) < 2:
            return path_cells

        anchors = [path_cells[0]]
        i, n = 0, len(path_cells)

        while i < n - 1:
            best = i + 1
            j = i + 1
            while j < n and self.segment_ok(free_mask, path_cells[i], path_cells[j], dist_cells, min_clearance_cells):
                best = j
                j += 1
            anchors.append(path_cells[best])
            i = best

        return anchors

    def edges_to_los_chains(self, g: nx.Graph, free_mask: np.ndarray, dist_cells: Optional[np.ndarray],
                           min_clearance_m: float) -> nx.Graph:
        res = float(g.graph.get("resolution_m", 1.0))
        origin = tuple(g.graph.get("origin_xy_m", (0.0, 0.0)))
        frame_id = g.graph.get("frame_id", "map")
        height = int(g.graph.get("height", free_mask.shape[0]))
        min_clearance_cells = (min_clearance_m / res) if min_clearance_m > 0 else 0.0

        def xy(cell: Tuple[int, int]) -> Tuple[float, float]:
            return self.cell_to_xy(cell, res, origin, height)

        out = nx.Graph(frame_id=frame_id, resolution_m=res, origin_xy_m=origin, height=height)
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

            anchors = self.path_to_los_anchors(free_mask, path, dist_cells, min_clearance_cells)
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

    def snap_merge_nodes(self, g: nx.Graph, snap_radius_cells: int = 3) -> nx.Graph:
        out = nx.Graph(**g.graph)

        buckets = {}
        for nid, nd in g.nodes(data=True):
            r, c = nd["cell"]
            key = (int(r // snap_radius_cells), int(c // snap_radius_cells))
            buckets.setdefault(key, []).append(nid)

        representative = {}
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
            # Recalcular xy_m con la nueva celda
            res = float(g.graph.get("resolution_m", 1.0))
            origin = tuple(g.graph.get("origin_xy_m", (0.0, 0.0)))
            height = int(g.graph.get("height", 0))
            base["xy_m"] = self.cell_to_xy((r0, c0), res, origin, height)
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


def main(args=None):
    rclpy.init(args=args)
    node = MapSemanticExtractorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()