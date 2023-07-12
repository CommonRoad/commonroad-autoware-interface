from typing import List

from commonroad.geometry.shape import Polygon
import numpy as np

vertices = [
    [448.74761, -396.89807],
    [449.5026424880962, -392.2530335346689],
    [452.9548887957979, -392.81475715806425],
    [452.22606, -397.294],
    [448.74761, -396.89807],
]


def _convert_vertex_list_to_polygons(vertices: List[List[float]]) -> List[Polygon]:
    """Convert a list of vertices as returned by SPOT to a list of polygons.

    SPOT returns all polygons as one list of vertices that needs to be split.
    This function converts the list of vertices to a list of polygons by
    creating a separate polygon whenever a vertex is reached, which has the
    same x and y coordinates as a preceding vertex.

    Example:
        SPOT returns the following list of vertices:
            [(0, 0), (1, 0), (1, 1), (0, 1), (0, 0), (2, 0), (2, 1), (0, 1), (2, 0)]
        This list is converted to the following list of polygons:
            [Polygon([(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)]),
                Polygon([(2, 0), (2, 1), (0, 1), (2, 0)])]

    Raises:
        ValueError: If the list of vertices is not closed correctly.

    """
    shapes: List[Polygon] = []
    first_idx = 0
    while first_idx < len(vertices):
        first_vertex = vertices[first_idx]
        for next_idx, next_vertex in enumerate(vertices[first_idx + 1 :], start=first_idx + 1):
            # check if polygon is closed by comparing x and y coords
            if first_vertex[0] != next_vertex[0] or first_vertex[1] != next_vertex[1]:
                if next_idx == len(vertices) - 1:
                    # last vertex reached, but polygon is not closed
                    raise ValueError(
                        "List of vertices is not closed correctly. \n" f"Vertices: {vertices}"
                    )
                continue

            # check if polygon has at least 3 vertices
            if next_idx - first_idx < 3:
                # TODO: handle logging
                print(
                    "Warning: one polygon skipped when copying predicted occupancies to CommonRoad"
                )
                # skip this first_vertex and continue with the next one
                first_idx += 1
                break
            else:
                # polygon is closed correctly and has at least 3 vertices
                shapes.append(Polygon(np.array(vertices[first_idx : next_idx + 1], dtype=float)))
                first_idx = next_idx + 1
                break
    return shapes


print(_convert_vertex_list_to_polygons(vertices))
