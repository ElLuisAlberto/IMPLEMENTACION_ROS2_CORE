#!/usr/bin/env python3
"""Convert the supplied Inventor GLB into a coloured, metre-scale COLLADA.

The source's per-node transforms and material colours are retained; its single
embedded texture is not converted. Requires numpy and pycollada at export time.
"""
import json
from pathlib import Path
import struct

import collada
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
MESH_DIR = ROOT / 'src/sistema_gripper_instrumentado_description/models/sistema_gripper_instrumentado/meshes'
SOURCE = MESH_DIR / 'sistema_gripper_instrumentado.glb'
DESTINATION = MESH_DIR / 'sistema_gripper_instrumentado_colores.dae'


def load_glb(path):
    data = path.read_bytes()
    magic, version, length = struct.unpack_from('<4sII', data)
    if magic != b'glTF' or version != 2 or length != len(data):
        raise ValueError('Invalid GLB 2.0 file')
    offset, chunks = 12, {}
    while offset < len(data):
        size, kind = struct.unpack_from('<II', data, offset)
        offset += 8
        chunks[kind] = data[offset:offset + size]
        offset += size
    return json.loads(chunks[0x4E4F534A]), chunks[0x004E4942]


def accessor(doc, binary, index):
    a = doc['accessors'][index]
    view = doc['bufferViews'][a['bufferView']]
    dtype = {5126: '<f4', 5125: '<u4'}[a['componentType']]
    width = {'SCALAR': 1, 'VEC3': 3}[a['type']]
    item = np.dtype(dtype).itemsize
    offset = view.get('byteOffset', 0) + a.get('byteOffset', 0)
    stride = view.get('byteStride', width * item)
    values = np.ndarray((a['count'], width), dtype=dtype, buffer=binary,
                        offset=offset, strides=(stride, item))
    return values.copy()


def convert():
    doc, binary = load_glb(SOURCE)
    grouped = {}

    def visit(index, parent):
        node = doc['nodes'][index]
        if any(key in node for key in ('translation', 'rotation', 'scale')):
            raise ValueError('Unexpected GLB TRS transform; update exporter')
        matrix = np.asarray(node.get('matrix', np.eye(4).ravel().tolist()), dtype=float).reshape(4, 4, order='F')
        transform = parent @ matrix
        if 'mesh' in node:
            for primitive in doc['meshes'][node['mesh']]['primitives']:
                if primitive.get('mode', 4) != 4:
                    raise ValueError('Only triangular GLB primitives are supported')
                vertices = accessor(doc, binary, primitive['attributes']['POSITION'])
                vertices = vertices @ transform[:3, :3].T + transform[:3, 3]
                normals = accessor(doc, binary, primitive['attributes']['NORMAL'])
                normals = normals @ np.linalg.inv(transform[:3, :3])
                normals /= np.maximum(np.linalg.norm(normals, axis=1)[:, None], 1e-12)
                indices = accessor(doc, binary, primitive['indices']).reshape(-1, 3)
                grouped.setdefault(primitive.get('material', -1), []).append((vertices, normals, indices))
        for child in node.get('children', []):
            visit(child, transform)

    for node in doc['scenes'][doc.get('scene', 0)]['nodes']:
        visit(node, np.eye(4))
    output = collada.Collada()
    output.assetInfo.unitname = 'meter'
    output.assetInfo.unitmeter = 1.0
    output.assetInfo.upaxis = 'Z_UP'
    nodes = []
    for material_index, groups in sorted(grouped.items()):
        key = f'material_{material_index + 1}'
        source_material = doc['materials'][material_index] if material_index >= 0 else {}
        color = source_material.get('pbrMetallicRoughness', {}).get('baseColorFactor', [1., 1., 1., 1.])
        effect = collada.material.Effect(key + '_effect', [], 'phong', diffuse=tuple(color),
                                         ambient=tuple(color), double_sided=True)
        material = collada.material.Material(key, source_material.get('name', key), effect)
        output.effects.append(effect)
        output.materials.append(material)
        vertices, normals, triangles, offset = [], [], [], 0
        for points, directions, faces in groups:
            vertices.append(points)
            normals.append(directions)
            triangles.append(faces + offset)
            offset += len(points)
        vertices = np.concatenate(vertices).astype(np.float32).ravel()
        normals = np.concatenate(normals).astype(np.float32).ravel()
        triangles = np.concatenate(triangles).astype(np.uint32).ravel()
        source = collada.source.FloatSource(key + '_vertices', vertices, ('X', 'Y', 'Z'))
        normal_source = collada.source.FloatSource(key + '_normals', normals, ('X', 'Y', 'Z'))
        geometry = collada.geometry.Geometry(output, key + '_geometry', key, [source, normal_source])
        inputs = collada.source.InputList()
        inputs.addInput(0, 'VERTEX', '#' + source.id)
        inputs.addInput(1, 'NORMAL', '#' + normal_source.id)
        indices = np.column_stack((triangles, triangles)).ravel()
        geometry.primitives.append(geometry.createTriangleSet(indices, inputs, key))
        output.geometries.append(geometry)
        binding = collada.scene.MaterialNode(key, material, inputs=[])
        nodes.append(collada.scene.Node(key + '_node', children=[collada.scene.GeometryNode(geometry, [binding])]))
    scene = collada.scene.Scene('gripper', nodes)
    output.scenes.append(scene)
    output.scene = scene
    output.write(str(DESTINATION))
    print(f'Exported {len(nodes)} material groups to {DESTINATION}')


if __name__ == '__main__':
    convert()
