"""Generate Host collision assets without changing the SDK description.

Requires numpy, trimesh==4.5.3, networkx, coacd==1.0.14 and mujoco.
Run from the Host root. Cached mesh decompositions are reused unless --rebuild.
"""
from pathlib import Path
import argparse
import hashlib
import json
import xml.etree.ElementTree as ET

import coacd
import trimesh
import numpy as np
from scipy.optimize import linprog
from scipy.spatial import ConvexHull, HalfspaceIntersection
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent.parent
SOURCE = ROOT / 'Panthera-HT_SDK/panthera_python/Panthera-HT_description_V1.1.0/mujoco'
OUTPUT = ROOT / 'models'
PARAMS = dict(threshold=0.002, real_metric=True, max_convex_hull=48,
              preprocess_mode='on', preprocess_resolution=40, resolution=1500,
              mcts_nodes=15, mcts_iterations=60, mcts_max_depth=3,
              decimate=True, max_ch_vertex=48, seed=42)
# Whole-assembly remeshing bridges the narrow motor/bracket gap at the zero pose.
# Keep disconnected CAD components separate for these two links. Indices refer
# to nondegenerate convex-hull components in the pinned trimesh split order.
COMPONENT_PARTS = {'link2_collision': {79, 80}, 'link5_collision': {21}}
FINE_PARAMS = {**PARAMS, 'threshold': 0.0005, 'max_convex_hull': 96,
               'preprocess_resolution': 80}
# These three approximate jaw hulls overlap at the confirmed 0-mm endpoint.
# Inset their collision skin only; retain finger-to-finger and external contacts.
JAW_SKIN = {('gripper_L_link_collision', 14), ('gripper_L_link_collision', 28),
            ('gripper_R_link_collision', 20)}


def add_wrist_camera(root):
    """Enable the SDK's optional fixed camera in the Host model only."""
    if root.find(".//body[@name='camera_link']") is not None:
        return
    urdf_path = SOURCE.parent / 'urdf/Panthera-HT_description_V1.1.0_follower.urdf'
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    urdf = ET.parse(urdf_path, parser).getroot()
    link = urdf.find("link[@name='camera_link']")
    joint = urdf.find("joint[@name='camera_joint']")
    if link is None or joint is None:
        for comment in urdf.iter(ET.Comment):
            if '<link name="camera_link">' in (comment.text or ''):
                optional = ET.fromstring(f'<robot>{comment.text}</robot>')
                link = optional.find("link[@name='camera_link']")
                joint = optional.find("joint[@name='camera_joint']")
                break
    if link is None or joint is None or joint.get('type') != 'fixed':
        raise ValueError('SDK must define camera_link and its fixed camera_joint')
    parent = root.find(f".//body[@name='{joint.find('parent').get('link')}']")
    if parent is None:
        raise ValueError('Camera parent is missing from the scene')
    origin = joint.find('origin')
    quaternion = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy'), sep=' ')).as_quat()
    body = ET.SubElement(parent, 'body', name='camera_link', pos=origin.get('xyz'),
                         quat=' '.join(map(str, quaternion[[3, 0, 1, 2]])))
    inertial = link.find('inertial')
    inertia = inertial.find('inertia')
    ET.SubElement(body, 'inertial', pos=inertial.find('origin').get('xyz'),
                  mass=inertial.find('mass').get('value'),
                  fullinertia=' '.join(inertia.get(key) for key in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')))
    asset = root.find('asset')
    visual = link.find('visual/geometry/mesh').get('filename').split('/', 3)[3]
    scene = trimesh.load(SOURCE.parent / visual, force='scene')
    for index, node in enumerate(scene.graph.nodes_geometry):
        _, geometry = scene.graph[node]
        part = scene.geometry[geometry]
        color = part.visual.material.main_color if part.visual.kind == 'texture' else part.visual.main_color
        name = f'camera_link_{index}'
        if not (SOURCE / f'meshes/obj/{name}.obj').exists():
            raise FileNotFoundError(f'SDK is missing the converted camera mesh: {name}')
        ET.SubElement(asset, 'mesh', name=name, file=f'meshes/obj/{name}.obj')
        ET.SubElement(body, 'geom', name=f'{name}_visual', mesh=name, type='mesh',
                      group='1', contype='0', conaffinity='0', density='0',
                      rgba=' '.join(map(str, np.asarray(color, dtype=float) / 255)))
    ET.SubElement(asset, 'mesh', name='camera_link_collision', file='meshes/obj/camera_link_collision.obj')
    ET.SubElement(body, 'geom', mesh='camera_link_collision', type='mesh', group='3')


def decompose(mesh, name):
    if name not in COMPONENT_PARTS:
        return coacd.run_coacd(coacd.Mesh(mesh.vertices, mesh.faces), **PARAMS)
    parts = []
    index = 0
    for component in mesh.split(only_watertight=False):
        try:
            hull = component.convex_hull
        except Exception:
            # Isolated planar/linear facets cannot form a volumetric collider.
            continue
        if hull.volume < 1e-10:
            continue
        if index in COMPONENT_PARTS[name]:
            parts.extend(coacd.run_coacd(coacd.Mesh(component.vertices, component.faces), **FINE_PARAMS))
        else:
            parts.append((hull.vertices, hull.faces))
        index += 1
    return parts


def inset(vertices, distance=0.0004):
    """Offset convex planes inward; omit fragments thinner than twice the skin."""
    planes = ConvexHull(vertices).equations
    normals, offsets = planes[:, :3], planes[:, 3]
    center = linprog([0, 0, 0, -1], A_ub=np.column_stack((normals, np.ones(len(planes)))),
                     b_ub=-offsets, bounds=[(None, None)] * 3 + [(0, None)], method='highs')
    if not center.success or center.x[3] <= distance * 1.01:
        return None
    shifted = planes.copy()
    shifted[:, 3] += distance
    points = HalfspaceIntersection(shifted, center.x[:3]).intersections
    return trimesh.Trimesh(points, ConvexHull(points).simplices).convex_hull


def enable_gripper(root):
    actuator = root.find('actuator')
    for side, limits in [('L', '-0.041 0'), ('R', '0 0.041')]:
        name = f'gripper_{side}_joint'
        body = root.find(f".//body[@name='gripper_{side}_link']")
        ET.SubElement(body, 'joint', name=name, type='slide', axis='0 1 0',
                      range=limits, damping='0.4', armature='0.0001')
        ET.SubElement(actuator, 'motor', name=f'{name}_force', joint=name,
                      gear='1', ctrllimited='true', ctrlrange='-5 5')
    equality = ET.SubElement(root, 'equality')
    ET.SubElement(equality, 'joint', joint1='gripper_L_joint', joint2='gripper_R_joint',
                  polycoef='0 -1 0 0 0', solref='0.003 1')
    for key in root.findall('keyframe/key'):
        key.set('qpos', key.get('qpos') + ' 0 0')


def build(rebuild=False):
    root = ET.parse(SOURCE / 'scene.xml').getroot()
    add_wrist_camera(root)
    enable_gripper(root)
    asset = root.find('asset')
    collision_dir = OUTPUT / 'collision'
    collision_dir.mkdir(parents=True, exist_ok=True)
    (OUTPUT / 'contact').mkdir(exist_ok=True)
    manifest_path = OUTPUT / 'collision-manifest.json'
    previous = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    manifest = {}
    coacd.set_log_level('warn')
    for body in root.findall('.//body'):
        for geom in list(body.findall('geom')):
            if geom.get('group') != '3':
                continue
            name = geom.get('mesh')
            original = asset.find(f"mesh[@name='{name}']")
            path = SOURCE / original.get('file')
            settings = PARAMS
            if name in COMPONENT_PARTS:
                settings = dict(params=PARAMS, fine=FINE_PARAMS, components=sorted(COMPONENT_PARTS[name]))
            digest = hashlib.sha256(path.read_bytes() + json.dumps(settings, sort_keys=True).encode()).hexdigest()
            cached = previous.get(name, {})
            if not rebuild and cached.get('sha256') == digest and all((OUTPUT / f).exists() for f in cached['files']):
                files = cached['files']
            else:
                mesh = trimesh.load(path, force='mesh')
                print(f'Decomposing {name}: {len(mesh.faces)} triangles', flush=True)
                parts = decompose(mesh, name)
                files = []
                for index, (vertices, faces) in enumerate(parts):
                    file = f'collision/{name}_{index:02d}.obj'
                    trimesh.Trimesh(vertices, faces, process=False).export(OUTPUT / file)
                    files.append(file)
                print(f'  {len(files)} convex parts', flush=True)
            manifest[name] = dict(sha256=digest, files=files)
            # Save progress so lengthy offline preprocessing can be resumed.
            manifest_path.write_text(json.dumps({**previous, **manifest}, indent=2) + '\n')
            body.remove(geom)
            asset.remove(original)
            for index, file in enumerate(files):
                part = f'{name}_{index:02d}'
                if name in COMPONENT_PARTS or (name, index) in JAW_SKIN:
                    raw = trimesh.load(OUTPUT / file, force='mesh')
                    fitted = inset(raw.vertices, 0.001 if (name, index) in JAW_SKIN else 0.0004)
                    if fitted is None:
                        continue
                    file = f'contact/{part}_skin.obj'
                    fitted.export(OUTPUT / file)
                ET.SubElement(asset, 'mesh', name=part, file=file)
                ET.SubElement(body, 'geom', name=part, type='mesh', mesh=part,
                              group='3', contype='2', conaffinity='3', density='0',
                              friction='0.6 0.01 0.001', margin='0.00005',
                              solref='0.003 1', solimp='0.99 0.999 0.0005')
    contact = ET.SubElement(root, 'contact')
    chain = ['base_link'] + [f'link{i}' for i in range(1, 7)]
    for parent, child in zip(chain, chain[1:]):
        ET.SubElement(contact, 'exclude', body1=parent, body2=child)
    root.find('option').set('iterations', '100')
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(OUTPUT / 'panthera.xml', encoding='utf-8', xml_declaration=True)
    used = {mesh.get('file') for mesh in asset.findall('mesh')}
    for generated in (OUTPUT / 'contact').glob('*.obj'):
        if generated.relative_to(OUTPUT).as_posix() not in used:
            generated.unlink()
    manifest_path.write_text(json.dumps(manifest, indent=2) + '\n')
    print('Generated models/panthera.xml', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--rebuild', action='store_true')
    build(parser.parse_args().rebuild)
