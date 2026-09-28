#!/usr/bin/env python3
"""Convert the Gazebo Classic Stewart model (stewart_fractal.sdf) to Gazebo Harmonic.

DART in gz-physics cannot build closed kinematic chains out of moving joints
("Asked to create a closed kinematic chain ... not supported by the dartsim
wrapper yet"). It can close a loop with a FIXED joint, which becomes a weld
constraint. So every leg is re-rooted as one tree that ends in a weld:

  base -rev- bottom_ball -rev- cylinder -prismatic- shaft
       -universal- top_ball -rev- top_cap =weld= platform

SDF cannot express those welds either (dartsim rejects a second parent joint on
platform_link, even a fixed one), so they are created at runtime as
DetachableJoint components by the controller plugin (joint_controller.cpp),
which DART implements as weld constraints.

Each leg is a UPS chain (2 + 1 + 3 DOF). The classic model also carried a
second leg-spin DOF at the bottom (revolute2 axis2) that is dropped here,
because a redundant spin mode is badly conditioned under DART's LCP solver.

Joint frames/axes are recomputed from the classic semantics (SDF 1.5: joint
pose in child link frame, axis in joint frame) and written as SDF 1.9 with
axes expressed in __model__, so re-parenting joints cannot flip an axis.

Usage: python3 convert_to_gz.py [in.sdf] [out.sdf]
"""

import sys
import xml.etree.ElementTree as ET

import numpy as np

MATERIALS = {
    'Gazebo/Black': (0.05, 0.05, 0.05),
    'Gazebo/DarkGrey': (0.18, 0.18, 0.18),
    'Gazebo/Footway': (0.55, 0.55, 0.52),
    'Gazebo/Gold': (0.85, 0.68, 0.15),
    'Gazebo/Trunk': (0.35, 0.22, 0.10),
    'Gazebo/White': (0.95, 0.95, 0.95),
}
UNLIMITED = 1e16
NUM_LEGS = 6


def rpy_to_rot(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def parse_pose(text):
    v = [float(x) for x in (text or '0 0 0 0 0 0').split()]
    return np.array(v[:3]), rpy_to_rot(*v[3:])


def fmt(v):
    return ' '.join(f'{x:.12g}' for x in v)


class Converter:
    def __init__(self, path):
        self.model = ET.parse(path).getroot().find('model')
        self.links = {l.get('name'): l for l in self.model.findall('link')}
        self.joints = {j.get('name'): j for j in self.model.findall('joint')}
        self.link_pose = {n: parse_pose(l.findtext('pose')) for n, l in self.links.items()}

    def classic_joint_frame(self, jname):
        """World position and rotation of a classic joint frame (pose in child frame)."""
        j = self.joints[jname]
        cp, cr = self.link_pose[j.findtext('child')]
        jp, jr = parse_pose(j.findtext('pose'))
        return cp + cr @ jp, cr @ jr

    def classic_axis(self, jname, tag='axis'):
        """Axis unit vector in model frame (classic: expressed in joint frame)."""
        _, rot = self.classic_joint_frame(jname)
        el = self.joints[jname].find(tag)
        xyz = np.array([float(x) for x in el.findtext('xyz').split()]) if el is not None else np.array([0, 0, 1.0])
        v = rot @ xyz
        return v / np.linalg.norm(v)

    def classic_limits(self, jname):
        lim = self.joints[jname].find('axis/limit')
        return float(lim.findtext('lower')), float(lim.findtext('upper'))

    # ------------------------------------------------------------------ output
    def build(self):
        sdf = ET.Element('sdf', version='1.9')
        model = ET.SubElement(sdf, 'model', name='stewart')

        plugin = ET.SubElement(model, 'plugin', name='stewart_platform::SDFJointController',
                               filename='stewart_joint_controller')
        old_init = self.model.find('plugin/gazebo_controller_init')
        new_init = ET.SubElement(plugin, 'gazebo_controller_init')
        for child in old_init:
            ET.SubElement(new_init, child.tag).text = child.text

        for name, link in self.links.items():
            model.append(self.convert_link(link))

        for i in range(1, NUM_LEGS + 1):
            model.append(self.top_cap_link(i))

        for i in range(1, NUM_LEGS + 1):
            for el in self.leg_joints(i):
                model.append(el)

        ET.indent(sdf, space='  ')
        return ET.ElementTree(sdf)

    def convert_link(self, link):
        out = ET.Element('link', name=link.get('name'))
        ET.SubElement(out, 'pose').text = link.findtext('pose')
        out.append(link.find('inertial'))
        for vis in link.findall('visual'):
            out.append(self.convert_visual(vis))
        for col in link.findall('collision'):
            out.append(col)
        return out

    def convert_visual(self, vis):
        out = ET.Element('visual', name=vis.get('name'))
        for tag in ('pose', 'cast_shadows', 'transparency'):
            if vis.find(tag) is not None:
                out.append(vis.find(tag))
        out.append(vis.find('geometry'))
        mat = vis.find('material')
        script_name = mat.findtext('script/name') if mat is not None else None
        new_mat = ET.SubElement(out, 'material')
        if script_name == 'ArucoVisualMarkerFractal/Marker':
            for tag, val in (('ambient', '1 1 1 1'), ('diffuse', '1 1 1 1'),
                             ('specular', '0 0 0 1'), ('emissive', '0 0 0 1')):
                ET.SubElement(new_mat, tag).text = val
            pbr = ET.SubElement(ET.SubElement(new_mat, 'pbr'), 'metal')
            ET.SubElement(pbr, 'albedo_map').text = \
                'model://aruco_visual_marker_fractal/materials/textures/aruco_mark_fractal.png'
            ET.SubElement(pbr, 'metalness').text = '0.0'
            ET.SubElement(pbr, 'roughness').text = '1.0'
        else:
            rgb = MATERIALS.get(script_name, (0.7, 0.7, 0.7))
            ET.SubElement(new_mat, 'ambient').text = fmt((*rgb, 1))
            ET.SubElement(new_mat, 'diffuse').text = fmt((*rgb, 1))
            ET.SubElement(new_mat, 'specular').text = '0.2 0.2 0.2 1'
        return out

    def top_cap_link(self, i):
        """Tiny link at the top ball centre that gets welded to the platform."""
        ball = self.links[f'top_ball{i}_link']
        self.link_pose[f'top_cap{i}_link'] = self.link_pose[f'top_ball{i}_link']
        out = ET.Element('link', name=f'top_cap{i}_link')
        ET.SubElement(out, 'pose').text = ball.findtext('pose')
        inertial = ET.SubElement(out, 'inertial')
        ET.SubElement(inertial, 'mass').text = '0.01'
        inertia = ET.SubElement(inertial, 'inertia')
        for tag in ('ixx', 'iyy', 'izz'):
            ET.SubElement(inertia, tag).text = '4e-05'
        for tag in ('ixy', 'ixz', 'iyz'):
            ET.SubElement(inertia, tag).text = '0'
        return out

    def joint(self, name, jtype, parent, child, world_pos, axes, damping=None, velocity=None, effort=None):
        """Emit an SDF 1.9 joint located at world_pos. axes: list of (axis, lower, upper)."""
        cp, cr = self.link_pose[child]
        local = cr.T @ (world_pos - cp)
        out = ET.Element('joint', name=name, type=jtype)
        ET.SubElement(out, 'parent').text = parent
        ET.SubElement(out, 'child').text = child
        ET.SubElement(out, 'pose').text = fmt((*local, 0, 0, 0))
        for tag, (axis, lower, upper) in zip(('axis', 'axis2'), axes):
            ax = ET.SubElement(out, tag)
            ET.SubElement(ax, 'xyz', expressed_in='__model__').text = fmt(axis)
            lim = ET.SubElement(ax, 'limit')
            ET.SubElement(lim, 'lower').text = f'{lower:.12g}'
            ET.SubElement(lim, 'upper').text = f'{upper:.12g}'
            if velocity is not None:
                ET.SubElement(lim, 'velocity').text = f'{velocity:g}'
            if effort is not None:
                ET.SubElement(lim, 'effort').text = f'{effort:g}'
            if damping is not None:
                ET.SubElement(ET.SubElement(ax, 'dynamics'), 'damping').text = f'{damping:g}'
        return out

    def leg_joints(self, i):
        base = 'base_link'
        bball, tball, cap = f'bottom_ball{i}_link', f'top_ball{i}_link', f'top_cap{i}_link'
        cyl, shaft = f'piston{i}_cylinder_link', f'piston{i}_shaft_link'

        j_bball = f'bottom_ball{i}_joint'
        j_bpitch = f'piston{i}_bottom_pitch_joint'
        j_prism = f'piston{i}_prismatic_joint'
        j_tpitch = f'piston{i}_top_pitch_joint'
        j_tball = f'top_ball{i}_joint'

        bottom_pos, _ = self.classic_joint_frame(j_bpitch)
        top_pos, _ = self.classic_joint_frame(j_tpitch)
        prism_pos, _ = self.classic_joint_frame(j_prism)
        top_ball_pos, _ = self.classic_joint_frame(j_tball)

        prism = self.joints[j_prism]
        return [
            self.joint(j_bball, 'revolute', base, bball, bottom_pos,
                       [(self.classic_axis(j_bball), *self.classic_limits(j_bball))], velocity=1),
            self.joint(j_bpitch, 'revolute', bball, cyl, bottom_pos,
                       [(self.classic_axis(j_bpitch), *self.classic_limits(j_bpitch))]),
            self.joint(j_prism, 'prismatic', cyl, shaft, prism_pos,
                       [(self.classic_axis(j_prism), *self.classic_limits(j_prism))],
                       damping=float(prism.findtext('axis/dynamics/damping')),
                       velocity=float(prism.findtext('axis/limit/velocity')),
                       effort=float(prism.findtext('axis/limit/effort'))),
            # Classic: universal top_ball -> shaft, axis x plus default axis2 z
            # (leg spin). Re-parented so the shaft is the parent.
            self.joint(j_tpitch, 'universal', shaft, tball, top_pos,
                       [(self.classic_axis(j_tpitch), *self.classic_limits(j_tpitch)),
                        (self.classic_axis(j_tpitch, 'axis2'), -UNLIMITED, UNLIMITED)]),
            # Classic: revolute platform -> top_ball (axis y, axis2 ignored).
            self.joint(j_tball, 'revolute', tball, cap, top_ball_pos,
                       [(self.classic_axis(j_tball), *self.classic_limits(j_tball))], velocity=1),
            # top_cap -> platform_link is welded at runtime by the controller plugin.
        ]


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else 'stewart_fractal.sdf'
    dst = sys.argv[2] if len(sys.argv) > 2 else 'stewart_fractal_gz.sdf'
    Converter(src).build().write(dst, xml_declaration=True, encoding='utf-8')
    print(f'wrote {dst}')


if __name__ == '__main__':
    main()
