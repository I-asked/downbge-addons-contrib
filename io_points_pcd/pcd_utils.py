# ##### BEGIN GPL LICENSE BLOCK #####
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software Foundation,
#  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# ##### END GPL LICENSE BLOCK #####

from __future__ import absolute_import
import bpy

from . import pcdparser


def create_and_link_mesh(name, points):
    """
    Create a blender mesh and object called name from a list of
    *points* and link it in the current scene.
    """

    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(points, [], [])

    # update mesh to allow proper display
    mesh.validate()
    mesh.update()

    scene = bpy.context.scene

    obj = bpy.data.objects.new(name, mesh)
    scene.objects.link(obj)
    obj.select = True


def import_pcd(filepath, name="new_pointcloud"):
    parser = pcdparser.PCDParser.factory(filepath, pcdparser.PointXYZ)
    parser.parseFile()
    points = parser.getPoints()

    blender_points = []
    for point in points:
        blender_points.append((point.x, point.y, point.z))

    create_and_link_mesh(name, blender_points)
  

def export_pcd(filepath):
    obj = bpy.context.active_object

    # apply object transformation and modifiers
    mesh = obj.to_mesh(bpy.context.scene, True, "PREVIEW")
    objmat = obj.matrix_world
    
    points = []
    for vert in mesh.vertices:
        co = objmat * vert.co
        point = pcdparser.PointXYZ()
        point.x = co.x
        point.y = co.y
        point.z = co.z
        points.append(point)

    writer = pcdparser.PCDWriter(points)
    writer.write(filepath)


