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
#  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110 - 1301, USA.
#
# ##### END GPL LICENSE BLOCK #####

from __future__ import absolute_import
from itertools import ifilter
bl_info = {
    "name": "Topokit 2",
    "author": "dustractor",
    "version": (2, 0),
    "blender": (2, 60, 0),
    "location": "edit mesh vertices/edges/faces menus",
    "description": "",
    "warning": "",
    "wiki_url": "",
    "tracker_url": "",
    "category": "Mesh"}


import bpy
# In between calls, this stores any data that is expensive or static,
# matched to the size of the mesh and the id of the operator that created it
cachedata = dict()
# and the object keeps the key to the cachedata
bpy.types.Object.tkkey = bpy.props.IntVectorProperty(size=4)

# just a mix-in for the operators...
class meshpoller(object):
    @classmethod
    def poll(self,context):
        try:
            assert context.active_object.type == "MESH"
        except:
            return False
        finally:
            return True

#BEGIN VERTICES SECTION

# This one works similarly to normal 'grow' (ctrl + NUMPAD_PLUS),
# except the original selection is not part of the result,
#
#   0--0--0         0--1--0
#   |  |  |         |  |  |
#   0--1--0  -->    1--0--1
#   |  |  |         |  |  |
#   0--0--0         0--1--0
#
class MESH_OT_vneighbors_edgewise(meshpoller,bpy.types.Operator):
    bl_idname = "mesh.v2v_by_edge"
    bl_label = "Neighbors by Edge"
    bl_options = set(["REGISTER","UNDO"])

    def execute(self,context):
        global cachedata
        bpy.ops.object.mode_set(mode="OBJECT")
        obj = context.active_object
        mesh = obj.data
        meshkey = (len(mesh.vertices),len(mesh.edges),len(mesh.polygons),id(self))
        next_state = bytearray(meshkey[0])
        if (meshkey == obj.tkkey) and (meshkey in cachedata):
            vert_to_vert_map,prev_state = cachedata[meshkey]
        else:
            vert_to_vert_map = dict((i, {}) for i in xrange(meshkey[0]))
            for a,b in mesh.edge_keys:
                vert_to_vert_map[a][b] = 1
                vert_to_vert_map[b][a] = 1
            obj.tkkey = meshkey
            prev_state = None
        if not prev_state:
            selected_vert_indices = ifilter(lambda _:mesh.vertices[_].select,xrange(len(mesh.vertices)))
        else:
            selected_vert_indices = ifilter(lambda _:mesh.vertices[_].select and not prev_state[_],xrange(len(mesh.vertices)))
        for v in selected_vert_indices:
            for neighbor_index in vert_to_vert_map[v]:
                next_state[neighbor_index] = True
        mesh.vertices.foreach_set("select",next_state)
        cachedata[meshkey] = (vert_to_vert_map,next_state)
        bpy.ops.object.mode_set(mode="EDIT")
        return set(["FINISHED"])

# This one is an alternate / counterpart to the previous.
# Think: diagonal opposite corners of a quad
# NOTE: does not apply to a triangle, since verts have no 'opposite'
#
#   0--0--0     1--0--1
#   |  |  |     |  |  |
#   0--1--0 --> 0--0--0
#   |  |  |     |  |  |
#   0--0--0     1--0--1
#
class MESH_OT_vneighbors_facewise(meshpoller,bpy.types.Operator):
    bl_idname = "mesh.v2v_facewise"
    bl_label = "Neighbors by Face - Edge"
    bl_options = set(["REGISTER","UNDO"])

    def execute(self,context):
        global cachedata
        bpy.ops.object.mode_set(mode="OBJECT")
        obj = context.active_object
        mesh = obj.data
        meshkey = (len(mesh.vertices),len(mesh.edges),len(mesh.polygons),id(self))
        next_state = bytearray(meshkey[0])
        if (meshkey == obj.tkkey) and (meshkey in cachedata):
            vert_to_vert_map = cachedata[meshkey]
        else:
            vert_to_vert_map = dict((i, {}) for i in xrange(meshkey[0]))
            for a,b in mesh.edge_keys:
                vert_to_vert_map[a][b] = 1
                vert_to_vert_map[b][a] = 1
            obj.tkkey = meshkey
        faces = ifilter(lambda face:(len(face.vertices)==4) and (face.select == False),mesh.polygons)
        for f in faces:
            has = False
            t = set()
            for v in f.vertices:
                if mesh.vertices[v].select:
                    has = True
                    t.update(vert_to_vert_map[v])
            if has:
                for v in f.vertices:
                    if not mesh.vertices[v].select:
                        if v not in t:
                            next_state[v]=1
        mesh.vertices.foreach_set("select",next_state)
        cachedata[meshkey] = vert_to_vert_map
        bpy.ops.object.mode_set(mode="EDIT")
        return set(["FINISHED"])

def vvmenuitem(self,context):
    self.layout.operator(MESH_OT_vneighbors_edgewise.bl_idname)
    self.layout.operator(MESH_OT_vneighbors_facewise.bl_idname)
    #for the sake of completeness, yes there is one alg missing - one for both...

#END VERTICES SECTION
#BEGIN EDGES SECTION

#   +--0--+--0--+--0--+          +--0--+--0--+--0--+
#   |     |     |     |          |     |     |     |
#   0     0     0     0          0     1     1     0
#   |     |     |     |          |     |     |     |
#   +--0--+--1--+--0--+   --->   +--0--+--0--+--0--+
#   |     |     |     |          |     |     |     |
#   0     0     0     0          0     1     1     0
#   |     |     |     |          |     |     |     |
#   +--0--+--0--+--0--+          +--0--+--0--+--0--+
class MESH_OT_eneighbors_shared_v_f(meshpoller,bpy.types.Operator):
    bl_idname = "mesh.e2e_evfe"
    bl_label = "Neighbors by Vert+Face"
    bl_options = set(["REGISTER","UNDO"])
    def execute(self,context):
        global cachedata
        bpy.ops.object.mode_set(mode="OBJECT")
        obj = context.active_object
        mesh = obj.data
        meshkey = (len(mesh.vertices),len(mesh.edges),len(mesh.polygons),id(self))
        state_mask = bytearray(meshkey[1])
        if (meshkey == obj.tkkey) and (meshkey in cachedata):
            edge_to_edges_dict = cachedata
        else:
            edge_key_to_index = dict((k, i) for i,k in enumerate(mesh.edge_keys))
            edge_to_edges_dict = dict((i, set()) for i in xrange(len(mesh.edges)))
            for f in mesh.polygons:
                fed=[edge_key_to_index[k] for k in f.edge_keys]
                for k in f.edge_keys:
                    edge_to_edges_dict[edge_key_to_index[k]].update(fed)
            obj.tkkey = meshkey
        for e in ifilter(lambda _:mesh.edges[_].select,edge_to_edges_dict):
            k1 = set(mesh.edges[e].key)
            for n in edge_to_edges_dict[e]:
                k2 = set(mesh.edges[n].key)
                if not k1.isdisjoint(k2):
                    state_mask[n] = True
        for e in mesh.edges:
            e.select ^= state_mask[e.index]
        cachedata[meshkey] = edge_key_to_index
        bpy.ops.object.mode_set(mode="EDIT")
        return set(["FINISHED"])


#   +--0--+--0--+--0--+          +--0--+--0--+--0--+
#   |     |     |     |          |     |     |     |
#   0     0     0     0          0     1     1     0
#   |     |     |     |          |     |     |     |
#   +--0--+--1--+--0--+   --->   +--1--+--0--+--1--+
#   |     |     |     |          |     |     |     |
#   0     0     0     0          0     1     1     0
#   |     |     |     |          |     |     |     |
#   +--0--+--0--+--0--+          +--0--+--0--+--0--+
class MESH_OT_eneighbors_shared_v(meshpoller,bpy.types.Operator):
    bl_idname = "mesh.e2e_eve"
    bl_label = "Neighbors by Vert"
    bl_options = set(["REGISTER","UNDO"])
    def execute(self,context):
        bpy.ops.object.mode_set(mode="OBJECT")
        mesh = context.active_object.data
        state_mask = bytearray(len(mesh.edges))
        for e in mesh.edges:
            state_mask[e.index] = mesh.vertices[e.vertices[0]].select ^ mesh.vertices[e.vertices[1]].select
        mesh.edges.foreach_set('select',state_mask)
        bpy.ops.object.mode_set(mode="EDIT")
        return set(["FINISHED"])


#   +--0--+--0--+--0--+          +--0--+--1--+--0--+
#   |     |     |     |          |     |     |     |
#   0     0     0     0          0     1     1     0
#   |     |     |     |          |     |     |     |
#   +--0--+--1--+--0--+   --->   +--0--+--0--+--0--+
#   |     |     |     |          |     |     |     |
#   0     0     0     0          0     1     1     0
#   |     |     |     |          |     |     |     |
#   +--0--+--0--+--0--+          +--0--+--1--+--0--+
class MESH_OT_eneighbors_shared_f(meshpoller,bpy.types.Operator):
    bl_idname = "mesh.e2e_efe"
    bl_label = "Neighbors by Face"
    bl_options = set(["REGISTER","UNDO"])
    def execute(self,context):
        global cachedata
        bpy.ops.object.mode_set(mode="OBJECT")
        obj = context.active_object
        mesh = obj.data
        meshkey = (len(mesh.vertices),len(mesh.edges),len(mesh.polygons),id(self))
        if (meshkey == obj.tkkey) and (meshkey in cachedata):
            edge_to_edges_dict = cachedata
        else:
            edge_key_to_index = dict((k, i) for i,k in enumerate(mesh.edge_keys))
            edge_to_edges_dict = dict((i, set()) for i in xrange(len(mesh.edges)))
            for f in mesh.polygons:
                fed=[edge_key_to_index[k] for k in f.edge_keys]
                for k in f.edge_keys:
                    edge_to_edges_dict[edge_key_to_index[k]].update(fed)
            obj.tkkey = meshkey
        state_mask,esel = (bytearray(meshkey[1]),bytearray(meshkey[1]))
        mesh.edges.foreach_get('select',esel)
        for e in ifilter(lambda _:mesh.edges[_].select,xrange(meshkey[1])):
            for n in edge_to_edges_dict[e]:
                state_mask[n] = 1
        for e in xrange(meshkey[1]):
            esel[e] ^= state_mask[e]
        mesh.edges.foreach_set('select',esel)
        cachedata[meshkey] = edge_to_edges_dict
        bpy.ops.object.mode_set(mode="EDIT")
        return set(["FINISHED"])

# notice that on these next two, the original selection stays
#   +--0--+--0--+--0--+          +--0--+--1--+--0--+
#   |     |     |     |          |     |     |     |
#   0     0     0     0          0     0     0     0
#   |     |     |     |          |     |     |     |
#   +--0--+--1--+--0--+   --->   +--0--+--1--+--0--+
#   |     |     |     |          |     |     |     |
#   0     0     0     0          0     0     0     0
#   |     |     |     |          |     |     |     |
#   +--0--+--0--+--0--+          +--0--+--1--+--0--+
class MESH_OT_eneighbors_shared_f_notv(meshpoller,bpy.types.Operator):
    bl_idname = "mesh.e2e_efnve"
    bl_label = "Lateral Neighbors"
    bl_options = set(["REGISTER","UNDO"])
    def execute(self,context):
        global cachedata
        bpy.ops.object.mode_set(mode="OBJECT")
        obj = context.active_object
        mesh = obj.data
        meshkey = (len(mesh.vertices),len(mesh.edges),len(mesh.polygons),id(self))
        state_mask = bytearray(meshkey[1])
        if (meshkey == obj.tkkey) and (meshkey in cachedata):
            edge_to_face_map,edge_key_to_index = cachedata[meshkey]
        else:
            edge_key_to_index = {}
            edge_to_face_map = dict((i, set()) for i in xrange(meshkey[1]))
            for i,k in enumerate(mesh.edge_keys):
                edge_key_to_index[k] = i
            for f in mesh.polygons:
                for k in f.edge_keys:
                    edge_to_face_map[edge_key_to_index[k]].add(f.index)
            obj.tkkey = meshkey
        selected_edge_indices = ifilter(lambda _:mesh.edges[_].select,xrange(meshkey[1]))
        for e in selected_edge_indices:
            for f in edge_to_face_map[e]:
                for k in mesh.polygons[f].edge_keys:
                    hasv_in = False
                    for v in mesh.edges[e].key:
                        if v in k:
                            hasv_in = True
                    if hasv_in:
                        continue
                    else:
                        state_mask[edge_key_to_index[k]] = True
        for e in ifilter(lambda _:state_mask[_],xrange(meshkey[1])):
            mesh.edges[e].select |= state_mask[e]
        cachedata[meshkey] = (edge_to_face_map,edge_key_to_index)
        bpy.ops.object.mode_set(mode="EDIT")
        return set(["FINISHED"])



#   +--0--+--0--+--0--+          +--0--+--0--+--0--+
#   |     |     |     |          |     |     |     |
#   0     0     0     0          0     0     0     0
#   |     |     |     |          |     |     |     |
#   +--0--+--1--+--0--+   --->   +--1--+--1--+--1--+
#   |     |     |     |          |     |     |     |
#   0     0     0     0          0     0     0     0
#   |     |     |     |          |     |     |     |
#   +--0--+--0--+--0--+          +--0--+--0--+--0--+
class MESH_OT_eneighbors_shared_v_notf(meshpoller,bpy.types.Operator):
    bl_idname = "mesh.e2e_evnfe"
    bl_label = "Longitudinal Edges"
    bl_options = set(["REGISTER","UNDO"])
    def execute(self,context):
        global cachedata
        bpy.ops.object.mode_set(mode="OBJECT")
        obj = context.active_object
        mesh = obj.data
        meshkey = (len(mesh.vertices),len(mesh.edges),len(mesh.polygons),id(self))
        state_mask = bytearray(meshkey[1])
        vstate = bytearray(meshkey[0])
        mesh.vertices.foreach_get('select',vstate)
        if (meshkey == obj.tkkey) and (meshkey in cachedata):
            edge_to_face_map,vert_to_vert_map,edge_key_to_index = cachedata[meshkey]
        else:
            edge_key_to_index = {}
            vert_to_vert_map = dict((i, set()) for i in xrange(meshkey[0]))
            edge_to_face_map = dict((i, set()) for i in xrange(meshkey[1]))
            for i,k in enumerate(mesh.edge_keys):
                edge_key_to_index[k] = i
                vert_to_vert_map[k[0]].add(k[1])
                vert_to_vert_map[k[1]].add(k[0])
            for f in mesh.polygons:
                for k in f.edge_keys:
                    edge_to_face_map[edge_key_to_index[k]].add(f.index)
            obj.tkkey = meshkey
        selected_edge_indices = ifilter(lambda _:mesh.edges[_].select,xrange(meshkey[1]))
        for e in selected_edge_indices:
            for v in mesh.edges[e].key:
                state_mask[v] ^=1
            for f in edge_to_face_map[e]:
                for v in mesh.polygons[f].vertices:
                    vstate[v] = 1
        for v in ifilter(lambda _:state_mask[_],xrange(meshkey[1])):
            for n in vert_to_vert_map[v]:
                if not vstate[n] and (n != v):
                    mesh.edges[edge_key_to_index[(min(v,n),max(v,n))]].select = True
        cachedata[meshkey] = (edge_to_face_map,vert_to_vert_map,edge_key_to_index)
        bpy.ops.object.mode_set(mode="EDIT")
        return set(["FINISHED"])

#deselects faces, leaving only edges selected
class MESH_OT_just_the_edges(meshpoller,bpy.types.Operator):
    bl_idname = "mesh.je"
    bl_label = "Just the Edge Selection"
    bl_options = set(["REGISTER","UNDO"])
    def execute(self,context):
        global cachedata
        bpy.ops.object.mode_set(mode="OBJECT")
        obj = context.active_object
        mesh = obj.data
        meshkey = (len(mesh.vertices),len(mesh.edges),len(mesh.polygons),id(self))
        state_mask = bytearray(meshkey[1])
        if (meshkey == obj.tkkey) and (meshkey in cachedata):
            edge_key_to_index = cachedata[meshkey]
        else:
            edge_key_to_index = dict((k, i) for i,k in enumerate(mesh.edge_keys))
            obj.tkkey = meshkey
        for f in ifilter(lambda _:mesh.polygons[_].select,xrange(meshkey[2])):
            for k in mesh.polygons[f].edge_keys:
                state_mask[edge_key_to_index[k]] = 1
        for e in xrange(meshkey[1]):
            mesh.edges[e].select ^= state_mask[e]
        cachedata[meshkey] = edge_key_to_index
        bpy.ops.object.mode_set(mode="EDIT")
        return set(["FINISHED"])

# deselects edges which are at the edge of a face-selection,
# causing selection to 'shrink in'
class MESH_OT_inner_edges(meshpoller,bpy.types.Operator):
    bl_idname = "mesh.ie"
    bl_label = "Inner Edge Selection"
    bl_options = set(["REGISTER","UNDO"])
    def execute(self,context):
        global cachedata
        bpy.ops.object.mode_set(mode="OBJECT")
        obj = context.active_object
        mesh = obj.data
        meshkey = (len(mesh.vertices),len(mesh.edges),len(mesh.polygons),id(self))
        state_mask = bytearray(meshkey[1])
        if (meshkey == obj.tkkey) and (meshkey in cachedata):
            edge_to_face_map = cachedata[meshkey]
        else:
            edge_key_to_index = dict((k, i) for i,k in enumerate(mesh.edge_keys))
            edge_to_face_map = dict((i, set()) for i in xrange(meshkey[1]))
            for f in mesh.polygons:
                for k in f.edge_keys:
                    edge_to_face_map[edge_key_to_index[k]].add(f.index)
            obj.tkkey = meshkey
        for e in ifilter(lambda _:mesh.edges[_].select,xrange(meshkey[1])):
            for f in edge_to_face_map[e]:
                if mesh.polygons[f].select:
                    state_mask[e] ^=1
        for e in xrange(meshkey[1]):
            mesh.edges[e].select ^= state_mask[e]
        cachedata[meshkey] = edge_to_face_map
        bpy.ops.object.mode_set(mode="EDIT")
        return set(["FINISHED"])


def eemenuitem(self,context):
    self.layout.operator(MESH_OT_eneighbors_shared_v_f.bl_idname)
    self.layout.operator(MESH_OT_eneighbors_shared_v.bl_idname)
    self.layout.operator(MESH_OT_eneighbors_shared_f.bl_idname)
    self.layout.operator(MESH_OT_eneighbors_shared_f_notv.bl_idname)
    self.layout.operator(MESH_OT_eneighbors_shared_v_notf.bl_idname)
    self.layout.operator(MESH_OT_just_the_edges.bl_idname)
    self.layout.operator(MESH_OT_inner_edges.bl_idname)

#END EDGES SECTION
#BEGIN FACES SECTION

# here is another one which functions very similarly to the ctrl+NUMPAD_PLUS 'growth'
# but it deselects the original selection, of course.
# This would be your checkerboard-type growth.
#   [0][0][0]          [0][1][0]
#   [0][1][0]   --->   [1][0][1]
#   [0][0][0]          [0][1][0]
class MESH_OT_fneighbors_shared_e(meshpoller,bpy.types.Operator):
    bl_idname = "mesh.f2f_fef"
    bl_label = "Neighbors by Edge"
    bl_options = set(["REGISTER","UNDO"])
    def execute(self,context):
        global cachedata
        bpy.ops.object.mode_set(mode="OBJECT")
        obj = context.active_object
        mesh = obj.data
        meshkey = (len(mesh.vertices),len(mesh.edges),len(mesh.polygons),id(self))
        if (meshkey == obj.tkkey) and (meshkey in cachedata):
            face_to_face_map = cachedata[meshkey]
        else:
            edge_key_to_index = dict((k, i) for i,k in enumerate(mesh.edge_keys))
            edge_to_face_map = dict((i, set()) for i in xrange(meshkey[1]))
            for f in mesh.polygons:
                for k in f.edge_keys:
                    edge_to_face_map[edge_key_to_index[k]].add(f.index)
            face_to_face_map = dict((i, set()) for i in xrange(meshkey[2]))
            for f in mesh.polygons:
                for k in f.edge_keys:
                    face_to_face_map[f.index].update(edge_to_face_map[edge_key_to_index[k]])
            obj.tkkey = meshkey
        mask_state = bytearray(meshkey[2])
        for f in ifilter(lambda _:mesh.polygons[_].select,xrange(meshkey[2])):
            for n in face_to_face_map[f]:
                mask_state[n] = True
        for f in xrange(meshkey[2]):
            mesh.polygons[f].select ^= mask_state[f]
        cachedata[meshkey] = face_to_face_map
        bpy.ops.object.mode_set(mode="EDIT")
        return set(["FINISHED"])


#   [0][0][0]          [1][0][1]
#   [0][1][0]   --->   [0][0][0]
#   [0][0][0]          [1][0][1]
class MESH_OT_fneighbors_shared_v_note(meshpoller,bpy.types.Operator):
    bl_idname = "mesh.f2f_fvnef"
    bl_label = "Neighbors by Vert not Edge"
    bl_options = set(["REGISTER","UNDO"])
    def execute(self,context):
        global cachedata
        bpy.ops.object.mode_set(mode="OBJECT")
        obj = context.active_object
        mesh = obj.data
        meshkey = (len(mesh.vertices),len(mesh.edges),len(mesh.polygons),id(self))
        if (meshkey == obj.tkkey) and (meshkey in cachedata):
            edge_key_to_index = cachedata[meshkey]
        else:
            edge_key_to_index = dict((k, i) for i,k in enumerate(mesh.edge_keys))
            obj.tkkey = meshkey
        state_mask = bytearray(meshkey[2])
        face_verts = set()
        for f in ifilter(lambda _:mesh.polygons[_].select,xrange(meshkey[2])):
            face_verts.update(mesh.polygons[f].vertices)
        for f in ifilter(lambda _:not mesh.polygons[_].select,xrange(meshkey[2])):
            ct = 0
            for v in mesh.polygons[f].vertices:
                ct += (v in face_verts)
            if ct == 1:
                state_mask[f] = 1
        mesh.polygons.foreach_set('select',state_mask)
        cachedata[meshkey] = edge_key_to_index
        bpy.ops.object.mode_set(mode="EDIT")
        return set(["FINISHED"])


# http://en.wikipedia.org/wiki/Conway's_Game_of_Life
class MESH_OT_conway(meshpoller,bpy.types.Operator):
    bl_idname = "mesh.conway"
    bl_label = "Conway"
    bl_options = set(["REGISTER","UNDO"])
    def execute(self,context):
        global cachedata
        bpy.ops.object.mode_set(mode="OBJECT")
        obj = context.active_object
        mesh = obj.data
        meshkey = (len(mesh.vertices),len(mesh.edges),len(mesh.polygons),id(self))
        if (meshkey == obj.tkkey) and (meshkey in cachedata):
            vert_to_face_map = cachedata[meshkey]
        else:
            vert_to_face_map = dict((i, set()) for i in xrange(meshkey[0]))
            for f in mesh.polygons:
                for v in f.vertices:
                    vert_to_face_map[v].add(f.index)
            obj.tkkey = meshkey
        sel = set()
        uns = set()
        F = dict((i, set()) for i in xrange(meshkey[2]))
        for f in xrange(meshkey[2]):
            for v in mesh.polygons[f].vertices:
                for n in ifilter(lambda _: mesh.polygons[_].select and (_ != f),vert_to_face_map[v]):
                    F[f].add(n)
        for f in F:
            if len(F[f]) == 3:
                sel.add(f)
            elif len(F[f]) != 2:
                uns.add(f)
        for f in xrange(meshkey[2]):
            if f in sel:
                mesh.polygons[f].select = True
            if f in uns:
                mesh.polygons[f].select = False
        cachedata[meshkey] = vert_to_face_map
        bpy.ops.object.mode_set(mode="EDIT")
        return set(["FINISHED"])



def ffmenuitem(self,context):
    self.layout.operator(MESH_OT_fneighbors_shared_e.bl_idname)
    self.layout.operator(MESH_OT_fneighbors_shared_v_note.bl_idname)
    self.layout.operator(MESH_OT_conway.bl_idname)

def register():
    bpy.utils.register_module(__name__)
    bpy.types.VIEW3D_MT_edit_mesh_vertices.append(vvmenuitem)
    bpy.types.VIEW3D_MT_edit_mesh_edges.append(eemenuitem)
    bpy.types.VIEW3D_MT_edit_mesh_faces.append(ffmenuitem)

def unregister():
    bpy.utils.unregister_module(__name__)
    bpy.types.VIEW3D_MT_edit_mesh_vertices.remove(vvmenuitem)
    bpy.types.VIEW3D_MT_edit_mesh_edges.remove(eemenuitem)
    bpy.types.VIEW3D_MT_edit_mesh_faces.remove(ffmenuitem)

if __name__ == "__main__":
    register()
