# -*- coding: utf-8 -*-

# ***** BEGIN GPL LICENSE BLOCK *****
#
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software Foundation,
# Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# ***** END GPL LICENCE BLOCK *****
# based completely on addon by zmj100
# added some distance limits to prevent overlap - max12345
# ------ ------
from __future__ import division
from __future__ import absolute_import
import bpy
import bmesh
from bpy.props import FloatProperty, IntProperty, BoolProperty, EnumProperty
from math import tan, cos, degrees, radians, sin
from mathutils import Matrix

# ------ ------
def edit_mode_out():
    bpy.ops.object.mode_set(mode = 'OBJECT')

def edit_mode_in():
    bpy.ops.object.mode_set(mode = 'EDIT')

def angle_rotation(rp,q,axis,angle):
    """returns the vector made by the rotation of the vector q - rp by angle around axis and then adds rp"""
    return (Matrix.Rotation(angle, 3, axis) * (q - rp)) + rp

# ------ ------
def face_inset_fillet(bme, face_index_list, inset_amount, distance, number_of_sides, out, radius, type_enum, kp):

    list_del = []

    for faceindex in face_index_list:

        bme.faces.ensure_lookup_table()
        #loops through the faces...
        f = bme.faces[faceindex]
        f.select_set(0)
        list_del.append(f)
        f.normal_update()
        vertex_index_list = [v.index for v in f.verts]
        dict_0 = {}
        orientation_vertex_list = []
        n = len(vertex_index_list)
        for i in xrange(n):
            #loops through the vertices
            dict_0[i] = []
            bme.verts.ensure_lookup_table()
            p  = (bme.verts[ vertex_index_list[i] ].co).copy()
            p1 = (bme.verts[ vertex_index_list[(i - 1) % n] ].co).copy()
            p2 = (bme.verts[ vertex_index_list[(i + 1) % n] ].co).copy()
            #copies some vert coordinates, always the 3 around i
            dict_0[i].append(bme.verts[vertex_index_list[i]])
            #appends the bmesh vert of the appropriate index to the dict
            vec1 = p - p1
            vec2 = p - p2
            #vectors for the other corner points to the cornerpoint
            #corresponding to i/p
            angle = vec1.angle(vec2)
            
            adj = inset_amount / tan(angle * 0.5)
            h = (adj ** 2 + inset_amount ** 2) ** 0.5
            if round(degrees(angle)) == 180 or round(degrees(angle)) == 0.0:
                #if they corer is a straight line...
                #I think this creates some new points...
                if out == True:
                    val=((f.normal).normalized() * inset_amount)
                else:
                    val=-((f.normal).normalized() * inset_amount)
                p6 = angle_rotation(p,p + val,vec1,radians(90))                
            else:
                #if the corner is an actual corner
                val=((f.normal).normalized() * h)
                if out==True:
                    #this shit -(p - (vec2.normalized() * adj))) is just the freaking axis afaik...
                    p6 = angle_rotation(p,p + val, -(p - (vec2.normalized() * adj)),-radians(90))
                else:
                    p6 = angle_rotation(p,p - val,((p - (vec1.normalized() * adj)) - (p - (vec2.normalized() * adj))),
                    -radians(90))
                    
                orientation_vertex_list.append(p6)

        new_inner_face = []
        orientation_vertex_list_length = len(orientation_vertex_list)
        ovll=orientation_vertex_list_length
        for j in xrange(ovll):
            q = orientation_vertex_list[j]
            q1 = orientation_vertex_list[(j - 1) % ovll]
            q2 = orientation_vertex_list[(j + 1) % ovll]
            #again, these are just vectors between somewhat displaced conernervertices
            vec1_ = q - q1
            vec2_ = q - q2
            ang_ = vec1_.angle(vec2_)
            #the angle between them
            if round(degrees(ang_)) == 180 or round(degrees(ang_)) == 0.0:
                #again... if it's really a line...
                v = bme.verts.new(q)
                new_inner_face.append(v)
                dict_0[j].append(v)
            else:
                #s.a.
                
                if radius == False:
                    h_ = distance * (1 / cos(ang_ * 0.5))
                    d = distance
                elif radius == True:
                    h_ = distance / sin(ang_ * 0.5)
                    d = distance / tan(ang_ * 0.5)
                #max(d) is vec1_.magnitude*0.5
                #or vec2_.magnitude*0.5 respectively
                
                #only functional difference v
                
                if d >vec1_.magnitude*0.5:
                    d=vec1_.magnitude*0.5
                if d >vec2_.magnitude*0.5:
                    d=vec2_.magnitude*0.5
                    
                #only functional difference ^ 
                    
                q3 = q - (vec1_.normalized() * d)
                q4 = q - (vec2_.normalized() * d)
                #these are new verts somewhat offset from the coners
                rp_ = q - ((q - ((q3 + q4) * 0.5)).normalized() * h_)
                #reference point inside the curvature
                axis_ = vec1_.cross(vec2_)
                #this should really be just the face normal
                vec3_ = rp_ - q3
                vec4_ = rp_ - q4
                rot_ang = vec3_.angle(vec4_)
                cornerverts = []
                
                for o in xrange(number_of_sides + 1):
                    
                    #this calculates the actual new vertices
                    
                    q5 = angle_rotation(rp_,q4,axis_,rot_ang * o / number_of_sides)
                    v = bme.verts.new(q5)
                    
                    #creates new bmesh vertices from it
                    bme.verts.index_update()

                    dict_0[j].append(v)
                    cornerverts.append(v)
                    
                cornerverts.reverse()
                new_inner_face.extend(cornerverts)

        if out == False:
            f = bme.faces.new(new_inner_face)
            f.select_set(True)
        elif out == True and kp == True:
            f = bme.faces.new(new_inner_face)
            f.select_set(True)

        n2_ = len(dict_0)
        #these are the new side faces, those that don't depend on cornertype
        for o in xrange(n2_):
            list_a = dict_0[o]
            list_b = dict_0[(o + 1) % n2_]
            bme.faces.new( [ list_a[0], list_b[0], list_b[-1], list_a[1] ] )
            bme.faces.index_update()
        #cornertype 1 - ngon faces
        if type_enum == 'opt0':
            for k in dict_0:
                if len(dict_0[k]) > 2:
                    bme.faces.new(dict_0[k])
                    bme.faces.index_update()
        #cornertype 2 - triangulated faces
        if type_enum == 'opt1':
            for k_ in dict_0:
                q_ = dict_0[k_][0]
                dict_0[k_].pop(0)
                n3_ = len(dict_0[k_])
                for kk in xrange(n3_ - 1):
                    bme.faces.new( [ dict_0[k_][kk], dict_0[k_][(kk + 1) % n3_], q_ ] )
                    bme.faces.index_update()


    del_ = [bme.faces.remove(f) for f in list_del]
    del del_

# ------ operator 0 ------
class faceinfillet_op0(bpy.types.Operator):
    bl_idname = 'faceinfillet.op0_id'
    bl_label = 'Face Inset Fillet'
    bl_description = 'inset selected faces'
    bl_options = set(['REGISTER', 'UNDO'])

    inset_amount = FloatProperty( name = '', default = 0.04, min = 0, max = 100.0, step = 1, precision = 3 )      # inset amount
    number_of_sides = IntProperty( name = '', default = 4, min = 1, max = 100, step = 1 )      # number of sides
    distance = FloatProperty( name = '', default = 0.04, min = 0.00001, max = 100.0, step = 1, precision = 3 )
    out = BoolProperty( name = 'Out', default = False )
    radius = BoolProperty( name = 'Radius', default = False )
    type_enum = EnumProperty( items =( ('opt0', 'Type 1', ''), ('opt1', 'Type 2', '') ), name = '', default = 'opt0' )
    kp = BoolProperty( name = 'Keep face', default = False )
    
    def draw(self, context):
        layout = self.layout
        box = layout.box()
        box.prop(self, 'type_enum', text = 'Corner type')
        row0 = box.row(align = True)
        row0.prop(self, 'out')
        if self.out == True:
            row0.prop(self, 'kp')
        row = box.split(0.40, align = True)
        row.label('Inset amount:')
        row.prop(self, 'inset_amount')
        row1 = box.split(0.60, align = True)
        row1.label('Number of sides:')
        row1.prop(self, 'number_of_sides', slider = True)
        box.prop(self, 'radius')
        row2 = box.split(0.40, align = True)
        if self.radius == True:
            row2.label('Radius:')
        else:
            row2.label('distance:')
        row2.prop(self, 'distance')

    def execute(self, context):
        #this really just prepares everything for the main function
        inset_amount = self.inset_amount
        number_of_sides = self.number_of_sides
        distance = self.distance
        out = self.out
        radius = self.radius
        type_enum = self.type_enum
        kp = self.kp

        edit_mode_out()
        ob_act = context.active_object
        bme = bmesh.new()
        bme.from_mesh(ob_act.data)
        #this 
        face_index_list = [ f.index for f in bme.faces if f.select and f.is_valid ]

        if len(face_index_list) == 0:
            self.report(set(['INFO']), 'No faces selected unable to continue.')
            edit_mode_in()
            return set(['CANCELLED'])
        elif len(face_index_list) != 0:
            face_inset_fillet(bme, face_index_list, inset_amount, distance, number_of_sides, out, radius, type_enum, kp)

        bme.to_mesh(ob_act.data)
        edit_mode_in()
        return set(['FINISHED'])

class inset_help(bpy.types.Operator):
    bl_idname = 'help.face_inset'
    bl_label = ''

    def draw(self, context):
        layout = self.layout
        layout.label('To use:')
        layout.label('Select a face or faces & inset.')
        layout.label('Inset square, circle or outside.')
        layout.label('To Help:')
        layout.label('Circle: use remove doubles to tidy joins.')
        layout.label('Outset: select & use normals flip before extruding.')
    
    def execute(self, context):
        return set(['FINISHED'])

    def invoke(self, context, event):
        return context.window_manager.invoke_popup(self, width = 350)
