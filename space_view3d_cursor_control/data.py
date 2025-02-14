# -*- coding: utf-8 -*-
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



"""
  TODO:

      IDEAS:

      LATER:

      ISSUES:
          Bugs:
          Mites:

      QUESTIONS:


"""



from __future__ import absolute_import
import bpy
import bgl
import math
from mathutils import Vector, Matrix
from mathutils import geometry
from .misc_utils import *
from .constants_utils import *
from .cursor_utils import *
from .ui_utils import *
from .geometry_utils import *


PRECISION = 4


class CursorControlData(bpy.types.PropertyGroup):
    # Step length properties
    stepLengthEnable = bpy.props.BoolProperty(name="Use step length",description="Use step length",default=0)
    stepLengthMode = bpy.props.EnumProperty(items=[
        ("Mode", "Mode", "Mode"),
        ("Absolute", "Absolute", "Absolute"),
        ("Proportional", "Proportional", "Proportional")],
        default="Proportional")
    stepLengthValue = bpy.props.FloatProperty(name="",precision=PRECISION,default=PHI)
    # Property for linex result select...
    linexChoice = bpy.props.IntProperty(name="",default=-1)
    deltaVector = bpy.props.FloatVectorProperty(name="",precision=PRECISION,default=(1,0,0))

    def hideLinexChoice(self):
        self.linexChoice = -1

    def cycleLinexCoice(self,limit):
        qc = self.linexChoice + 1
        if qc<0:
            qc = 1
        if qc>=limit:
            qc = 0
        self.linexChoice = qc
  
    def setCursor(self,v):
        if self.stepLengthEnable:
            c = CursorAccess.getCursor()
            if((Vector(c)-Vector(v)).length>0):
                if self.stepLengthMode=='Absolute':
                    v = Vector(v)-Vector(c)
                    v.normalize()
                    v = v*self.stepLengthValue + Vector(c)
                if self.stepLengthMode=='Proportional':
                    v = (Vector(v)-Vector(c))*self.stepLengthValue + Vector(c)
        CursorAccess.setCursor(Vector(v))
        
    def guiStates(self,context):
        tvs = 0
        tes = 0
        tfs = 0
        edit_mode = False
        obj = context.active_object
        if (context.mode == 'EDIT_MESH'):
            if (obj and obj.type=='MESH' and obj.data):
                tvs = obj.data.total_vert_sel

                tes = obj.data.total_edge_sel
                tfs = obj.data.total_face_sel
                edit_mode = True
        return (tvs, tes, tfs, edit_mode)

    def invertDeltaVector(self):
        self.deltaVector = Vector([0,0,0])-Vector(self.deltaVector)

    def normalizeDeltaVector(self):
        q = Vector(self.deltaVector)
        q.normalize()
        self.deltaVector = q

    def addDeltaVectorToCursor(self):
        c = CursorAccess.getCursor()
        CursorAccess.setCursor(Vector(c)+Vector(self.deltaVector))

    def subDeltaVectorToCursor(self):
        c = CursorAccess.getCursor()
        CursorAccess.setCursor(Vector(c)-Vector(self.deltaVector))
