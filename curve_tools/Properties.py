from __future__ import absolute_import
import time

import bpy
from bpy.props import *



class CurveTools2SelectedObjectHeader(bpy.types.Header):
    bl_label = "Selection"
    bl_space_type = "VIEW_3D"
    
    def __init__(self):
        self.update()

        
    def update(self):
        blenderSelectedObjects = bpy.context.selected_objects
        selectedObjects = bpy.context.scene.curvetools.SelectedObjects
        
        selectedObjectsToRemove = []
        for selectedObject in selectedObjects:
            if not selectedObject.IsElementOf(blenderSelectedObjects): selectedObjectsToRemove.append(selectedObject)
        for selectedObject in selectedObjectsToRemove: selectedObjects.remove(selectedObject)
        
        blenderObjectsToAdd = []
        for blenderObject in blenderSelectedObjects:
            if not CurveTools2SelectedObject.ListContains(selectedObjects, blenderObject): blenderObjectsToAdd.append(blenderObject)
        for blenderObject in blenderObjectsToAdd:
            newSelectedObject = CurveTools2SelectedObject(blenderObject)
            selectedObjects.append(newSelectedObject)

        
    def draw(self, context):
        selectedObjects = bpy.context.scene.curvetools.SelectedObjects
        nrSelectedObjects = len(selectedObjects)
        
        layout = self.layout
        row = layout.row()
        row.label("Sel:", nrSelectedObjects)


class CurveTools2SelectedObject(bpy.types.PropertyGroup):
    name = StringProperty(name = "name", default = "??")

    
    @staticmethod
    def UpdateThreadTarget(lock, sleepTime, selectedObjectNames, selectedBlenderObjectNames):
        time.sleep(sleepTime)
        
        newSelectedObjectNames = []
        
        for name in selectedObjectNames:
            if name in selectedBlenderObjectNames: newSelectedObjectNames.append(name)
            
        for name in selectedBlenderObjectNames:
            if not (name in selectedObjectNames): newSelectedObjectNames.append(name)
            
        # sometimes it still complains about the context
        try:
            nrNewSelectedObjects = len(newSelectedObjectNames)
            bpy.context.scene.curvetools.NrSelectedObjects = nrNewSelectedObjects
            
            selectedObjects = bpy.context.scene.curvetools.SelectedObjects
            selectedObjects.clear()
            for i in xrange(nrNewSelectedObjects): selectedObjects.add()
            for i, newSelectedObjectName in enumerate(newSelectedObjectNames):
                selectedObjects[i].name = newSelectedObjectName
        except: pass

        
    @staticmethod
    def GetSelectedObjectNames():
        selectedObjects = bpy.context.scene.curvetools.SelectedObjects
        
        rvNames = []
        selectedObjectValues = selectedObjects.values()
        for selectedObject in selectedObjectValues: rvNames.append(selectedObject.name)
        
        return rvNames
        
        
    @staticmethod
    def GetSelectedBlenderObjectNames():
        blenderSelectedObjects = bpy.context.selected_objects
        
        rvNames = []
        for blObject in blenderSelectedObjects: rvNames.append(blObject.name)
        
        return rvNames
        
