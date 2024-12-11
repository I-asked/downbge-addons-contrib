from __future__ import division
from __future__ import absolute_import
import bpy
import bmesh

from . import Math
from . import Curves



class LoftedSplineSurface(object):
    def __init__(self, activeSpline, otherSpline, bMesh, vert0Index, resolution):
        self.splineA = activeSpline
        self.splineO = otherSpline
        
        self.bMesh = bMesh
        self.vert0Index = vert0Index
        self.resolution = resolution

        
    def Apply(self, worldMatrixA, worldMatrixO):
        #deltaPar = 1.0 / float(self.resolution - 1)
        
        par = 0.0
        pointA = worldMatrixA * self.splineA.CalcPoint(par)
        pointO = worldMatrixO * self.splineO.CalcPoint(par)
        self.bMesh.verts[self.vert0Index].co = pointA
        self.bMesh.verts[self.vert0Index + 1].co = pointO
        
        fltResm1 = float(self.resolution - 1)
        for i in xrange(1, self.resolution):
            par = float(i) / fltResm1
        
            pointA = worldMatrixA * self.splineA.CalcPoint(par)
            pointO = worldMatrixO * self.splineO.CalcPoint(par)
            self.bMesh.verts[self.vert0Index + 2 * i].co = pointA
            self.bMesh.verts[self.vert0Index + 2 * i + 1].co = pointO

        
    def AddFaces(self):
        currIndexA = self.vert0Index
        currIndexO = self.vert0Index + 1
        
        bmVerts = self.bMesh.verts
        bmVerts.ensure_lookup_table()

        for i in xrange(1, self.resolution):
            nextIndexA = self.vert0Index + 2 * i
            nextIndexO = nextIndexA + 1
            
            self.bMesh.faces.new([bmVerts[currIndexA], bmVerts[currIndexO], bmVerts[nextIndexO], bmVerts[nextIndexA]])
            
            currIndexA = nextIndexA
            currIndexO = nextIndexO
            

class LoftedSurface(object):
    @staticmethod
    def FromSelection():
        selObjects = bpy.context.selected_objects
        if len(selObjects) != 2: raise Exception("len(selObjects) != 2") # shouldn't be possible
        
        blenderActiveCurve = bpy.context.active_object
        blenderOtherCurve = selObjects[0]
        if blenderActiveCurve == blenderOtherCurve: blenderOtherCurve = selObjects[1]
        
        aCurve = Curves.Curve(blenderActiveCurve)
        oCurve = Curves.Curve(blenderOtherCurve)
        
        name = "TODO: autoname"
        
        return LoftedSurface(aCurve, oCurve, name)
    

    def __init__(self, activeCurve, otherCurve, name = "LoftedSurface"):
        self.curveA = activeCurve
        self.curveO = otherCurve
        self.name  = name
        
        self.nrSplines = self.curveA.nrSplines
        if self.curveO.nrSplines < self.nrSplines: self.nrSplines = self.curveO.nrSplines
        
        self.bMesh = bmesh.new()
        
        self.splineSurfaces = self.SetupSplineSurfaces()
        
        self.Apply()
        
        
    def SetupSplineSurfaces(self):
        rvSplineSurfaces = []
        
        currV0Index = 0
        for i in xrange(self.nrSplines):
            splineA = self.curveA.splines[i]
            splineO = self.curveO.splines[i]
            
            res = splineA.resolution
            if splineO.resolution < res: res = splineO.resolution
            
            for iv in xrange(2 * res): self.bMesh.verts.new()
            
            splSurf = LoftedSplineSurface(splineA, splineO, self.bMesh, currV0Index, res)
            splSurf.AddFaces()
            rvSplineSurfaces.append(splSurf)
            
            currV0Index += 2 * res
        
        return rvSplineSurfaces
        
        
    def Apply(self):
        for splineSurface in self.splineSurfaces: splineSurface.Apply(self.curveA.worldMatrix, self.curveO.worldMatrix)
        
        
    def AddToScene(self):
        mesh = bpy.data.meshes.new("Mesh" + self.name)
        
        self.bMesh.to_mesh(mesh)
        mesh.update()
        
        meshObject = bpy.data.objects.new(self.name, mesh)
        
        bpy.context.scene.objects.link(meshObject)



# active spline is swept over other spline (rail)
class SweptSplineSurface(object):
    def __init__(self, activeSpline, otherSpline, bMesh, vert0Index, resolutionA, resolutionO):
        self.splineA = activeSpline
        self.splineO = otherSpline
        
        self.bMesh = bMesh
        self.vert0Index = vert0Index
        self.resolutionA = resolutionA
        self.resolutionO = resolutionO

        
    def Apply(self, worldMatrixA, worldMatrixO):
        localPointsA = []
        fltResAm1 = float(self.resolutionA - 1)
        for i in xrange(self.resolutionA):
            par = float(i) / fltResAm1
            pointA = self.splineA.CalcPoint(par)
            localPointsA.append(pointA)

        
        worldPointsO = []
        localDerivativesO = []
        fltResOm1 = float(self.resolutionO - 1)
        for i in xrange(self.resolutionO):
            par = float(i) / fltResOm1
            
            pointO = self.splineO.CalcPoint(par)
            worldPointsO.append(worldMatrixO * pointO)
            
            derivativeO = self.splineO.CalcDerivative(par)
            localDerivativesO.append(derivativeO)
        
        
        currWorldMatrixA = worldMatrixA
        worldMatrixOInv = worldMatrixO.inverted()
        prevDerivativeO = localDerivativesO[0]
        for iO in xrange(self.resolutionO):
            currDerivativeO = localDerivativesO[iO]
            localRotMatO = Math.CalcRotationMatrix(prevDerivativeO, currDerivativeO)
            
            currLocalAToLocalO = worldMatrixOInv * currWorldMatrixA       
            worldPointsA = []
            for iA in xrange(self.resolutionA):
                pointALocalToO = currLocalAToLocalO * localPointsA[iA]
                rotatedPointA = localRotMatO * pointALocalToO
                worldPointsA.append(worldMatrixO * rotatedPointA)
                
            worldOffsetsA = []
            worldPoint0A = worldPointsA[0]
            for i in xrange(self.resolutionA): worldOffsetsA.append(worldPointsA[i] - worldPoint0A)
            
            
            for iA in xrange(self.resolutionA):
                iVert = self.vert0Index + (self.resolutionA * iO) + iA
                currVert = worldPointsO[iO] + worldOffsetsA[iA]
                self.bMesh.verts[iVert].co = currVert
                
            prevDerivativeO = currDerivativeO
            currWorldMatrixA = worldMatrixO * localRotMatO * currLocalAToLocalO

        
    def AddFaces(self):
        bmVerts = self.bMesh.verts
        bmVerts.ensure_lookup_table()
        
        for iO in xrange(self.resolutionO - 1):
            for iA in xrange(self.resolutionA - 1):
                currIndexA1 = self.vert0Index + (self.resolutionA * iO) + iA
                currIndexA2 = currIndexA1 + 1
                nextIndexA1 = self.vert0Index + (self.resolutionA * (iO + 1)) + iA
                nextIndexA2 = nextIndexA1 + 1
        
                self.bMesh.faces.new([bmVerts[currIndexA1], bmVerts[currIndexA2], bmVerts[nextIndexA2], bmVerts[nextIndexA1]])
        
        

class SweptSurface(object):
    @staticmethod
    def FromSelection():
        selObjects = bpy.context.selected_objects
        if len(selObjects) != 2: raise Exception("len(selObjects) != 2") # shouldn't be possible
        
        blenderActiveCurve = bpy.context.active_object
        blenderOtherCurve = selObjects[0]
        if blenderActiveCurve == blenderOtherCurve: blenderOtherCurve = selObjects[1]
        
        aCurve = Curves.Curve(blenderActiveCurve)
        oCurve = Curves.Curve(blenderOtherCurve)
        
        name = "TODO: autoname"
        
        return SweptSurface(aCurve, oCurve, name)
    

    def __init__(self, activeCurve, otherCurve, name = "SweptSurface"):
        self.curveA = activeCurve
        self.curveO = otherCurve
        self.name  = name
        
        self.nrSplines = self.curveA.nrSplines
        if self.curveO.nrSplines < self.nrSplines: self.nrSplines = self.curveO.nrSplines
        
        self.bMesh = bmesh.new()
        
        self.splineSurfaces = self.SetupSplineSurfaces()
        
        self.Apply()
        
        
    def SetupSplineSurfaces(self):
        rvSplineSurfaces = []
        
        currV0Index = 0
        for i in xrange(self.nrSplines):
            splineA = self.curveA.splines[i]
            splineO = self.curveO.splines[i]
            
            resA = splineA.resolution
            resO = splineO.resolution
            
            for iv in xrange(resA * resO): self.bMesh.verts.new()
            
            splSurf = SweptSplineSurface(splineA, splineO, self.bMesh, currV0Index, resA, resO)
            splSurf.AddFaces()
            rvSplineSurfaces.append(splSurf)
            
            currV0Index += resA * resO
        
        return rvSplineSurfaces
        
        
    def Apply(self):
        for splineSurface in self.splineSurfaces: splineSurface.Apply(self.curveA.worldMatrix, self.curveO.worldMatrix)
        
        
    def AddToScene(self):
        mesh = bpy.data.meshes.new("Mesh" + self.name)
        
        self.bMesh.to_mesh(mesh)
        mesh.update()
        
        meshObject = bpy.data.objects.new(self.name, mesh)
        
        bpy.context.scene.objects.link(meshObject)
        


# profileSpline is swept over rail1Spline and scaled/rotated to have its endpoint on rail2Spline
class BirailedSplineSurface(object):
    def __init__(self, rail1Spline, rail2Spline, profileSpline, bMesh, vert0Index, resolutionRails, resolutionProfile):
        self.rail1Spline = rail1Spline
        self.rail2Spline = rail2Spline
        self.profileSpline = profileSpline
        
        self.bMesh = bMesh
        self.vert0Index = vert0Index
        self.resolutionRails = resolutionRails
        self.resolutionProfile = resolutionProfile

        
    def Apply(self, worldMatrixRail1, worldMatrixRail2, worldMatrixProfile):
        localPointsProfile = []
        fltResProfilem1 = float(self.resolutionProfile - 1)
        for i in xrange(self.resolutionProfile):
            par = float(i) / fltResProfilem1
            pointProfile = self.profileSpline.CalcPoint(par)
            localPointsProfile.append(pointProfile)

        
        worldPointsRail1 = []
        localDerivativesRail1 = []
        worldPointsRail2 = []
        fltResRailsm1 = float(self.resolutionRails - 1)
        for i in xrange(self.resolutionRails):
            par = float(i) / fltResRailsm1
            
            pointRail1 = self.rail1Spline.CalcPoint(par)
            worldPointsRail1.append(worldMatrixRail1 * pointRail1)
            
            derivativeRail1 = self.rail1Spline.CalcDerivative(par)
            localDerivativesRail1.append(derivativeRail1)
            
            pointRail2 = self.rail2Spline.CalcPoint(par)
            worldPointsRail2.append(worldMatrixRail2 * pointRail2)
        
        
        currWorldMatrixProfile = worldMatrixProfile
        worldMatrixRail1Inv = worldMatrixRail1.inverted()
        prevDerivativeRail1 = localDerivativesRail1[0]
        for iRail in xrange(self.resolutionRails):
            currDerivativeRail1 = localDerivativesRail1[iRail]
            localRotMatRail1 = Math.CalcRotationMatrix(prevDerivativeRail1, currDerivativeRail1)
            
            currLocalProfileToLocalRail1 = worldMatrixRail1Inv * currWorldMatrixProfile       
            worldPointsProfileRail1 = []
            for iProfile in xrange(self.resolutionProfile):
                pointProfileLocalToRail1 = currLocalProfileToLocalRail1 * localPointsProfile[iProfile]
                rotatedPointProfile = localRotMatRail1 * pointProfileLocalToRail1
                worldPointsProfileRail1.append(worldMatrixRail1 * rotatedPointProfile)
                
            worldOffsetsProfileRail1 = []
            worldPoint0ProfileRail1 = worldPointsProfileRail1[0]
            for iProfile in xrange(self.resolutionProfile): worldOffsetsProfileRail1.append(worldPointsProfileRail1[iProfile] - worldPoint0ProfileRail1)
                
            worldStartPointProfileRail1 = worldPointsRail1[iRail]
            worldEndPointProfileRail1 = worldStartPointProfileRail1 + worldOffsetsProfileRail1[-1]
            v3From = worldEndPointProfileRail1 - worldStartPointProfileRail1
            v3To = worldPointsRail2[iRail] - worldStartPointProfileRail1
            scaleFactorRail2 = v3To.magnitude / v3From.magnitude
            rotMatRail2 = Math.CalcRotationMatrix(v3From, v3To)
            
            worldOffsetsProfileRail2 = []
            for iProfile in xrange(self.resolutionProfile):
                offsetProfileRail1 = worldOffsetsProfileRail1[iProfile]
                worldOffsetsProfileRail2.append(rotMatRail2 * (offsetProfileRail1 * scaleFactorRail2))
            
            
            for iProfile in xrange(self.resolutionProfile):
                iVert = self.vert0Index + (self.resolutionProfile * iRail) + iProfile
                currVert = worldPointsRail1[iRail] + worldOffsetsProfileRail2[iProfile]
                self.bMesh.verts[iVert].co = currVert
                
            prevDerivativeRail1 = currDerivativeRail1
            currWorldMatrixProfile = worldMatrixRail1 * localRotMatRail1 * currLocalProfileToLocalRail1

        
    def AddFaces(self):
        bmVerts = self.bMesh.verts
        bmVerts.ensure_lookup_table()
        
        for iRail in xrange(self.resolutionRails - 1):
            for iProfile in xrange(self.resolutionProfile - 1):
                currIndex1 = self.vert0Index + (self.resolutionProfile * iRail) + iProfile
                currIndex2 = currIndex1 + 1
                nextIndex1 = self.vert0Index + (self.resolutionProfile * (iRail + 1)) + iProfile
                nextIndex2 = nextIndex1 + 1
        
                self.bMesh.faces.new([bmVerts[currIndex1], bmVerts[currIndex2], bmVerts[nextIndex2], bmVerts[nextIndex1]])
        
        

class BirailedSurface(object):
    @staticmethod
    def FromSelection():
        nrSelectedObjects = bpy.context.scene.curvetools.NrSelectedObjects
        if nrSelectedObjects != 3: raise Exception("nrSelectedObjects != 3") # shouldn't be possible
        
        
        selectedObjects = bpy.context.scene.curvetools.SelectedObjects
        selectedObjectValues = selectedObjects.values()

        curveName = selectedObjectValues[0].name
        rail1BlenderCurve = None
        try: rail1BlenderCurve = bpy.data.objects[curveName]
        except: rail1BlenderCurve = None
        if rail1BlenderCurve is None: raise Exception("rail1BlenderCurve is None")

        curveName = selectedObjectValues[1].name
        rail2BlenderCurve = None
        try: rail2BlenderCurve = bpy.data.objects[curveName]
        except: rail2BlenderCurve = None
        if rail2BlenderCurve is None: raise Exception("rail2BlenderCurve is None")

        curveName = selectedObjectValues[2].name
        profileBlenderCurve = None
        try: profileBlenderCurve = bpy.data.objects[curveName]
        except: profileBlenderCurve = None
        if profileBlenderCurve is None: raise Exception("profileBlenderCurve is None")
        
        
        rail1Curve = Curves.Curve(rail1BlenderCurve)
        rail2Curve = Curves.Curve(rail2BlenderCurve)
        profileCurve = Curves.Curve(profileBlenderCurve)
        
        name = "TODO: autoname"
        
        return BirailedSurface(rail1Curve, rail2Curve, profileCurve, name)
    

    def __init__(self, rail1Curve, rail2Curve, profileCurve, name = "BirailedSurface"):
        self.rail1Curve = rail1Curve
        self.rail2Curve = rail2Curve
        self.profileCurve = profileCurve
        self.name  = name
        
        self.nrSplines = self.rail1Curve.nrSplines
        if self.rail2Curve.nrSplines < self.nrSplines: self.nrSplines = self.rail2Curve.nrSplines
        if self.profileCurve.nrSplines < self.nrSplines: self.nrSplines = self.profileCurve.nrSplines
        
        self.bMesh = bmesh.new()
        
        self.splineSurfaces = self.SetupSplineSurfaces()
        
        self.Apply()
        
        
    def SetupSplineSurfaces(self):
        rvSplineSurfaces = []
        
        currV0Index = 0
        for i in xrange(self.nrSplines):
            splineRail1 = self.rail1Curve.splines[i]
            splineRail2 = self.rail2Curve.splines[i]
            splineProfile = self.profileCurve.splines[i]
            
            resProfile = splineProfile.resolution
            resRails = splineRail1.resolution
            if splineRail2.resolution < resRails: resRails = splineRail2.resolution
            
            for iv in xrange(resProfile * resRails): self.bMesh.verts.new()
            
            splSurf = BirailedSplineSurface(splineRail1, splineRail2, splineProfile, self.bMesh, currV0Index, resRails, resProfile)
            splSurf.AddFaces()
            rvSplineSurfaces.append(splSurf)
            
            currV0Index += resProfile * resRails
        
        return rvSplineSurfaces
        
        
    def Apply(self):
        for splineSurface in self.splineSurfaces: splineSurface.Apply(self.rail1Curve.worldMatrix, self.rail2Curve.worldMatrix, self.profileCurve.worldMatrix)
        
        
    def AddToScene(self):
        mesh = bpy.data.meshes.new("Mesh" + self.name)
        
        self.bMesh.to_mesh(mesh)
        mesh.update()
        
        meshObject = bpy.data.objects.new(self.name, mesh)
        
        bpy.context.scene.objects.link(meshObject)

    
