# vert class and overloading experiments
import bpy
#PKHG>NEEDED? 
import bmesh 
import math
from math import sqrt,acos,pi,sin,cos,atan,tan,fabs
from mathutils import Vector

#PKHG>DBG change the DBG_info and use  extra_DBG_info
DBG_info ={"MeshInfo":False, "StrutMesh":False, "HubMesh":False}

#in de functie load_images_from had ik debug informatie nodig
#def load_images_from(name_list = ["no image"], where = [], p = "\\addons\\stepwise2PKHG\\" ):
 
def extra_DBG_info(name = "value from DBG_info", info_text="default\n", info_obj=None):
    global DBG_info
    DBG_keys = DBG_info.keys()
    if name in DBG_keys:
        if DBG_info[name]:
            print info_text, info_obj

sgn = lambda x : (x>0) - (x<0) #missing signum function in Python
"""
not in Blender 2.7*?
try:
    breakpoint = bpy.types.bp.bp
except:
    pass    
"""

from __future__ import division
from __future__ import absolute_import
from bpy_extras.object_utils import AddObjectHelper, object_data_add
from collections import Counter

'''PKHG not needed?
def find_twice_vert(l1, l2):
    tmp = [el for el in l1]
    tmp.extend(l2)
    tt = Counter(tmp)
    result = max(tt.keys(), key = lambda k:tt[k])
    print("twice give", result)
    return result
'''

def vefm_add_object(selfobj):
    for i in xrange(len(selfobj.verts)):
        selfobj.verts[i].index = i
    v = [el.vector for el in selfobj.verts]
    #PKHG>??? 14jul14 11:00  print("\n************* edges?", selfobj.edges)
    e = [[edge.a.index, edge.b.index] for edge in selfobj.edges]
    #PKHG> OLD??? e = []
    if type(selfobj.faces[0]) == type([]):
#    print("\n=========== selfobj.faces[0]", selfobj.faces[0], type(selfobj.faces[0]))
#PKHG should be a list of vertices, which have an index
        f =  [[v.index for v in  face] for face in selfobj.faces]
    else:
        f =  [[v.index for v in  face.vertices] for face in selfobj.faces]
#PKHG_DBG_25-11    print("dbg 25-11 f list =", f)
#PKHG_DBG_25-11    print("dgb 25-11 v list +", v)
    m = bpy.data.meshes.new(name= selfobj.name)
    m.from_pydata(v, e, f )
    # useful for development when the mesh may be invalid.
    m.validate(verbose = False)
    object_data_add(bpy.context, m, operator = None)    
#???ERROR PKHG in AddSelf    setMaterial(bpy.context.active_object, pkhg_red_color)

#extra test phase


class vertex(object):
    def __init__(self, vec=(0,0,0)): #default x = 0, y = 0, z = 0):        
        self.vector  = Vector(vec)
        self.length = self.vector.length
        self.index = 0
        self.normal = 0
        self.edges = []
        self.faces = []
        self.boundary = 0

    def findlength(self):
         self.length = self.vector.length

    def normalize(self):
        self.findlength()
        if self.length > 0:
            tmp = 1.0/self.length
            self.vector  =  tmp * self.vector 
#            self.x = self.vector[0] #(1.0/self.length)
#            self.y = self.vector[1] #(1.0/self.length)
#            self.z = self.vector[2] #(1.0/self.length)
            self.length = 1.0

    def findnormal(self):
        target = []
        if self.faces[:] == []:
            print "vefm vertex L81 pkhg:*****ERROR**** findnormal has no faces"
            return
        for currentface in self.faces:
            target.append(currentface.normal)
        self.normal = average(target).centroid()
        self.normal.findlength()
        if self.length == 0:
            print "******ERROR*** lenght zero in findnormal, j = (0,1,0) replcaced"
            self.normal = vertex((0,1,0))
        self.normal.normalize()

    def clockwise(self): #PKHG self is a vertex
        if self.boundary:
            start = self.boundarystart() #???PKHG TODO error
        else:
            start = self.faces[0]
#PKHG TODO do understand 21-11 solves starify of a normal tetrahedron
#        start = self.faces[0]     #PKHG TODO see error above!
#        start.docorners() #PKHG???? 
        self.tempedges = []
        self.tempfaces = []
        for i in xrange(len(self.edges)):
            #print("\n----------------------voor breakpunt pkhg in clockwise")
            #breakpoint(locals(), self.index == 0)
            self.tempfaces.append(start)
            for corner in start.corners:
                if corner[0] is not self:
                    pass
                elif corner[0] is self:
                    self.tempedges.append(corner[1])
                    nextedge = corner[2]
            for facey in nextedge.faces:
                if facey is not start:
                    start = facey
                    break
        self.edges = self.tempedges
        self.faces = self.tempfaces

    def boundarystart(self):
        #PKHG not implemented, needed?
        #Noctumsolis: What's it supposed to do?
        pass

#Noctumsolis: add and sub nolonger differ in structure. Replaced str(type()) cast with isinstance().
    def __add__(self, other):
        if isinstance(other, Vector):
            tmp = self.vector + other
        else:
            tmp = self.vector + other.vector
        return vertex(tmp)

    def __sub__(self, other):
        if isinstance(other, Vector):
            tmp = self.vector -  other
        else:
            tmp = self.vector - other.vector
        return vertex(tmp)

    def __mul__(self, other):
        tmp = self.vector * other
        return vertex(tmp)

    def __truediv__(self, other):
        denom = 1.0/other
        tmp = self.vector * denom
        return (tmp)

    def negative(self):
        return vertex(-self.vector)

class crossp(object):
    ##   Takes in two vertices(vectors), returns the cross product.
    def __init__(self, v1, v2):
        self.v1 = v1
        self.v2 = v2
#
    def docrossproduct(self):
        tmp = self.v1.vector.cross(self.v2.vector)
        return vertex(tmp)

class average(object):
    ##   Takes a list of vertices and returns the average. If two verts are passed, returns midpoint.
    def __init__(self, vertlist):
        self.vertlist = vertlist
 
    def centroid(self):
        tmp  =  Vector()
#PKHG avoid emptylist problems        
        divisor = 1.0
        nr_vertices = len(self.vertlist)
        if nr_vertices > 1:
            divisor = 1.0 / len(self.vertlist)
        elif nr_vertices == 0:
            print "\n***WARNING*** empty list in vefm_271.centroid! L180"
        for vert in self.vertlist:
            tmp = tmp + vert.vector
        tmp = tmp * divisor
        return vertex(tmp)

class edge(object):
    def __init__(self, a = 0, b = 0):
        self.a = a
        self.b = b
        self.index = 0
        self.normal = 0
        self.cross = 0
        self.unit = 0
        self.faces = []
        self.vect = 0  #PKHG becomes  b - a
        self.vectb = 0 #PKHG becomes  a - b
#        self.length = 0
        self.boundary = 0
        self.findvect()
#        print("vect len before", self.vect.length)
        self.findlength()
#        print("vect after", self.vect.length)

    def findvect(self):
        self.vect = self.b - self.a
        self.vectb = self.a - self.b

    def findlength(self):
        self.vect.findlength()
        self.vectb.length = self.vect.length

    def findnormal(self):
                
        if self.boundary:
            self.normal = self.faces[0].normal    #average([self.a, self.b]).centroid()
        else:
            self.normal = average([self.faces[0].normal, self.faces[1].normal]).centroid()
        self.normal.normalize()
#     def findother(self, vertindex):
#
#         if vertindex==self.a:
#
#             return self.b
#
#         else:
#             return self.a
##        different classes for 3,4,> sides??

class face(object):
    def __init__(self, vertices=[]):
#PKHG ok good for tri's at least        print("\n ========= vefm L226======dbg face vertices = ", vertices)
        self.vertices = vertices    ## List of vertex instances.
        self.edges=[]               ##   Will be filled with the sides of the face.
        self.boundary = 0           ##   When set will have bool and id of edge concerned.
        self.normal = 0             ##   Face normal found through cross product.
        self.corners=[]
        self.spokes=[]              ## Vectors of the bisecting angles from each corner
                                    ##          to the centre + dotproduct.
        self.index = 0
 
 #dotproduct is misleading name, it is the hook between two vectors!
    def dotproduct(self, v1, v2):
        v1.findlength()
        v2.findlength()
        if v1.length == 0 or v2.length == 0:
            print "\nPKHG warning, =====vefm_271 dotproduct L245====== at least one zero vector 0 used"         
            return 0 # pi * 0.25 #PKHT_TEST04nov pi * 0.25  #45 degrees??? #PKHG???TODO
        dot = v1.vector.dot(v2.vector)
        costheta = dot / (v1.length * v2.length)
        tmp = acos(costheta)
        return tmp
    
    def orderedges(self):
        temp=[]
        finish = len(self.vertices)
        for i in xrange(finish):
            current = self.vertices[i]
            if i==finish-1:
                next = self.vertices[0]
            else:
                next = self.vertices[i+1]
            for edge in face.edges:
                if edge.a==current and edge.b==next:
                    face.clockw.append(edge.vect)
                    face.aclockw.append(edge.vectb)
                    temp.append(edge)
                if edge.b==current and edge.a==next:
                    face.clockw.append(edge.vectb)
                    face.aclockw.append(edge.vect)
                    temp.append(edge)
            for edge in face.edges:
                if edge.a==current and edge.b==next:
                    face.clockw.append(edge.vect)
                    face.aclockw.append(edge.vectb)
                    temp.append(edge)
                if edge.b==current and edge.a==next:
                    face.clockw.append(edge.vectb)
                    face.aclockw.append(edge.vect)
                    temp.append(edge)
            face.vertices = temp
    
    
    def docorners(self):
        ##   This function identifies and stores the vectors coming from each vertex
        ##   allowing easier calculation of cross and dot products.
        finish = len(self.vertices)
        '''
        which = None
        occur1 = None
        occur2 = None
        if finish == 3 and len(self.edges) == 2 :    
            print("\n***ERROR*** only two edges should be three")
      #      return        
            occur1 = [self.vertices.index(self.edges[0].a), self.vertices.index(self.edges[0].b)]
            occur2 = [self.vertices.index(self.edges[1].a), self.vertices.index(self.edges[1].b)]
            #occur2 = [self.edges[1].a.index, self.edges[1].b.index]
            twice = find_twice_vert(occur1, occur2)
            occur1.remove(twice)
            occur2.remove(twice)
            #new_edge = edge(self.vertices[occur1[0]], self.vertices[occur2[0]])
            #self.edges.append(new_edge)
        '''
        for i in xrange(finish):
            current = self.vertices[i]
            if i==finish-1:
                next = self.vertices[0]
            else:
                next = self.vertices[i+1]
            if i==0:
                previous = self.vertices[-1]
            else:
                previous = self.vertices[i-1]
            corner=[current] #PKHG new for each vertex = current
            #corner = current
            rightedge = None
            leftedge = None
            teller = -1
            for edge in self.edges:
        #PKHG?>>> 14jul 12:39
        #        if finish == 3 and len(self.edges) == 2  and i == 2:
        #            return    
                if finish == 3 and len(self.edges) == 2  and i == 2:
                    return    
                teller += 1                                                        
                currentinfo = (current, edge.a, edge.b, edge.a is current, edge.b is current)
                #next and previous are vertex with respect to ith vertex
                if edge.a is current or edge.b is current:    ##  does this edge contain our current vert
                    if edge.a is current:
                        if edge.b is next:
                            rightedge = edge
                            rightvect = edge.vect
                        if edge.b is previous:
                            leftedge = edge
                            leftvect = edge.vect
                    elif edge.b is current:
                        if edge.a is next:
                            rightedge = edge
                            rightvect = edge.vectb
                        if edge.a is previous:
                            leftedge = edge
                            leftvect = edge.vectb
            corner.append(rightedge)
            corner.append(leftedge)
            if rightedge and leftedge:
                '''
                if rightedge:
                    print("rightedge", rightedge.index)
                if leftedge:
                    print( "leftedge", leftedge.index)
                print("corner", corner)
                #'''
                dotty = self.dotproduct(rightvect, leftvect)
                corner.append(dotty)
            self.corners.append(corner)
     
     
    def findnormal(self):
        one = self.corners[1][2]
        two = self.corners[1][1]
        if one.a is self.corners[1][0]:
            one = one.vect
        elif one.b is self.corners[1][0]:
            one = one.vectb
        if two.a is self.corners[1][0]:
            two = two.vect
        elif two.b is self.corners[1][0]:
            two = two.vectb
        self.normal = crossp(one, two).docrossproduct()
        self.normal.findlength()
        self.normal.normalize()

    def dospokes(self):
#PKHG_OK_24-11        print("\n============vefm L375==============dbg, dospokes called corners =", self.corners[:])
        for corner in self.corners:
            vert = corner[0]
            right = corner[1]
            left = corner[2]
            if right.a is vert:
                one = vertex(right.vect.vector)
            elif right.b is vert:
                one = vertex(right.vectb.vector)
            if left.a is vert:
                two = vertex(left.vect.vector)
            elif left.b is vert:
                two = vertex(left.vectb.vector)
            
            one.normalize()
            two.normalize()
            spoke = one+two
            spoke.normalize()
            self.spokes.append(spoke)

    def artspokes(self):
        centre = average(self.vertices).centroid()
        for point in self.vertices:
            newedge = edge(point, centre)
            spokes.append(newedge)
            
class mesh(object):
    def __init__(self , name="GD_mesh"):
        self.name = name #pkhg test phase at least ;-)
        self.verts=[]
        self.edges=[]
        self.faces=[]
        self.edgeflag = 0
        self.faceflag = 0
        self.vertexflag = 0
        self.vertedgeflag = 0
        self.vertfaceflag = 0
        self.faceedgeflag = 0
        self.boundaryflag = 0
        self.vertnormalflag = 0
        self.edgenormalflag = 0
        self.facenormalflag = 0
        #found in geodesic.py ??? needed for test here!
        self.a45 = pi * 0.25
        self.a90 = pi * 0.5
        self.a180 = pi
        self.a270 = pi * 1.5
        self.a360 = pi * 2        

        
    def power(self, a, b):         ## Returns a power, including negative numbers
        result = sgn(a)*(abs(a)**b)
        return result

    def sign(self, d):    ## Works out the sign of a number.
        return sgn(d)

    def ellipsecomp(self, efactor, theta):
        if theta==self.a90:
            result = self.a90
        elif theta==self.a180:
            result = self.a180
        elif theta==self.a270:
            result = self.a270
        elif theta==self.a360:
            result = 0.0
        else:                        
            result = atan(tan(theta)/efactor**0.5)
            if result<0.0:
                if theta>self.a180:
                    result = result+self.a180
                elif theta<self.a180:
                    result = result+self.a180
    ##  Fishy - check this.
            if result>0.0:
                if theta>self.a180:
                    result = result+self.a180
                elif theta<self.a180:
                    result = result
        return result
        
    def connectivity(self):
        self.dovertedge()
        self.dovertface()
        self.dofaceedge()
        self.boundary()

    def superell(self, n1, uv, turn):
        t1 = sin(uv+turn)
        t1 = abs(t1)
        t1 = t1**n1
        t2 = cos(uv+turn)
        t2 = abs(t2)
        t2 = t2**n1
        r = self.power(1.0/(t1+t2),(1.0/n1))
        return r

    def superform(self, m, n1, n2, n3, uv, a, b, twist):
        t1 = cos(m*(uv+twist)*.25)*a
        t1 = abs(t1)
        t1 = t1**n2
        t2 = sin(m*(uv+twist)*.25)*b
        t2 = abs(t2)
        t2 = t2**n3
        r = self.power(1.0/(t1+t2), n1)        
        return r
        
    def dovertedge(self):
        if not self.vertedgeflag:
            for vert in self.verts:
                vert.edges = []
            for currentedge in self.edges:
                currentedge.a.edges.append(currentedge)
                currentedge.b.edges.append(currentedge)
        self.vertedgeflag = 1
        
    def dovertface(self):
        if not self.vertfaceflag:
            for vert in self.verts:
                vert.faces = []
            for face in self.faces:
                for vert in face.vertices:
                    vert.faces.append(face)
        self.vertfaceflag = 1
        
    def dofaceedge(self):
        self.dovertedge()    ## just in case they haven't been done
        self.dovertface()    ##
        if not self.faceedgeflag:
            for edge in self.edges:
                edge.faces=[]
            for face in self.faces:
                face.edges = []
            for face in self.faces:
                finish = len(face.vertices)
                for i in xrange(finish):
                    current = face.vertices[i]
                    if i == finish-1:
                        next = face.vertices[0]
                    else:
                        next = face.vertices[i+1]
                    for edge in current.edges:
                        if edge.a is current or edge.b is current:
                            if edge.b is next or edge.a is next:
                                edge.faces.append(face)
                                face.edges.append(edge)
        self.faceedgeflag = 1
 
    def boundary(self):
        if not self.boundaryflag:
            for edge in self.edges:
                if len(edge.faces) < 2:
                    edge.boundary = 1
                    edge.faces[0].boundary = 1
                    edge.a.boundary = 1
                    edge.b.boundary = 1
                    
##   The functions below turn the basic triangular faces into
##   hexagonal faces, creating the buckyball effect.
#PKHG seems to work only for meshes with tri's ;-) ??!!
    def hexify(self):
        self.hexverts=[]
        self.hexedges=[]
        self.hexfaces=[]
        #PKHG renumbering the index of the verts
        for i in xrange(len(self.verts)):
            self.verts[i].index = i
        #PKHG renumbering the index of the edges
        for i in xrange(len(self.edges)):
            self.edges[i].index = i
        #PKHG  self=> dovertedge, dovertface, dofaceedge, boundary()
        self.connectivity()
        hexvert_counter = 0
        for edge in self.edges:
#            print("21-11 >>>>>>>>>>>>>>dbg hexify L552")
#            breakpoint(locals(), True)
            self.hexshorten(edge, hexvert_counter)
            hexvert_counter += 2 #PKHG two new vertices done
#PKHG 21-11            print("21-11 <<<<<<<<<<<<<<< na hexshorten L557", hexvert_counter)
#PKHG 21-11            breakpoint(locals(), True)

        for face in self.faces:
            self.makehexfaces(face)
        
        for vert in self.verts:
            vert.clockwise()
            self.hexvertface(vert)
        self.verts = self.hexverts
        self.edges = self.hexedges
        self.faces = self.hexfaces
        self.vertedgeflag = 0
        self.vertfaceflag = 0
        self.faceedgeflag = 0
#PKHG_DBG        print("\n ==========================self hexified I hope")
        #breakpoint(locals(), True)
 
    def hexshorten(self, currentedge, hexvert_counter):
        third = vertex(currentedge.vect/3.0)
        newvert1 = vertex(currentedge.a.vector)
        newvert2 = vertex(currentedge.b.vector)
        newvert1 = newvert1 + third
        newvert1.index = hexvert_counter
        newvert2 = newvert2 - third
        newvert2.index = hexvert_counter + 1 #PKHG caller adjusts +=2 
        newedge = edge(newvert1, newvert2)
        newedge.index = currentedge.index
        self.hexverts.append(newvert1)
        self.hexverts.append(newvert2)
        self.hexedges.append(newedge)
 
    def makehexfaces(self, currentface):
        vertices=[]
        currentface.docorners()
        for corner in currentface.corners:
            vert = corner[0]
            rightedge = corner[1]
            leftedge = corner[2]
            lid = leftedge.index
            rid = rightedge.index
            
            if leftedge.a is vert:
                vertices.append(self.hexedges[lid].a)
            elif leftedge.b is vert:
                vertices.append(self.hexedges[lid].b)
                
            if rightedge.a is vert:
                vertices.append(self.hexedges[rid].a)
            elif rightedge.b is vert:
                vertices.append(self.hexedges[rid].b)
                
        newface = face(vertices)
        newedge1 = edge(vertices[0], vertices[1])
        newedge2 = edge(vertices[2], vertices[3])
        newedge3 = edge(vertices[4], vertices[5])
        self.hexfaces.append(newface)
        self.hexedges.append(newedge1)
        self.hexedges.append(newedge2)
        self.hexedges.append(newedge3)

    def hexvertface(self, vert):
        vertices=[]
        for edge in vert.edges:
            eid = edge.index
            if edge.a is vert:
                vertices.append(self.hexedges[eid].a)
            elif edge.b is vert:
                vertices.append(self.hexedges[eid].b)
        newface = face(vertices)
        self.hexfaces.append(newface)
 
    def starify(self):
        self.starverts=[]
        self.staredges=[]
        self.starfaces=[]
        for i in xrange(len(self.verts)):
            self.verts[i].index = i
        for i in xrange(len(self.edges)):
            self.edges[i].index = i
        self.connectivity()
        star_vert_counter = 0
        for currentedge in self.edges:
            newvert = average([currentedge.a, currentedge.b]).centroid()
            newvert.index = star_vert_counter
            star_vert_counter += 1
            self.starverts.append(newvert)
        star_face_counter = 0
        star_edge_counter = 0
        for currentface in self.faces:
            currentface.docorners()
            vertices=[]
            for corner in currentface.corners:
                vert = self.starverts[corner[1].index]
#                vert.index = star_vert_counter
#                star_vert_counter += 1
                vertices.append(vert)
            newface = face(vertices)
            newface.index = star_face_counter
            star_face_counter += 1
            newedge1 = edge(vertices[0], vertices[1])
            newedge1.index = star_edge_counter
            newedge2 = edge(vertices[1], vertices[2])
            newedge2.index = star_edge_counter + 1
            newedge3 = edge(vertices[2], vertices[0])
            newedge3.index = star_edge_counter + 2
            star_edge_counter += 3
            self.starfaces.append(newface)
            self.staredges.append(newedge1)
            self.staredges.append(newedge2)
            self.staredges.append(newedge3)
        for vert in self.verts:
            vertices=[]
            vert.clockwise()
            for currentedge in vert.edges:
                eid = currentedge.index
                vertices.append(self.starverts[eid])
            newface = face(vertices)
            newface.index = star_face_counter
            star_face_counter += 1
            self.starfaces.append(newface)
        self.verts = self.starverts
        self.edges = self.staredges
        self.faces = self.starfaces
        self.vertedgeflag = 0
        self.vertfaceflag = 0
        self.faceedgeflag = 0

    def class2(self):
        self.class2verts=[] #PKHG_??? used?
        self.class2edges=[] #PKHG_??? used?
        self.class2faces=[]
        
        newvertstart = len(self.verts)
        newedgestart = len(self.edges)
        counter_verts = len(self.verts) #PKHG
#        for i in range(len(self.verts)):
        for i in xrange(counter_verts):
            self.verts[i].index = i
        for i in xrange(len(self.edges)):
            self.edges[i].index = i
        for i in xrange(len(self.faces)):
            self.faces[i].index = i
        self.connectivity()
        for currentface in self.faces:
            currentface.docorners()
            newvert = average(currentface.vertices).centroid()
            newvert.index = counter_verts
#PKHG_??? 20-11
            counter_verts += 1
            self.verts.append(newvert)
            newedge1 = edge(currentface.vertices[0], newvert)
            newedge2 = edge(currentface.vertices[1], newvert)
            newedge3 = edge(currentface.vertices[2], newvert)
            self.edges.append(newedge1)
            self.edges.append(newedge2)
            self.edges.append(newedge3)
        for currentedge in xrange(newedgestart):
            self.edges[currentedge].a = self.verts[self.edges[currentedge].faces[0].index+newvertstart]
            self.edges[currentedge].b = self.verts[self.edges[currentedge].faces[1].index+newvertstart]
            self.edges[currentedge].findvect()
        #breakpoint(locals(), True)                
        for currentvert in xrange(newvertstart):
            vert = self.verts[currentvert]
            vertices=[]
            vert.clockwise()
            for currentface in vert.faces:
                eid = currentface.index
#PKHG_OK                print(">>>>eid = ", eid, newvertstart + eid)
                vertices.append(self.verts[newvertstart + eid])
            #print("21-11 L710  currentvert is=", currentvert)
            #breakpoint(locals(), True)    
            for i in xrange(len(vertices)):
                if i == len(vertices) - 1:
                    next = vertices[0]
                else:
                    next = vertices[i+1]
                #print("21-11 L710  i is=", i)
                #breakpoint(locals(), True)    
                newface = face([vert, vertices[i], next])
                self.class2faces.append(newface)
        #self.verts = self.class2verts
        #self.edges = self.class2edges
        self.faces = self.class2faces
        self.vertedgeflag = 0
        self.vertfaceflag = 0
        self.faceedgeflag = 0    
        
    def dual(self):
        self.dualverts=[]
#        self.dualedges=[]
        self.dualfaces=[]
#PKHG 21-11 dual problem?!        
        counter_verts = len(self.verts)
        for i in xrange(counter_verts):
            self.verts[i].index = i
        for i in xrange(len(self.edges)):
            self.edges[i].index = i
        for i in xrange(len(self.faces)):
            self.faces[i].index = i
        self.connectivity()
        counter_verts = 0
        for currentface in self.faces:
            currentface.docorners()
            newvert = average(currentface.vertices).centroid()
            newvert.index = counter_verts #PKHG needed in >= 2.59 
            counter_verts += 1            
            self.dualverts.append(newvert)
        for vert in self.verts:
            vertices=[]
            vert.clockwise()
            for currentface in vert.faces:
                eid = currentface.index
                vertices.append(self.dualverts[eid])
            newface = face(vertices)
            self.dualfaces.append(newface)
        for currentedge in self.edges:
            currentedge.a = self.dualverts[currentedge.faces[0].index]
            currentedge.b = self.dualverts[currentedge.faces[1].index]
        self.verts = self.dualverts
#        self.edges = self.staredges
        self.faces = self.dualfaces
        self.vertedgeflag = 0
        self.vertfaceflag = 0
        self.faceedgeflag = 0
 
class facetype(mesh):
    def __init__(self, basegeodesic, parameters, width, height, relative):
        mesh.__init__(self)
        self.detatch = parameters[0]
        self.endtype = parameters[1]
        self.coords = parameters[2]
        self.base = basegeodesic
        self.relative = relative
        self.width = width
    
        if not self.relative:
            newwidth = self.findrelative()
            self.width = width*newwidth
        self.height = height
        self.base.connectivity()
        for coord in self.coords:
            coord[0]=coord[0]*self.width
            coord[1]=coord[1]*self.height
        if not self.base.facenormalflag:
            for currentface in self.base.faces:
            #    print("face normal ", currentface.normal)
                currentface.docorners()
                currentface.findnormal()
            #    print("face normal ", currentface.normal.x, currentface.normal.y, currentface.normal.z)
            self.base.facenormalflag = 1
        if self.endtype==4 and not self.base.vertnormalflag:
            for currentvert in self.base.verts:
                currentvert.findnormal()
            self.base.vertnormalflag = 1
        self.createfaces()
 
    def findrelative(self):
        centre = average(self.base.faces[0].vertices).centroid()
        edgelist=[]
        for point in self.base.faces[0].vertices:
            newedge = edge(centre, point)
            edgelist.append(newedge)
        length = 0
        for edg in edgelist:
            extra = edg.vect.length
            length = length+extra
        #    print("length", length,"extra", extra)
        length = length/len(edgelist)
    #    print("find relative", length)
        return length
 
    def createfaces(self):
        if not self.detatch:
            for point in self.base.verts:
                self.verts.append(point)
        if self.endtype==4:
            self.createghostverts()
        for currentface in self.base.faces:
            self.doface(currentface)

    def createghostverts(self):
        self.ghoststart = len(self.verts)
        for vert in self.base.verts:
            newvert = vert + (vert.normal * self.coords[-1][1])
            self.verts.append(newvert)
        
    def doface(self, candidate):
        grid=[]
        candidate.dospokes()
    #    print("Candidate normal", candidate.normal.x, candidate.normal.y, candidate.normal.z)
        if not self.detatch:
            line=[]
            for vert in candidate.vertices:
                line.append(vert)
            grid.append(line)
        else:
            line=[]
            for point in candidate.vertices:
                newvert = vertex(point.vector)
                self.verts.append(newvert)
                line.append(newvert)
            grid.append(line)
        finish = len(self.coords)
        if self.endtype==1 or self.endtype==4:
            finish = finish-1
        for i in xrange(finish):
            up = candidate.normal*self.coords[i][1]
            line=[]
            for j in xrange(len(candidate.vertices)):
                dotfac = candidate.corners[j][3]*0.5
                vec=(candidate.spokes[j]*(self.coords[i][0]/sin(dotfac)))       
                #self.coords[i][0])#(self.coords[i][0]/sin(dotfac)))#+up
                newvert = candidate.vertices[j]+vec+up
                line.append(newvert)
                self.verts.append(newvert)
            grid.append(line)
        if self.endtype==4:
            line=[]
            for i in xrange(len(candidate.vertices)):
                vert = self.verts[candidate.vertices[i].index+self.ghoststart]
                line.append(vert)
            #    self.verts.append(vert)
            grid.append(line)
        for line in grid:
            line.append(line[0])
        if self.endtype==3:
            grid.append(grid[0])
        for i in xrange(len(grid)-1):
            for j in xrange(len(grid[i])-1):
                one = grid[i][j]
                two = grid[i][j+1]
                three = grid[i+1][j+1]
                four = grid[i+1][j]
                newface = face([one, two, three, four])
                self.faces.append(newface)
        if self.endtype==2:
            finalfaceverts = grid[-1]
            newface = face(finalfaceverts[:-1])
            self.faces.append(newface)
        if self.endtype==1:
            lastvert = average(candidate.vertices).centroid()
            up = candidate.normal*self.coords[-1][1]
            newvert = lastvert+up
            self.verts.append(newvert)
            ring = grid[-1]
            for i in xrange(len(ring)-1):
                newface = face([newvert, ring[i], ring[i+1]])
                self.faces.append(newface)

class importmesh(mesh):
    def __init__(self, meshname, breakquadflag):    
        mesh.__init__(self)
        #PKHG>DBG: print("vefm L913 mesh and breakquad", meshname,  breakquadflag, "\n\n")
        #        impmesh = NMesh.GetRawFromObject(meshname)  #NMesh is not anymore
        #Does not have faces:        impmesh = bpy.data.objects[meshname] #PKHG>TODO error? struts?22
        obj = bpy.data.objects[meshname]
        bpy.context.scene.objects.active = obj
        obj.select = True
        copied_mesh = None
        impmesh = None
        if not breakquadflag:
            bpy.ops.object.mode_set(mode='EDIT')  
            impmesh = bmesh.new()   # create an empty BMesh
            impmesh.from_mesh(obj.data)   # fill it in from a Mesh
            bpy.ops.object.mode_set(mode='OBJECT')  

        if breakquadflag:
            #PKHG>OLD name = impmesh.name
            name = obj.name
#PKHG???needed???NO 3-11-2011            impmesh.name = "Original_" + name
            impmesh_name =  "Original_" + name
#PKHG TODO use a copy?   no not necessary         bpy.ops.object.duplicate_move(OBJECT_OT_duplicate={"linked":False, "mode":'TRANSLATION'}, TRANSFORM_OT_translate={"value":(0, 0,0}))
#PKHG TODO use a copy?            copied_mesh = bpy.context.active_object
            bpy.ops.object.mode_set(mode='EDIT')
            bpy.ops.mesh.quads_convert_to_tris()
            impmesh = bmesh.new()   # create an empty BMesh
            impmesh.from_mesh(obj.data)   # fill it in from a Mesh
            bpy.ops.object.mode_set(mode='OBJECT')  

        #PKHG>OLD        for v in impmesh.data.vertices:
        for v in impmesh.verts:
            vert = vertex(v.co)
            vert.index = v.index
            self.verts.append(vert)
            #PKHG verts is now a list of vertex, so to say a copy of the Vectors

        #PKHG edges
        #PKHF>OLD for e in impmesh.data.edges:
        for e in impmesh.edges:
            tmp = []
            for vert in e.verts:
                a = self.verts[vert.index]
                tmp.append(a)
            newedge = edge(tmp[0], tmp[1])
            newedge.index = e.index
            self.edges.append(newedge)            
        #PKHG faces
        extra_DBG_info("MeshInfo","vefm L995 the mesh impmesh", impmesh.faces[:])
        #PKHG>OLD for f in impmesh.data.faces:
        for f in impmesh.faces:
            temp=[]
            for vert in f.verts:  #PKHG a list! of indices ??? PKHG>??? 
                a = self.verts[vert.index] #PKHG verts contains already vertex objects
                temp.append(a)
            newface = face(temp)
            newface.index = f.index #indexcount
            self.faces.append(newface)
        self.dovertedge()
        self.dovertface()
        self.temp=[]    

        for i in xrange(len(self.verts)):
            self.temp.append([])
            self.verts[i].index = i
        for i in xrange(len(self.verts)):
            target = self.surroundingverts(self.verts[i])    
            for j in xrange(len(target)):                ## go through those verts
                temptarg = self.temp[target[j].index]        
                flag = 0                                ## set a flag up
        
                for k in xrange(len(temptarg)):          ## go through temp list for each of those verts
                    
                    if temptarg[k]==i:                  ## if we find a match to the current vert...
                        flag = 1           ## raise the flag
            
                if flag==0:                ## if there is no flag after all that...
                    self.temp[target[j].index].append(i)    ## add current vert to temp list of this surrounding vert
                    self.temp[i].append(target[j].index)    ## add this surrounding vert to the current temp list
                    newedge = edge(self.verts[i], self.verts[target[j].index])
                    self.edges.append(newedge)    ## add the newly found edge to the edges list
        
        for edg in self.edges:
            edg.findvect()
        self.vertedgeflag = 0    
        self.vertedgeflag = 0    
        self.connectivity()
#PKHG_DBG_OK        print("\n======= mesh imported")
                                                    
    def surroundingverts(self, vert):
        ''' Find the verts surrounding vert'''        
        surround=[]                    ## list to be filled and returned        
        for faces in vert.faces:        ## loop through faces attached to vert
            finish = len(faces.vertices)
            for i in xrange(finish):            
                if i==finish-1:
                    next = faces.vertices[0]
                else:
                    next = faces.vertices[i+1]
                if vert == faces.vertices[i]:
                    surround.append(next)                                    
        return surround

    def breakquad(self, quad_face):
        ''' turn quads into triangles'''
        distance1 = quad_face.vertices[0]-quad_face.vertices[2]
        distance2 = quad_face.vertices[1]-quad_face.vertices[3]        
        distance1.findlength()
        distance2.findlength()        
        if abs(distance1.length)<abs(distance2.length):
            self.faces[quad_face.index]=face([quad_face.vertices[0], quad_face.vertices[1], quad_face.vertices[2]])
            self.faces.append(face([quad_face.vertices[0], quad_face.vertices[2], quad_face.vertices[3]]))
        else:
            self.faces[quad_face.index]=face([quad_face.vertices[0], quad_face.vertices[1], quad_face.vertices[3]])
            self.faces.append(face([quad_face.vertices[1], quad_face.vertices[2], quad_face.vertices[3]]))            
        
                        
class strut(mesh):
    def __init__(self, base, struttype, width, height, length, widthtog, heighttog, lengthtog, meshname, stretchflag, lift):
        extra_DBG_info(name = "StrutMesh", info_text="vefm L1026\nstrut called: ", info_obj=[base, struttype, width, height, length, widthtog, heighttog, lengthtog, meshname, stretchflag, lift])
        mesh.__init__(self)
        ## put in strut prep stuff here
        if struttype==None:
            return
        total = 0
        divvy = len(base.faces[0].edges)
        for lengf in base.faces[0].edges:
            lengf.vect.findlength()
            total = total+lengf.vect.length
        yardstick = total/divvy
        if widthtog:
            self.width = width
        else:
            self.width = width * yardstick
        if heighttog:
            self.height = height
        else:
            self.height = height*yardstick
        if lengthtog:
            self.shrink = length
        else:
            self.shrink = length * yardstick        
        if not base.facenormalflag:
            for currentface in base.faces:
                currentface.docorners()
                currentface.findnormal()
            base.facenormalflag = 1
        for edj in base.edges:
            edj.findnormal()
            side = edge(edj.a, edj.b)
            edj.unit = side.vect
            edj.unit.normalize()    
            edj.cross = crossp(edj.normal, edj.unit).docrossproduct()
        template = importmesh(meshname,0)
        maxx = 0
        minx = 0
        for vert in template.verts:
#            if vert.x>maxx:
            if vert.vector.x > maxx:
#                maxx = vert.x
                maxx = vert.vector.x
#            if vert.x<minx:
            if vert.vector.x < minx:
#                minx = vert.x
                minx = vert.vector.x
        for edj in base.edges:
            start = len(self.verts)
            centre = average([edj.a, edj.b]).centroid()
            split = edj.vect.length/2
            #PKHG no division by zero!!
            tmp = 1.0
            if maxx != minx:
                tmp = 1.0/(maxx - minx)
#            dubbl = edj.vect.length/(maxx-minx)
            dubbl = edj.vect.length * tmp
            #PKHG end no division by zero!!
            diffplus = split-maxx
            diffminus=-split-minx
            for point in template.verts:
#                ay=(edj.normal*point.z*self.height)+(edj.normal*lift)
                ay=(edj.normal * point.vector.z * self.height) + (edj.normal * lift)
#                ce = edj.cross * point.y * self.width
                ce = edj.cross * point.vector.y * self.width
                if stretchflag:
#                    be = edj.unit*self.shrink*dubbl*point.x
                    be = edj.unit * self.shrink * dubbl * point.vector.x
                else:
#                    if point.x > 0.0:
                    if point.vector.x > 0.0:    
#                        be = edj.unit * self.shrink * (point.x + diffplus)
                        be = edj.unit * self.shrink * (point.vector.x + diffplus)
#                    elif point.x < 0.0:
                    elif point.vector.x < 0.0:    
#                        be = edj.unit * self.shrink * (point.x + diffminus)
                        be = edj.unit * self.shrink * (point.vector.x + diffminus)
#                    elif point.x == 0.0:
                    elif point.vector.x == 0.0:
#                        be = edj.unit * self.shrink * point.x
                        be = edj.unit * self.shrink * point.vector.x
                de = ay + be + ce
                newvert = centre + de
                self.verts.append(newvert)
            for edjy in template.edges:
                one = edjy.a.index+start
                two = edjy.b.index+start
                newedge = edge(self.verts[one], self.verts[two])
                self.edges.append(newedge)
            for facey in template.faces:
                faceverts=[]
                for verty in facey.vertices:
                    index = verty.index+start
                    faceverts.append(self.verts[index])
                newface = face(faceverts)
                self.faces.append(newface)
        self.vertedgeflag = 0    
        self.vertedgeflag = 0                        
        self.connectivity() 
        
        
class hub(mesh):
    def __init__(self, base, hubtype, width, height, length,\
                 widthtog, heighttog, lengthtog, meshname):
        mesh.__init__(self)
        self.width = 1.0
        self.height = 1.0
        self.shrink = 1.0
        ## put in strut prep stuff here
        extra_DBG_info("vefm L1133 HubMesh","base is ", str(dir(base))+"\n and meshname = " +  meshname)
        if hubtype==None:
            return
        total = 0
        divvy = len(base.faces[0].edges)
        for lengf in base.verts[0].edges:
            lengf.vect.findlength()
            total = total+lengf.vect.length
        yardstick = total / divvy
        if widthtog:
            self.width = width
        else:
            self.width = width * yardstick
        if heighttog:
            self.height = height
        else:
            self.height = height * yardstick
        if lengthtog:
            self.shrink = length
        else:
            self.shrink = length * yardstick
            
        if not base.facenormalflag:
            for currentface in base.faces:
                currentface.docorners()
                currentface.findnormal()
            base.facenormalflag = 1
#PKHG_2411        breakpoint(locals(), True) #PKHG_DBG 24-11 reason ERROR**** findnormal has no faces

            
        for apex in base.verts:
            apex.findnormal()
            side = edge(apex.edges[0].a, apex.edges[0].b)
            apex.unit = side.vect #PKHG is Vector: b -a
            apex.unit.normalize()    
            apex.cross = crossp(apex.normal, apex.unit).docrossproduct()
            apex.unit = crossp(apex.cross, apex.normal).docrossproduct()
            
        template = importmesh(meshname,0)
        for apex in base.verts:
            start = len(self.verts)
            centre = apex    
            for point in template.verts:
                ay = apex.normal * point.vector.z * self.height
                ce = apex.cross * point.vector.y * self.width
                be = apex.unit * point.vector.x * self.shrink
                de = ay+be+ce
                newvert = centre+de
                self.verts.append(newvert)
            for edjy in template.edges:
                one = edjy.a.index+start
                two = edjy.b.index+start
                newedge = edge(self.verts[one], self.verts[two])
                self.edges.append(newedge)
            for facey in template.faces:
                faceverts=[]
                for verty in facey.vertices:
                    index = verty.index+start
                    faceverts.append(self.verts[index])
                newface = face(faceverts)
                self.faces.append(newface)
        self.vertedgeflag = 0    
        self.vertedgeflag = 0                        
        self.connectivity()

#???PKHG TODO Nmesh used yet wrong!            
def finalfill(source, target):
    if source == target: #PKHG: otherewise >infinite< loop
        print "\n***WARNING*** vefm_271.finalfill L1148 source == target empty mesh used"
        target = mesh() #
#PKHG_??? maybe renumverting and checkkin faces wiht >=4 5 vertices?        
    count = 0
#PKHG_OK    print("\n>>>>>>>>20-11 DBG vefm_271 in finalfill L1152, len(source.verts) =", len(source.verts))
    for point in source.verts:
#        newvert = NMesh.Vert(point.x, point.y, point.z)
        newvert = vertex(point.vector)
#PKHG_??? needed???
        newvert.index = count 
        target.verts.append(newvert)
        point.index = count  #PKHG_INFO source renumbered too!
#PKHG_OK        print("test gelijk", newvert.vector == point.vector)
        count += 1 #count+1
    
    #PKHG indices of the vertex in faceyvertices are renumbered!
#PKHG_OK    print("\n>>>>>>>>20-11 DBG vefm_271 in finalfill L1163, len(source.faces) =", len(source.faces))    
    for facey in source.faces:
        row = len(facey.vertices)
        if row >= 5:
#PKHG does not take the good vectors???            newvert = average(facey.vertices).centroid()
            tmp = Vector()
            for el in facey.vertices:
                tmp = tmp +  target.verts[el.index].vector
            tmp = tmp / row
            centre = vertex(tmp) #does not work here!==>  average(facey.vertices).centroid()
            centre.index = count #PKHG_??? give it a good index
            count += 1
#PKHG_DBG 21-11
            #breakpoint(locals(), True)
            
            target.verts.append(centre)
            for i in xrange(row):
                if i == row - 1:
                    a = target.verts[facey.vertices[-1].index]
                    b = target.verts[facey.vertices[0].index]
                else:
                    a = target.verts[facey.vertices[i].index]
                    b = target.verts[facey.vertices[i+1].index]
                target.faces.append([a, b, centre])
        else:
            f = []
#PKHG_DBG 21-11
#            print("\n-----++++ L1217 dbg final dual", facey )
#PKHG_DBG 21-11
#            breakpoint(locals(), True)

            for j in xrange(len(facey.vertices)):
                a = facey.vertices[j]
#PKHG_NOTNEEDED                tmp = a.index
                f.append(target.verts[a.index])
            target.faces.append(f)

