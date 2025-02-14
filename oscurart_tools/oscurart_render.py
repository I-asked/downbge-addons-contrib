from __future__ import with_statement
from __future__ import division
from __future__ import absolute_import
import bpy
import math
import os
import stat
from io import open


##-------------------------------- RENDER ALL SCENES ----------------------------


def defRenderAll (frametype, scenes):

    
    activescene = bpy.context.scene
    FC = bpy.context.scene.frame_current
    FS = bpy.context.scene.frame_start
    FE = bpy.context.scene.frame_end
    print "---------------------"
    types = set(['MESH','META','CURVE'])

    for ob in bpy.data.objects:
        if ob.type in types:
            if not len(ob.material_slots):
                ob.data.materials.append(None)

    slotlist = { ob : [sl.material for sl in ob.material_slots] for ob in bpy.data.objects if ob.type in types if len (ob.material_slots)} 

    for scene in scenes:     
        proptolist = list(eval(scene.overrides))
        cursc = scene.name
        renpath = scene.render.filepath
        endpath = scene.render.filepath
        filepath = bpy.data.filepath

        if frametype:
            scene.frame_start=FC
            scene.frame_end=FC
            scene.frame_end=FC
            scene.frame_start=FC
            
        for group, material in proptolist:
            for object in bpy.data.groups[group].objects:
                lenslots = len(object.material_slots)
                if object.type in types:
                    if len(object.data.materials):
                        object.data.materials.clear()
                        for newslot in xrange(lenslots):
                            object.data.materials.append(bpy.data.materials[material])            
        filename = os.path.basename(bpy.data.filepath.rpartition(".")[0])  
        uselayers = dict((layer, layer.use) for layer in scene.render.layers) 
        for layer, usado in uselayers.items():
            if usado:
                for i in scene.render.layers: i.use = False  
                layer.use = 1  
                print "SCENE: %s" % scene.name
                print "LAYER: %s" % layer.name
                print "OVERRIDE: %s" % str(proptolist)   
                scene.render.filepath = os.path.join(os.path.dirname(renpath), filename, scene.name, layer.name,"%s_%s_%s" % (filename,scene.name,layer.name))
                bpy.context.window.screen.scene = scene  
                bpy.ops.render.render(animation=True, write_still=True, layer=layer.name, scene= scene.name)   
                print "DONE"
                print "---------------------" 
        for layer, usado in uselayers.items():
            layer.use = usado
        scene.render.filepath = renpath
        for ob,slots in slotlist.items():
            ob.data.materials.clear()
            for slot in slots:
                ob.data.materials.append(slot)
        if frametype:
            scene.frame_start=FS
            scene.frame_end=FE
            scene.frame_end=FE
            scene.frame_start=FS    
                  
    bpy.context.window.screen.scene = activescene   


class renderAll (bpy.types.Operator):
    bl_idname="render.render_all_scenes_osc"
    bl_label="Render All Scenes"

    frametype=bpy.props.BoolProperty(default=False)

    def execute(self,context):
        defRenderAll(self.frametype,[scene for scene in bpy.data.scenes])
        return set(['FINISHED'])


##--------------------------------RENDER SELECTED SCENES----------------------------

bpy.types.Scene.use_render_scene = bpy.props.BoolProperty()


class renderSelected (bpy.types.Operator):
    bl_idname="render.render_selected_scenes_osc"
    bl_label="Render Selected Scenes"

    frametype=bpy.props.BoolProperty(default=False)

    def execute(self,context):
        defRenderAll(self.frametype,[ sc for sc in bpy.data.scenes if sc.use_render_scene])
        return set(['FINISHED'])

##--------------------------------RENDER CURRENT SCENE----------------------------


class renderCurrent (bpy.types.Operator):
    bl_idname="render.render_current_scene_osc"
    bl_label="Render Current Scene"

    frametype=bpy.props.BoolProperty(default=False)

    def execute(self,context):

        defRenderAll(self.frametype,[bpy.context.scene])

        return set(['FINISHED'])


##--------------------------RENDER CROP----------------------
bpy.types.Scene.rcPARTS = bpy.props.IntProperty(default=0, min=2, max=50, step=1)

def OscRenderCropFunc():
    
    
    SCENENAME = os.path.split(bpy.data.filepath)[-1].partition(".")[0]
    PARTS = bpy.context.scene.rcPARTS
    CHUNKYSIZE = 1/PARTS
    FILEPATH = bpy.context.scene.render.filepath
    bpy.context.scene.render.use_border = True
    bpy.context.scene.render.use_crop_to_border = True
    for PART in xrange(PARTS):
        bpy.context.scene.render.border_min_y = PART*CHUNKYSIZE
        bpy.context.scene.render.border_max_y = (PART*CHUNKYSIZE)+CHUNKYSIZE
        bpy.context.scene.render.filepath = "%s_part%s" % (os.path.join(FILEPATH,SCENENAME,bpy.context.scene.name,SCENENAME),PART)
        bpy.ops.render.render(animation=False, write_still=True)
        
    bpy.context.scene.render.filepath = FILEPATH    
        
class renderCrop (bpy.types.Operator):
    bl_idname="render.render_crop_osc"
    bl_label="Render Crop: Render!"
    def execute(self,context):
        OscRenderCropFunc()
        return set(['FINISHED'])

##---------------------------BATCH MAKER------------------
def defoscBatchMaker(TYPE,BIN):
    
    
    if os.sys.platform.startswith("w"):
        print "PLATFORM: WINDOWS"
        SYSBAR = os.sep
        EXTSYS = ".bat"
        QUOTES = '"'
    else:
        print "PLATFORM:LINUX"
        SYSBAR = os.sep
        EXTSYS = ".sh"
        QUOTES = ''
    
    FILENAME = bpy.data.filepath.rpartition(SYSBAR)[-1].rpartition(".")[0]
    BINDIR = bpy.app[4]
    SHFILE = os.path.join (bpy.data.filepath.rpartition(SYSBAR)[0] , FILENAME + EXTSYS)
    
    with open(SHFILE,"w") as FILE:
        # assign permission in linux
        if EXTSYS == ".sh":
            try:
                os.chmod(SHFILE, stat.S_IRWXU)
            except:
                print "** Oscurart Batch maker can not modify the permissions."    
        if not BIN:
            FILE.writelines("%s%s%s -b %s -x 1 -o %s -P %s%s.py  -s %s -e %s -a" %
                (QUOTES,BINDIR,QUOTES,bpy.data.filepath,bpy.context.scene.render.filepath,bpy.data.filepath.rpartition(SYSBAR)[0]+
                SYSBAR,TYPE,str(bpy.context.scene.frame_start),str(bpy.context.scene.frame_end)) )
        else:
            FILE.writelines("%s -b %s -x 1 -o %s -P %s%s.py  -s %s -e %s -a" %
                ("blender",bpy.data.filepath,bpy.context.scene.render.filepath,bpy.data.filepath.rpartition(SYSBAR)[0]+
                SYSBAR,TYPE,str(bpy.context.scene.frame_start),str(bpy.context.scene.frame_end)) )            
    
    RLATFILE =  "%s%sosRlat.py" % (bpy.data.filepath.rpartition(SYSBAR)[0] , SYSBAR )
    if not os.path.isfile(RLATFILE):
        with open(RLATFILE,"w")  as file:      
            if EXTSYS == ".sh":
                try:
                    os.chmod(RLATFILE, stat.S_IRWXU)  
                except:
                    print "** Oscurart Batch maker can not modify the permissions."                             
            file.writelines("import bpy \nbpy.ops.render.render_all_scenes_osc()\nbpy.ops.wm.quit_blender()")

    else:
        print "The All Python files Skips: Already exist!"   
         
    RSLATFILE = "%s%sosRSlat.py" % (bpy.data.filepath.rpartition(SYSBAR)[0] , SYSBAR)    
    if not os.path.isfile(RSLATFILE):  
        with  open(RSLATFILE,"w")  as file:          
            if EXTSYS == ".sh":
                try:
                    os.chmod(RSLATFILE, stat.S_IRWXU)   
                except:
                    print "** Oscurart Batch maker can not modify the permissions."                            
            file.writelines("import bpy \nbpy.ops.render.render_selected_scenes_osc()\nbpy.ops.wm.quit_blender()")
    else:
        print "The Selected Python files Skips: Already exist!"          

class oscBatchMaker (bpy.types.Operator):
    bl_idname = "file.create_batch_maker_osc"
    bl_label = "Make render batch"
    bl_options = set(['REGISTER', 'UNDO'])

    type = bpy.props.EnumProperty(
            name="Render Mode",
            description="Select Render Mode.",
            items=(('osRlat', "All Scenes", "Render All Layers At Time"),
                   ('osRSlat', "Selected Scenes", "Render Only The Selected Scenes")),
            default='osRlat',
            )

    bin = bpy.props.BoolProperty(default=False,name="Use Environment Variable")
    
    def execute(self,context):
        defoscBatchMaker(self.type,self.bin)
        return set(['FINISHED'])

## --------------------------------------PYTHON BATCH--------------------------------------------------------
def defoscPythonBatchMaker(BATCHTYPE,SIZE):
    
    
    # REVISO SISTEMA
    if os.sys.platform.startswith("w"):
        print "PLATFORM: WINDOWS"
        SYSBAR = "\\"
        EXTSYS = ".bat"
        QUOTES = '"'
    else:
        print "PLATFORM:LINUX"
        SYSBAR = "/"
        EXTSYS = ".sh"    
        QUOTES = ''
    
    # CREO VARIABLES
    FILENAME = bpy.data.filepath.rpartition(SYSBAR)[-1].rpartition(".")[0]
    SHFILE = "%s%s%s_PythonSecureBatch.py"   % (bpy.data.filepath.rpartition(SYSBAR)[0],SYSBAR,FILENAME)
    BATCHLOCATION = "%s%s%s%s"   % (bpy.data.filepath.rpartition(SYSBAR)[0],SYSBAR,FILENAME,EXTSYS)
    
    with open(SHFILE,"w") as FILEBATCH:
            
        if EXTSYS == ".bat":
            BATCHLOCATION=BATCHLOCATION.replace("\\","/")    
        
        # SI EL OUTPUT TIENE DOBLE BARRA LA REEMPLAZO
        FRO=bpy.context.scene.render.filepath        
        if bpy.context.scene.render.filepath.count("//"):
            FRO=bpy.context.scene.render.filepath.replace("//", bpy.data.filepath.rpartition(SYSBAR)[0]+SYSBAR)         
        if EXTSYS == ".bat":
            FRO=FRO.replace("\\","/")                
                     
        #CREO BATCH
        bpy.ops.file.create_batch_maker_osc(type=BATCHTYPE)
        
        SCRIPT = "import os \nREPITE= True \nBAT= '%s'\nSCENENAME ='%s' \nDIR='%s%s' \ndef RENDER():\n    os.system(BAT) \ndef CLEAN():\n    global REPITE\n    FILES  = [root+'/'+FILE for root, dirs, files in os.walk(os.getcwd()) if len(files) > 0 for FILE in files if FILE.count('~') == False]\n    RESPUESTA=False\n    for FILE in FILES:\n        if os.path.getsize(FILE) < %s:\n            os.remove(FILE)\n            RESPUESTA= True\n    if RESPUESTA:\n        REPITE=True\n    else:\n        REPITE=False\nREPITE=True\nwhile REPITE:\n    REPITE=False\n    RENDER()\n    os.chdir(DIR)\n    CLEAN()" % (BATCHLOCATION,FILENAME,FRO,FILENAME,SIZE)        
        
        # DEFINO ARCHIVO DE BATCH
        FILEBATCH.writelines(SCRIPT)    
    
    
    # ARCHIVO CALL
    CALLFILENAME = bpy.data.filepath.rpartition(SYSBAR)[-1].rpartition(".")[0]
    CALLFILE = "%s%s%s_CallPythonSecureBatch%s"   % (bpy.data.filepath.rpartition(SYSBAR)[0],SYSBAR,CALLFILENAME,EXTSYS)  

    with open(CALLFILE,"w") as CALLFILEBATCH:
        
        SCRIPT = "python %s" % (SHFILE)
        CALLFILEBATCH.writelines(SCRIPT)
  
    if EXTSYS == ".sh":
        try:
            os.chmod(CALLFILE, stat.S_IRWXU)  
            os.chmod(SHFILE, stat.S_IRWXU) 
        except:
            print "** Oscurart Batch maker can not modify the permissions."      

    
class oscPythonBatchMaker (bpy.types.Operator):
    bl_idname = "file.create_batch_python"
    bl_label = "Make Batch Python"
    bl_options = set(['REGISTER', 'UNDO'])

    size = bpy.props.IntProperty(name="Size in Bytes", default=10, min=0)
    
    type = bpy.props.EnumProperty(
            name="Render Mode",
            description="Select Render Mode.",
            items=(('osRlat', "All Scenes", "Render All Layers At Time"),
                   ('osRSlat', "Selected Scenes", "Render Only The Selected Scenes")),
            default='osRlat',
            )

    def execute(self,context):
        defoscPythonBatchMaker(self.type, self.size)
        return set(['FINISHED'])


# ---------------------------------- BROKEN FRAMES --------------------- 

class VarColArchivos (bpy.types.PropertyGroup):
    filename = bpy.props.StringProperty(name="", default="")
    value = bpy.props.IntProperty(name="", default=10)    
    fullpath = bpy.props.StringProperty(name="", default="")
    checkbox = bpy.props.BoolProperty(name="", default=True)
bpy.utils.register_class(VarColArchivos)

class SumaFile(bpy.types.Operator):
    bl_idname = "object.add_broken_file"
    bl_label = "Add Broken Files"

    def execute(self, context):
        os.chdir(os.path.dirname(bpy.data.filepath))
        absdir = os.path.join(os.path.dirname(bpy.data.filepath),bpy.context.scene.render.filepath.replace(r"//",""))
        for root, folder, files in os.walk(absdir):  
            for f in files:
                if os.path.getsize(os.path.join(root,f)) <  10:    
                    print f      
                    i = bpy.context.scene.broken_files.add()
                    i.filename = f
                    i.fullpath = os.path.join(root,f)
                    i.value = os.path.getsize(os.path.join(root,f))   
                    i.checkbox = True   
        return set(['FINISHED'])  

class ClearFile(bpy.types.Operator):
    bl_idname = "object.clear_broken_file"
    bl_label = "Clear Broken Files"

    def execute(self, context):
        bpy.context.scene.broken_files.clear()
        return set(['FINISHED']) 

class DeleteFiles(bpy.types.Operator):
    bl_idname = "object.delete_broken_file"
    bl_label = "Delete Broken Files"

    def execute(self, context):
        for file in bpy.context.scene.broken_files:
            if file.checkbox:
                os.remove(file.fullpath)
        bpy.context.scene.broken_files.clear()    
        return set(['FINISHED'])     


bpy.types.Scene.broken_files = bpy.props.CollectionProperty(type=VarColArchivos)

class BrokenFramesPanel (bpy.types.Panel):
    bl_label = "Oscurart Broken Render Files"
    bl_idname = "OBJECT_PT_osc_broken_files"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "render"

    def draw(self, context):
        layout = self.layout
        obj = context.object
        col = layout.column(align=1)
               
        for i in bpy.context.scene.broken_files:
            colrow = col.row(align=1)
            colrow.prop(i, "filename")
            colrow.prop(i, "value")
            colrow.prop(i, "checkbox")

        col = layout.column(align=1)
        colrow = col.row(align=1)
        colrow.operator("object.add_broken_file")
        colrow.operator("object.clear_broken_file")
        colrow = col.row(align=1)
        colrow.operator("object.delete_broken_file")


##--------------------------------COPY RENDER SETTINGS----------------------------

def defCopyRenderSettings(mode):
    
    sc = bpy.context.scene
    sceneslist = bpy.data.scenes[:]
    sceneslist.remove(sc)    

    excludes = set(['name','objects', 'object_bases', 'has_multiple_engines', 'display_settings', 'broken_files', 'rna_type', 'frame_subframe', 'view_settings', 'tool_settings', 'render', 'user_clear', 'animation_data_create', 'collada_export', 'keying_sets', 'icon_props', 'image_settings', 'library', 'bake', 'active_layer', 'frame_current_final', 'sequence_editor_clear', 'rigidbody_world', 'unit_settings', 'orientations', '__slots__', 'ray_cast', 'sequencer_colorspace_settings', 'ffmpeg', 'is_movie_format', 'frame_path', 'frame_set', 'network_render', 'animation_data_clear', 'is_nla_tweakmode', 'keying_sets_all', 'sequence_editor', '__doc__', 'ovlist', 'file_extension', 'users', 'node_tree', 'is_updated_data', 'bl_rna', 'is_library_indirect', 'cycles_curves', 'timeline_markers', 'statistics', 'use_shading_nodes', 'use_game_engine', 'sequence_editor_create', 'is_updated', '__module__', 'update_tag', 'update', 'animation_data', 'cycles', 'copy', 'game_settings', 'layers','__weakref__','string','double','overrides','use_render_scene','engine','use_nodes','world'])
       
    
    if mode == "render":    
        scenerenderdict = {}
        scenedict = {}
        sceneimagesettingdict = {}
        for prop in dir(bpy.context.scene.render):
            if prop not in excludes:
                try:
                    scenerenderdict[prop] = getattr(bpy.context.scene.render,prop) 
                except:
                    print "%s does not exist." % (prop)     

        for prop in dir(bpy.context.scene):
            if prop not in excludes:
                try:
                    scenedict[prop] = getattr(bpy.context.scene,prop) 
                except:
                    print "%s does not exist." % (prop)  
                
        for prop in dir(bpy.context.scene.render.image_settings):
            if prop not in excludes:
                try:
                    sceneimagesettingdict[prop] = getattr(bpy.context.scene.render.image_settings,prop) 
                except:
                    print "%s does not exist." % (prop)   
                                               
        """    
        scenerenderdict = {prop : getattr(bpy.context.scene.render,prop) for prop in dir(bpy.context.scene.render)}        
        scenedict = {prop : getattr(bpy.context.scene,prop) for prop in dir(bpy.context.scene) if prop not in excludes}
        sceneimagesettingdict = {prop : getattr(bpy.context.scene.render.image_settings,prop) for prop in dir(bpy.context.scene.render.image_settings)}        
        """    
        
        # render
        for escena in sceneslist:
            for prop,value in scenerenderdict.items():
                try:
                    setattr(escena.render, prop,value)
                except:
                    print "%s was not copied!" % (prop)
                    pass
     
        # scene        
        for escena in sceneslist:    
            for prop,value in scenedict.items():
                try:
                    setattr(escena, prop,value)
                except:
                    print "%s was not copied!" % (prop)
                    pass
        # imageSettings             
        for escena in sceneslist:
            for prop,value in sceneimagesettingdict.items():  
                try:
                    setattr(escena.render.image_settings, prop,value)
                except:
                    print "%s was not copied!" % (prop)                             
                    pass      
                    
    if mode == "cycles":  
        scenecyclesdict = {}
        for prop in dir(bpy.context.scene.cycles):
            if prop not in excludes:
                try:
                    scenecyclesdict[prop] = getattr(bpy.context.scene.cycles,prop) 
                except:
                    print "%s does not exist." % (prop)        
        
        """
        scenecyclesdict = {prop : getattr(bpy.context.scene.cycles,prop) for prop in dir(bpy.context.scene.cycles)}  
        """            
        # cycles
        for escena in sceneslist:
            for prop,value in scenecyclesdict.items():   
                try:
                    setattr(escena.cycles, prop,value)
                except:
                    print "%s was not copied!" % (prop)                             
                    pass
              
   

class copyRenderSettings (bpy.types.Operator):
    bl_idname="render.copy_render_settings_osc"
    bl_label="Copy Render Settings"
    #bl_options = {'REGISTER', 'UNDO'}

    mode = bpy.props.StringProperty(default="")

    def execute(self,context):
        
        defCopyRenderSettings(self.mode)

        return set(['FINISHED'])









