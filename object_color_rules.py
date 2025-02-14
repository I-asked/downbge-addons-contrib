# ***** BEGIN GPL LICENSE BLOCK *****
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

from __future__ import absolute_import
from itertools import izip
bl_info = {
    "name": "Object Color Rules",
    "author": "Campbell Barton",
    "version": (0, 0, 1),
    "blender": (2, 73, 0),
    "location": "Properties > Object Buttons",
    "description": "Rules for assigning object color (used for wireframe colors).",
    "wiki_url": "http://wiki.blender.org/index.php/Extensions:2.6/Py/"
                "Scripts/Object/Color_Rules",
    "category": "Object",
    }


def test_name(rule, needle, haystack):
    # TODO, compile expression for re-use
    if rule.use_match_regex:
        import re
        return (re.match(needle, haystack) is not None)
    else:
        return (needle in haystack)


class rule_test(object):
    __slots__ = ()

    def __new__(cls, *args, **kwargs):
        raise RuntimeError("%s should not be instantiated" % cls)

    @staticmethod
    def NAME(obj, rule, cache):
        match_name = rule.match_name
        return test_name(rule, match_name, obj.name)

    def DATA(obj, rule, cache):
        match_name = rule.match_name
        obj_data = obj.data
        if obj_data is not None:
            return test_name(rule, match_name, obj_data.name)
        else:
            return False

    @staticmethod
    def GROUP(obj, rule, cache):
        if not cache:
            match_name = rule.match_name
            objects = set([o for g in bpy.data.groups if test_name(rule, match_name, g.name) for o in g.objects])
            cache["objects"] = objects
        else:
            objects = cache["objects"]

        return obj in objects

    @staticmethod
    def MATERIAL(obj, rule, cache):
        match_name = rule.match_name
        materials = getattr(obj.data, "materials", None)

        return ((materials is not None) and
                (any((test_name(rule, match_name, m.name) for m in materials if m is not None))))

    @staticmethod
    def LAYER(obj, rule, cache):
        match_layers = rule.match_layers[:]
        obj_layers = obj.layers[:]

        return any((match_layers[i] and obj_layers[i]) for i in xrange(20))

    @staticmethod
    def TYPE(obj, rule, cache):
        return (obj.type == rule.match_object_type)

    @staticmethod
    def EXPR(obj, rule, cache):
        if not cache:
            match_expr = rule.match_expr
            expr = compile(match_expr, rule.name, 'eval')

            namespace = {}
            namespace.update(__import__("math").__dict__)

            cache["expr"] = expr
            cache["namespace"] = namespace
        else:
            expr = cache["expr"]
            namespace = cache["namespace"]

        try:
            return bool(eval(expr, {}, {"self": obj}))
        except:
            import traceback
            traceback.print_exc()
            return False


class rule_draw(object):
    __slots__ = ()

    def __new__(cls, *args, **kwargs):
        raise RuntimeError("%s should not be instantiated" % cls)

    @staticmethod
    def _generic_match_name(layout, rule):
        layout.label("Match Name:")
        row = layout.row(align=True)
        row.prop(rule, "match_name", text="")
        row.prop(rule, "use_match_regex", text="", icon='SORTALPHA')

    @staticmethod
    def NAME(layout, rule):
        rule_draw._generic_match_name(layout, rule)

    @staticmethod
    def DATA(layout, rule):
        rule_draw._generic_match_name(layout, rule)

    @staticmethod
    def GROUP(layout, rule):
        rule_draw._generic_match_name(layout, rule)

    @staticmethod
    def MATERIAL(layout, rule):
        rule_draw._generic_match_name(layout, rule)

    @staticmethod
    def TYPE(layout, rule):
        row = layout.row()
        row.prop(rule, "match_object_type")

    @staticmethod
    def LAYER(layout, rule):
        row = layout.row()
        row.prop(rule, "match_layers")

    @staticmethod
    def EXPR(layout, rule):
        col = layout.column()
        col.label("Scripted Expression:")
        col.prop(rule, "match_expr", text="")


def object_colors_calc(rules, objects):
    from mathutils import Color

    rules_cb = [getattr(rule_test, rule.type) for rule in rules]
    rules_blend = [(1.0 - rule.factor, rule.factor) for rule in rules]
    rules_color = [Color(rule.color) for rule in rules]
    rules_cache = [{} for i in xrange(len(rules))]
    rules_inv = [rule.use_invert for rule in rules]

    for obj in objects:
        is_set = False
        obj_color = Color(obj.color[0:3])

        for (rule, test_cb, color, blend, cache, use_invert) \
             in izip(rules, rules_cb, rules_color, rules_blend, rules_cache, rules_inv):

            if test_cb(obj, rule, cache) is not use_invert:
                if is_set is False:
                    obj_color = color
                else:
                    # prevent mixing colors loosing saturation
                    obj_color_s = obj_color.s
                    obj_color = (obj_color * blend[0]) + (color * blend[1])
                    obj_color.s = (obj_color_s * blend[0]) + (color.s * blend[1])

                is_set = True

        if is_set:
            obj.show_wire_color = True
            obj.color[0:3] = obj_color


def object_colors_select(rule, objects):
    cache = {}

    rule_type = rule.type
    test_cb = getattr(rule_test, rule_type)

    for obj in objects:
        obj.select = test_cb(obj, rule, cache)


def object_colors_rule_validate(rule, report):
    rule_type = rule.type

    if rule_type in set(['NAME', 'DATA', 'GROUP', 'MATERIAL']):
        if rule.use_match_regex:
            import re
            try:
                re.compile(rule.match_name)
            except Exception, e:
                report(set(['ERROR']), "Rule %r: %s" % (rule.name, str(e)))
                return False

    elif rule_type == 'EXPR':
        try:
            compile(rule.match_expr, rule.name, 'eval')
        except Exception, e:
            report(set(['ERROR']), "Rule %r: %s" % (rule.name, str(e)))
            return False

    return True



import bpy
from bpy.types import (
        Operator,
        Panel,
        UIList,
        )
from bpy.props import (
        StringProperty,
        BoolProperty,
        IntProperty,
        FloatProperty,
        EnumProperty,
        CollectionProperty,
        BoolVectorProperty,
        FloatVectorProperty,
        )


class OBJECT_PT_color_rules(Panel):
    bl_label = "Color Rules"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "object"

    def draw(self, context):
        layout = self.layout

        scene = context.scene

        # Rig type list
        row = layout.row()
        row.template_list(
                "OBJECT_UL_color_rule", "color_rules",
                scene, "color_rules",
                scene, "color_rules_active_index")

        col = row.column()
        colsub = col.column(align=True)
        colsub.operator("object.color_rules_add", icon='ZOOMIN', text="")
        colsub.operator("object.color_rules_remove", icon='ZOOMOUT', text="")

        colsub = col.column(align=True)
        colsub.operator("object.color_rules_move", text="", icon='TRIA_UP').direction = -1
        colsub.operator("object.color_rules_move", text="", icon='TRIA_DOWN').direction = 1

        colsub = col.column(align=True)
        colsub.operator("object.color_rules_select", text="", icon='RESTRICT_SELECT_OFF')

        if scene.color_rules:
            index = scene.color_rules_active_index
            rule = scene.color_rules[index]

            box = layout.box()
            row = box.row(align=True)
            row.prop(rule, "name", text="")
            row.prop(rule, "type", text="")
            row.prop(rule, "use_invert", text="", icon='ARROW_LEFTRIGHT')

            draw_cb = getattr(rule_draw, rule.type)
            draw_cb(box, rule)

            row = layout.split(0.75, align=True)
            props = row.operator("object.color_rules_assign", text="Assign Selected")
            props.use_selection = True
            props = row.operator("object.color_rules_assign", text="All")
            props.use_selection = False


class OBJECT_UL_color_rule(UIList):
    def draw_item(self, context, layout, data, rule, icon, active_data, active_propname, index):
        # assert(isinstance(rule, bpy.types.ShapeKey))
        # scene = active_data
        split = layout.split(0.5)
        row = split.split(align=False)
        row.label(text="%s (%s)" % (rule.name, rule.type.lower()))
        split = split.split(0.7)
        split.prop(rule, "factor", text="", emboss=False)
        split.prop(rule, "color", text="")


class OBJECT_OT_color_rules_assign(Operator):
    """Assign colors to objects based on user rules"""
    bl_idname = "object.color_rules_assign"
    bl_label = "Assign Colors"
    bl_options = set(['UNDO'])

    use_selection = BoolProperty(
            name="Selected",
            description="Apply to selected (otherwise all objects in the scene)",
            default=True,
            )
    def execute(self, context):
        scene = context.scene

        if self.use_selection:
            objects = context.selected_editable_objects
        else:
            objects = scene.objects

        rules = scene.color_rules[:]
        for rule in rules:
            if not object_colors_rule_validate(rule, self.report):
                return set(['CANCELLED'])

        object_colors_calc(rules, objects)
        return set(['FINISHED'])


class OBJECT_OT_color_rules_select(Operator):
    """Select objects matching the current rule"""
    bl_idname = "object.color_rules_select"
    bl_label = "Select Rule"
    bl_options = set(['UNDO'])

    def execute(self, context):
        scene = context.scene
        rule = scene.color_rules[scene.color_rules_active_index]

        if not object_colors_rule_validate(rule, self.report):
            return set(['CANCELLED'])

        objects = context.visible_objects
        object_colors_select(rule, objects)
        return set(['FINISHED'])


class OBJECT_OT_color_rules_add(Operator):
    bl_idname = "object.color_rules_add"
    bl_label = "Add Color Layer"
    bl_options = set(['UNDO'])

    def execute(self, context):
        scene = context.scene
        rules = scene.color_rules
        rule = rules.add()
        rule.name = "Rule.%.3d" % len(rules)
        scene.color_rules_active_index = len(rules) - 1
        return set(['FINISHED'])


class OBJECT_OT_color_rules_remove(Operator):
    bl_idname = "object.color_rules_remove"
    bl_label = "Remove Color Layer"
    bl_options = set(['UNDO'])

    def execute(self, context):
        scene = context.scene
        rules = scene.color_rules
        rules.remove(scene.color_rules_active_index)
        if scene.color_rules_active_index > len(rules) - 1:
            scene.color_rules_active_index = len(rules) - 1
        return set(['FINISHED'])


class OBJECT_OT_color_rules_move(Operator):
    bl_idname = "object.color_rules_move"
    bl_label = "Remove Color Layer"
    bl_options = set(['UNDO'])
    direction = IntProperty()

    def execute(self, context):
        scene = context.scene
        rules = scene.color_rules
        index = scene.color_rules_active_index
        index_new = index + self.direction
        if index_new < len(rules) and index_new >= 0:
            rules.move(index, index_new)
            scene.color_rules_active_index = index_new
            return set(['FINISHED'])
        else:
            return set(['CANCELLED'])


class ColorRule(bpy.types.PropertyGroup):
    name = StringProperty(
            name="Rule Name",
            )
    color = FloatVectorProperty(
            name="Color",
            description="Color to assign",
            subtype='COLOR', size=3, min=0, max=1, precision=3, step=0.1,
            default=(0.5, 0.5, 0.5),
            )
    factor = FloatProperty(
            name="Opacity",
            description="Color to assign",
            min=0, max=1, precision=1, step=0.1,
            default=1.0,
            )
    type = EnumProperty(
            name="Rule Type",
            items=(('NAME', "Name", ""),
                   ('DATA', "Data Name", "Name of the object data"),
                   ('GROUP', "Group Name", "Object in group"),
                   ('MATERIAL', "Material Name", "Object uses material"),
                   ('TYPE', "Type", "Object type"),
                   ('LAYER', "Layer", "Object in layer"),
                   ('EXPR', "Expression", "Scripted expression"),
                   ),
            )

    use_invert = BoolProperty(
            name="Invert",
            description="Match when the rule isn't met",
            )

    # ------------------
    # Matching Variables

    # shared by all name matching
    match_name = StringProperty(
            name="Match Name",
            )
    use_match_regex = BoolProperty(
            name="Regex",
            description="Use regular expressions for pattern matching",
            )
    # type == 'LAYER'
    match_layers = BoolVectorProperty(
            name="Layers",
            size=20,
            subtype='LAYER',
            )
    # type == 'TYPE'
    match_object_type = EnumProperty(
            name="Object Type",
            items=([(i.identifier, i.name, "")
                    for i in bpy.types.Object.bl_rna.properties['type'].enum_items]
                    )
            )
    # type == 'EXPR'
    match_expr = StringProperty(
            name="Expression",
            description="Python expression, where 'self' is the object variable"
            )

classes = (
    OBJECT_PT_color_rules,
    OBJECT_OT_color_rules_add,
    OBJECT_OT_color_rules_remove,
    OBJECT_OT_color_rules_move,
    OBJECT_OT_color_rules_assign,
    OBJECT_OT_color_rules_select,
    OBJECT_UL_color_rule,
    ColorRule,
    )


def register():
    for cls in classes:
        bpy.utils.register_class(cls)

    bpy.types.Scene.color_rules = CollectionProperty(type=ColorRule)
    bpy.types.Scene.color_rules_active_index = IntProperty()


def unregister():
    for cls in classes:
        bpy.utils.unregister_class(cls)

    del bpy.types.Scene.color_rules
