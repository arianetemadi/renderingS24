
import math
import os, re
import shutil
from xml.dom.minidom import Document

import bpy
import bpy_extras
from bpy.props import BoolProperty, IntProperty, StringProperty
from bpy_extras.io_utils import ExportHelper
from mathutils import Matrix, Vector

bl_info = {
    "name": "Export Nori scenes format",
    "author": "Adrien Gruson, Delio Vicini, Tizian Zeltner",
    "version": (0, 1),
    "blender": (2, 80, 0),
    "location": "File > Export > Nori exporter (.xml)",
    "description": "Export Nori scene format (.xml)",
    "warning": "",
    "wiki_url": "",
    "tracker_url": "",
    "category": "Import-Export"}


class NoriWriter:

    def __init__(self, context, filepath):
        self.context = context
        self.filepath = filepath
        self.working_dir = os.path.dirname(self.filepath)

    def create_xml_element(self, name, attr):
        el = self.doc.createElement(name)
        for k, v in attr.items():
            el.setAttribute(k, v)
        return el

    def create_xml_entry(self, t, name, value):
        return self.create_xml_element(t, {"name": name, "value": value})

    def create_xml_transform(self, mat, el=None):
        transform = self.create_xml_element("transform", {"name": "toWorld"})
        if(el):
            transform.appendChild(el)
        value = ""
        for j in range(4):
            for i in range(4):
                value += str(mat[j][i]) + ","
        transform.appendChild(self.create_xml_element("matrix", {"value": value[:-1]}))
        return transform

    def create_xml_mesh_entry(self, filename):
        meshElement = self.create_xml_element("mesh", {"type": "obj"})
        meshElement.appendChild(self.create_xml_element("string", {"name": "filename", "value": "meshes/"+filename}))
        return meshElement

    def write(self):
        """Main method to write the blender scene into Nori format"""

        n_samples = 128
        # create xml document
        self.doc = Document()
        self.scene = self.doc.createElement("scene")
        self.doc.appendChild(self.scene)

        # 1) write integrator configuration
        integrator_element = self.create_xml_element("integrator", {"type": "path_tracer_recursive"})
        integrator_element.appendChild(self.create_xml_element("boolean", {"name": "rr", "value": "true"}))
        integrator_element.appendChild(self.create_xml_element("boolean", {"name": "rr_dynamic", "value": "true"}))
        integrator_element.appendChild(self.create_xml_element("boolean", {"name": "nee", "value": "true"}))
        integrator_element.appendChild(self.create_xml_element("boolean", {"name": "mis", "value": "true"}))
        self.scene.appendChild(integrator_element);
		
        # 2) write sampler
        sampler = self.create_xml_element("sampler", {"type": "independent"})
        sampler.appendChild(self.create_xml_element("integer", {"name": "sampleCount", "value": str(n_samples)}))
        self.scene.appendChild(sampler)

        # 3) export one camera
        cameras = [cam for cam in self.context.scene.objects
                   if cam.type in {'CAMERA'}]
        if(len(cameras) == 0):
            print("WARN: No camera to export")
        else:
            if(len(cameras) > 1):
                print("WARN: Does not handle multiple camera, only export the active one")
            self.scene.appendChild(self.write_camera(self.context.scene.camera))  # export the active one

        # 4) export all meshes
        if not os.path.exists(self.working_dir + "/meshes"):
            os.makedirs(self.working_dir + "/meshes")

        meshes = [obj for obj in self.context.scene.objects
                  if obj.type in {'MESH', 'FONT', 'SURFACE', 'META'}]
        print(meshes)
        for mesh in meshes:
            self.write_mesh(mesh)

		# 5) export parallogram lights
        emitters = [emitter for emitter in self.context.scene.objects if emitter.type in {'LIGHT'} and emitter.data.type in {'AREA'}]

        for emitter in emitters:
            self.write_emitter(emitter)

        # 6) write the xml file
        self.doc.writexml(open(self.filepath, "w"), "", "\t", "\n")

    def write_camera(self, cam):
        """convert the selected camera (cam) into xml format"""
        camera = self.create_xml_element("camera", {"type": "perspective"})
        camera.appendChild(self.create_xml_entry("float", "fov", str(cam.data.angle * 180 / math.pi)))
        camera.appendChild(self.create_xml_entry("float", "nearClip", str(cam.data.clip_start)))
        camera.appendChild(self.create_xml_entry("float", "farClip", str(cam.data.clip_end)))
        percent = self.context.scene.render.resolution_percentage / 100.0
        camera.appendChild(self.create_xml_entry("integer", "width", str(
            int(self.context.scene.render.resolution_x * percent))))
        camera.appendChild(self.create_xml_entry("integer", "height", str(
            int(self.context.scene.render.resolution_y * percent))))

        mat = cam.matrix_world

        # Conversion to Y-up coordinate system
        coord_transf = bpy_extras.io_utils.axis_conversion(
            from_forward='Y', from_up='Z', to_forward='-Z', to_up='Y').to_4x4()
        mat = coord_transf @ mat
        pos = mat.translation
        # Nori's camera needs this these coordinates to be flipped
        m = Matrix([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 0]])
        t = mat.to_3x3() @ m.to_3x3()
        mat = Matrix([[t[0][0], t[0][1], t[0][2], pos[0]],
                      [t[1][0], t[1][1], t[1][2], pos[1]],
                      [t[2][0], t[2][1], t[2][2], pos[2]],
                      [0, 0, 0, 1]])
        trans = self.create_xml_transform(mat)
        camera.appendChild(trans)
        return camera

    def write_mesh(self, mesh):
        viewport_selection = self.context.selected_objects
        bpy.ops.object.select_all(action='DESELECT')

        obj_name = mesh.name + ".obj"
        obj_path = os.path.join(self.working_dir, 'meshes', obj_name)
        mesh.select_set(True)
        bpy.ops.export_scene.obj(filepath=obj_path, check_existing=False,
                                    use_selection=True, use_edges=False, use_smooth_groups=False,
                                    use_materials=False, use_triangles=True, use_mesh_modifiers=True)
        mesh.select_set(False)

        # Add the corresponding entry to the xml
        mesh_element = self.create_xml_mesh_entry(obj_name)
        # We currently just export a default material, a more complex material conversion
        # could be implemented following: http://tinyurl.com/nnhxwuh
        bsdf_element = self.create_xml_element("bsdf", {"type": "diffuse"})
        bsdf_element.appendChild(self.create_xml_entry("color", "albedo", "0.75,0.75,0.75"))
        mesh_element.appendChild(bsdf_element)
        self.scene.appendChild(mesh_element)

        for ob in viewport_selection:
            ob.select_set(True)

    def write_emitter(self, emitter):
        if emitter.data.type == 'AREA' and emitter.data.shape in {'SQUARE', 'RECTANGLE'}:
            # Use snake case instead of camel case
            emitter_name = emitter.name

            # Calculate emitter transform
            emitter_v = emitter.matrix_world.to_quaternion() @ Vector((emitter.matrix_world.to_scale()[0], 0.0, 0.0))
            emitter_u = emitter.matrix_world.to_quaternion() @ Vector((0.0, emitter.matrix_world.to_scale()[1], 0.0))
            
            if emitter.data.shape == 'SQUARE':
                emitter_size_x = emitter.data.size
                emitter_size_y = emitter.data.size
            else:            
                emitter_size_x = emitter.data.size
                emitter_size_y = emitter.data.size_y
        
            emitter_position_origin = emitter.location + emitter_u * emitter_size_y/2 + emitter_v * emitter_size_x/2

            u = -emitter_u * emitter_size_y
            v = -emitter_v * emitter_size_x

            # Conversion to Y-up coordinate system
            coord_transf = bpy_extras.io_utils.axis_conversion(
                from_forward='Y', from_up='Z', to_forward='-Z', to_up='Y').to_4x4()
            emitter_position_origin = coord_transf @ emitter_position_origin
            u = coord_transf @ u
            v = coord_transf @ v

            # Convert energy to radiance (lighting should be pretty much the same as in cycles, should work imho)
            radiance = 1.0 / (u.cross(v).length * 4.0) * emitter.data.energy * emitter.data.color if u.cross(v).length > 0.0 else 0.0
            
            # Write emitter parameters
            mesh_element = self.create_xml_element("mesh", {"type": "parallelogram"})
            mesh_element.appendChild(self.create_xml_element("string", {"name": "name", "value": emitter_name}))
            mesh_element.appendChild(self.create_xml_element("point", {"name": "origin", "value": ', '.join(str(e) for e in emitter_position_origin)}))
            mesh_element.appendChild(self.create_xml_element("vector", {"name": "u", "value": ', '.join(str(e) for e in u)}))
            mesh_element.appendChild(self.create_xml_element("vector", {"name": "v", "value": ', '.join(str(e) for e in v)}))
            
            emitter_element = self.create_xml_element("emitter", {"type": "parallelogram_emitter"})
            emitter_element.appendChild(self.create_xml_element("color", {"name": "radiance", "value": ', '.join(str(round(e, 2)) for e in radiance)}))
            
            mesh_element.appendChild(emitter_element)
            
            self.scene.appendChild(mesh_element)
        else:
            print("WARN: Only square and rectangle shaped emitters are supported.")
			

class NoriExporter(bpy.types.Operator, ExportHelper):
    """Export a blender scene into Nori scene format"""

    # add to menu
    bl_idname = "export_scene.nori"
    bl_label = "Export Nori scene"

    filename_ext = ".xml"
    filter_glob: StringProperty(default="*.xml", options={'HIDDEN'})

    def execute(self, context):
        nori = NoriWriter(context, self.filepath)
        nori.write()
        return {'FINISHED'}

def menu_func_export(self, context):
    self.layout.operator(NoriExporter.bl_idname, text="Export Nori scene...")


def register():
    bpy.utils.register_class(NoriExporter)
    bpy.types.TOPBAR_MT_file_export.append(menu_func_export)


def unregister():
    bpy.utils.unregister_class(NoriExporter)
    bpy.types.TOPBAR_MT_file_export.remove(menu_func_export)


if __name__ == "__main__":
    register()
