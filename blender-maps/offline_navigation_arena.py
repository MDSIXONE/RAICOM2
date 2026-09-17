"""Create the Blender source scene for the local offline navigation map.

Run with a local Blender installation:
    blender --background --python offline_navigation_arena.py

The generated .blend is a visual source asset.  ROS map_server reads the
matching 2-D occupancy grid in robot_dog_navigation/maps instead.
"""

from pathlib import Path

import bpy


SCENE_FILE = Path(__file__).resolve().with_suffix(".blend")


def material(name, color):
    result = bpy.data.materials.new(name)
    result.diffuse_color = (*color, 1.0)
    return result


def box(name, location, dimensions, surface):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=location)
    item = bpy.context.active_object
    item.name = name
    item.dimensions = dimensions
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    item.data.materials.append(surface)
    return item


def main():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)

    floor = material("Floor", (0.18, 0.28, 0.32))
    wall = material("Walls", (0.72, 0.75, 0.72))
    marker = material("Robot_Start", (0.05, 0.7, 0.35))

    # The floor is 8 m x 6 m and matches the 20 x 15, 0.4 m/pixel PGM map.
    box("Floor", (4.0, 3.0, -0.08), (8.0, 6.0, 0.16), floor)
    wall_height = 0.65
    wall_depth = 0.16

    box("North_Wall", (4.0, 6.0, wall_height / 2), (8.0, wall_depth, wall_height), wall)
    box("South_Wall", (4.0, 0.0, wall_height / 2), (8.0, wall_depth, wall_height), wall)
    box("West_Wall", (0.0, 3.0, wall_height / 2), (wall_depth, 6.0, wall_height), wall)
    box("East_Wall", (8.0, 3.0, wall_height / 2), (wall_depth, 6.0, wall_height), wall)

    # Interior walls mirror the PGM's two vertical partitions and horizontal bar.
    box("Left_Partition_North", (2.2, 4.55, wall_height / 2), (wall_depth, 2.9, wall_height), wall)
    box("Left_Partition_South", (2.2, 0.95, wall_height / 2), (wall_depth, 1.9, wall_height), wall)
    box("Right_Partition", (5.0, 2.2, wall_height / 2), (wall_depth, 4.4, wall_height), wall)
    box("Upper_Hall_Wall", (5.9, 3.4, wall_height / 2), (2.0, wall_depth, wall_height), wall)

    bpy.ops.mesh.primitive_cylinder_add(vertices=32, radius=0.24, depth=0.08, location=(1.4, 1.8, 0.04))
    start = bpy.context.active_object
    start.name = "Offline_Robot_Start"
    start.data.materials.append(marker)

    bpy.ops.object.light_add(type="AREA", location=(4.0, 3.0, 7.0))
    bpy.context.active_object.data.energy = 900
    bpy.context.active_object.data.shape = "DISK"
    bpy.context.active_object.data.size = 6.0

    bpy.ops.object.camera_add(location=(4.0, 3.0, 10.0), rotation=(0.0, 0.0, 0.0))
    camera = bpy.context.active_object
    camera.name = "Top_Down_Camera"
    camera.data.type = "ORTHO"
    camera.data.ortho_scale = 9.5
    camera.rotation_euler = (0.0, 0.0, 0.0)
    # Point the camera down the negative Z axis without relying on UI state.
    camera.rotation_euler = (0.0, 0.0, 0.0)
    camera.rotation_euler[0] = 0.0
    bpy.context.scene.camera = camera

    bpy.context.scene.unit_settings.system = "METRIC"
    bpy.context.scene.render.engine = "BLENDER_EEVEE_NEXT"
    bpy.context.scene.world.color = (0.04, 0.04, 0.04)
    bpy.ops.wm.save_as_mainfile(filepath=str(SCENE_FILE))
    print(f"Saved Blender navigation scene: {SCENE_FILE}")


if __name__ == "__main__":
    main()
