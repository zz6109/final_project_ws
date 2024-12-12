# map.pgm & map.yaml을 바탕으로 gazebo map model 생성하는 코드

import os
import shutil
from PIL import Image

home_dir = "/home/test"
gazebo_model_dir = os.path.join(home_dir, ".gazebo/models/map")
textures_dir = os.path.join(gazebo_model_dir, "materials/textures")
scripts_dir = os.path.join(gazebo_model_dir, "materials/scripts")

os.makedirs(textures_dir, exist_ok=True)
os.makedirs(scripts_dir, exist_ok=True)

# Convert PGM to PNG
pgm_file = "/home/test/map.pgm"
png_file = os.path.join(textures_dir, "map.png")

print("Converting PGM to PNG...")
with Image.open(pgm_file) as img:
    img = img.convert("RGB")
    img.save(png_file)
print(f"Saved PNG file at {png_file}")

# Create model.config
model_config_content = """<?xml version=\"1.0\"?>
<model>
    <name>map</name>
    <version>1.0</version>
    <sdf version=\"1.6\">model.sdf</sdf>
    <author>
        <name>Your Name</name>
        <email>your.email@example.com</email>
    </author>
    <description>Map based on uploaded files</description>
</model>
"""

model_config_path = os.path.join(gazebo_model_dir, "model.config")
with open(model_config_path, "w") as f:
    f.write(model_config_content)
print(f"Created model.config at {model_config_path}")

# Create model.sdf
model_sdf_content = f"""<?xml version=\"1.0\" ?>
<sdf version=\"1.6\">
  <model name=\"map\">
    <static>true</static>
    <link name=\"map_link\">
      <visual name=\"map_visual\">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>10 10</size> <!-- Adjust size based on map resolution -->
          </plane>
        </geometry>
        <material>
          <script>
            <uri>model://map/materials/scripts</uri>
            <uri>model://map/materials/textures</uri>
            <name>Map/MapTexture</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

model_sdf_path = os.path.join(gazebo_model_dir, "model.sdf")
with open(model_sdf_path, "w") as f:
    f.write(model_sdf_content)
print(f"Created model.sdf at {model_sdf_path}")

# Create a world file
world_content = """<?xml version=\"1.0\" ?>
<sdf version=\"1.6\">
  <world name=\"default\">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://map</uri>
    </include>
  </world>
</sdf>
"""

world_file_path = os.path.join(home_dir, "map_world.world")
with open(world_file_path, "w") as f:
    f.write(world_content)
print(f"Created world file at {world_file_path}")

print("Gazebo map and world generation complete!")