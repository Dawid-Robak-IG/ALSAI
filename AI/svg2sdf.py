import sys
import xml.etree.ElementTree as ET
import math
import os

def generate_model(svg_file, wall_height=2.5, wall_thickness=0.2):
    model_name=svg_file[0:-4]
    svg_file = os.path.expanduser(f"~/ALSAI/worlds/pics/{svg_file}")
    output_dir=os.path.expanduser(f"~/ALSAI/worlds/{model_name}")
    resolution=0.1
    tree = ET.parse(svg_file)
    root = tree.getroot()

    ns = {"svg": "http://www.w3.org/2000/svg"}

    walls = []

    for line in root.findall(".//svg:line", ns):
        x1 = float(line.get("x1")) * resolution
        y1 = float(line.get("y1")) * resolution
        x2 = float(line.get("x2")) * resolution
        y2 = float(line.get("y2")) * resolution

        length = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0

        yaw = math.atan2((y2 - y1), (x2 - x1))

        walls.append((length, wall_thickness, wall_height, cx, cy, wall_height/2, yaw))

    sdf_lines = [
        '<?xml version="1.0" ?>',
        '<sdf version="1.6">',
        f'  <model name="{model_name}">',
        '    <static>true</static>'
    ]

    for i, (L, W, H, x, y, z, yaw) in enumerate(walls):
        sdf_lines.append(f'    <link name="wall_{i}">')
        sdf_lines.append(f'      <pose>{x} {y} {z} 0 0 {yaw}</pose>')
        sdf_lines.append('      <collision name="collision">')
        sdf_lines.append('        <geometry>')
        sdf_lines.append(f'          <box><size>{L} {W} {H}</size></box>')
        sdf_lines.append('        </geometry>')
        sdf_lines.append('      </collision>')
        sdf_lines.append('      <visual name="visual">')
        sdf_lines.append('        <geometry>')
        sdf_lines.append(f'          <box><size>{L} {W} {H}</size></box>')
        sdf_lines.append('        </geometry>')
        sdf_lines.append('        <material>')
        sdf_lines.append('          <ambient>0.7 0.7 0.7 1</ambient>')
        sdf_lines.append('          <diffuse>0.7 0.7 0.7 1</diffuse>')
        sdf_lines.append('        </material>')
        sdf_lines.append('      </visual>')
        sdf_lines.append('    </link>')

    sdf_lines.append('  </model>')
    sdf_lines.append('</sdf>')

    os.makedirs(output_dir, exist_ok=True)
    sdf_path = os.path.join(output_dir, "model.sdf")
    config_path = os.path.join(output_dir, "model.config")

    with open(sdf_path, "w") as f:
        f.write("\n".join(sdf_lines))

    config = f"""<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Auto SVG2SDF</name>
    <email>none</email>
  </author>
  <description>
    Labirynt wygenerowany automatycznie z pliku {os.path.basename(svg_file)}.
  </description>
</model>
"""
    with open(config_path, "w") as f:
        f.write(config)

    print("✅ Wygenerowano model.sdf i model.config")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Użycie: python svg2sdf.py labirynt.svg")
        sys.exit(1)

    generate_model(sys.argv[1])
