import xml.etree.ElementTree as ET
import sys
import os

def convert_paths_to_lines(svg_in, svg_out=None):
    svg_in = os.path.expanduser(f"~/ALSAI/worlds/pics/{svg_in}")
    tree = ET.parse(svg_in)
    root = tree.getroot()

    changed = 0

    ns = {"svg": "http://www.w3.org/2000/svg"}
    paths = root.findall(".//svg:path", ns)
    if not paths:
        paths = root.findall(".//path")

    for path in paths:
        d = path.get("d")
        if not d:
            continue

        tokens = d.replace(",", " ").split()
        points = []
        i = 0
        x0 = y0 = None
        cur_x = cur_y = None

        while i < len(tokens):
            cmd = tokens[i]
            if cmd.lower() == "m":
                x = float(tokens[i+1])
                y = float(tokens[i+2])
                if cmd == 'm' and cur_x is not None:
                    x += cur_x
                    y += cur_y
                cur_x, cur_y = x, y
                if x0 is None:
                    x0, y0 = x, y
                points.append((cur_x, cur_y))
                i += 3
                while i + 1 < len(tokens) and not tokens[i].isalpha():
                    dx = float(tokens[i])
                    dy = float(tokens[i+1])
                    if cmd == 'm':
                        cur_x += dx
                        cur_y += dy
                    else:
                        cur_x = dx
                        cur_y = dy
                    points.append((cur_x, cur_y))
                    i += 2
            elif cmd.lower() == "l":
                x = float(tokens[i+1])
                y = float(tokens[i+2])
                if cmd == 'l':
                    cur_x += x
                    cur_y += y
                else:
                    cur_x, cur_y = x, y
                points.append((cur_x, cur_y))
                i += 3
            elif cmd.lower() == "z":
                if points and x0 is not None:
                    points.append((x0, y0))
                    cur_x, cur_y = x0, y0
                i += 1
            else:
                i += 1

        if len(points) < 2:
            continue
        style = path.get("style", "stroke:black;stroke-width:1")

        for j in range(len(points)-1):
            x1, y1 = points[j]
            x2, y2 = points[j+1]
            line = ET.Element("ns0:line", {
                "x1": str(x1),
                "y1": str(y1),
                "x2": str(x2),
                "y2": str(y2),
                "style": style
            })

            parent = None
            for p in root.iter():
                for idx, child in enumerate(list(p)):
                    if child is path:
                        parent = p
                        p.remove(child)
                        p.insert(idx, line)
                        break
                if parent:
                    break

        changed += 1

    polylines = root.findall(".//svg:polyline", ns)
    if not polylines:
        polylines = root.findall(".//polyline")

    for poly in polylines:
        pts = poly.get("points")
        if not pts:
            continue
        coords = []
        for pair in pts.strip().split():
            if "," in pair:
                x, y = pair.split(",")
                coords.append((float(x), float(y)))

        if len(coords) < 2:
            continue

        style = poly.get("style", "stroke:black;stroke-width:1")

        parent = None
        for p in root.iter():
            for idx, child in enumerate(list(p)):
                if child is poly:
                    parent = p
                    p.remove(child)
                    for j in range(len(coords) - 1):
                        x1, y1 = coords[j]
                        x2, y2 = coords[j+1]
                        line = ET.Element("ns0:line", {
                            "x1": str(x1),
                            "y1": str(y1),
                            "x2": str(x2),
                            "y2": str(y2),
                            "style": style
                        })
                        p.insert(idx + j, line)
                    break
            if parent:
                break

        changed += 1


    if not svg_out:
        svg_out = svg_in.replace(".svg", "_lines.svg")

    tree.write(svg_out)
    print(f"✅ Zamieniono {changed} pathów na <line>, zapisano do: {svg_out}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Użycie: python paths2lines.py plik.svg [plik_wy.svg]")
        sys.exit(1)

    svg_in = sys.argv[1]
    svg_out = sys.argv[2] if len(sys.argv) > 2 else None
    convert_paths_to_lines(svg_in, svg_out)
