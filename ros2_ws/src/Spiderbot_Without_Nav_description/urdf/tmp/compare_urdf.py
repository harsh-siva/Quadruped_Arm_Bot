import xml.etree.ElementTree as ET

def parse_urdf(path):
    tree = ET.parse(path)
    root = tree.getroot()
    data = {"joints": {}, "links": {}}
    for j in root.findall("joint"):
        name = j.get("name")
        origin = j.find("origin")
        axis = j.find("axis")
        limit = j.find("limit")
        data["joints"][name] = {
            "origin_xyz": origin.get("xyz") if origin is not None else None,
            "origin_rpy": origin.get("rpy") if origin is not None else None,
            "axis": axis.get("xyz") if axis is not None else None,
            "limit_upper": float(limit.get("upper")) if limit is not None else None,
            "limit_lower": float(limit.get("lower")) if limit is not None else None,
        }
    for l in root.findall("link"):
        name = l.get("name")
        inertial = l.find("inertial")
        if inertial is not None:
            origin = inertial.find("origin")
            mass = inertial.find("mass")
            data["links"][name] = {
                "inertial_xyz": origin.get("xyz") if origin is not None else None,
                "mass": mass.get("value") if mass is not None else None,
            }
        else:
            data["links"][name] = None
    return data

hand = parse_urdf("Spiderbot_Without_Nav.urdf")
gen = parse_urdf("/tmp/xacro_generated.urdf")

print("=== JOINT COMPARISON ===")
all_joints = set(hand["joints"]) | set(gen["joints"])
mismatches = 0
for name in sorted(all_joints):
    h = hand["joints"].get(name)
    g = gen["joints"].get(name)
    if h != g:
        mismatches += 1
        print(f"MISMATCH: {name}")
        print(f"  hand: {h}")
        print(f"  xacro: {g}")
print(f"Joints checked: {len(all_joints)}, mismatches: {mismatches}")

print("\n=== LINK INERTIAL COMPARISON ===")
all_links = set(hand["links"]) | set(gen["links"])
link_mismatches = 0
for name in sorted(all_links):
    h = hand["links"].get(name)
    g = gen["links"].get(name)
    if h != g:
        link_mismatches += 1
        print(f"MISMATCH: {name}")
        print(f"  hand: {h}")
        print(f"  xacro: {g}")
print(f"Links checked: {len(all_links)}, mismatches: {link_mismatches}")