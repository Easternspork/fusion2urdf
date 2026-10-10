"""Check and view a fusion2urdf export (assets/<name>/urdf/*.xacro) in MuJoCo.

  python   scripts/view_urdf.py path/to/<robot>_description --zup   (not mjpython: launch() crashes under it)
  python   scripts/view_urdf.py path/to/<robot>_description --check-only

Needs: pip install mujoco numpy (and trimesh for meshes over 200k faces).

Prints a validity report first (joint/link tree, unreferenced meshes, oversized
meshes), then opens the viewer with the base pinned in the air and every joint as a
slider-controlled position servo. Meshes over MuJoCo's 200k-face limit are decimated
into a cache dir for viewing only. Nothing in the export is modified.

The CAD frame is Fusion's Y-up; --zup rotates it so +Y CAD becomes +Z world.
"""

import argparse
import re
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

MAX_FACES = 200_000


def stl_faces(path: Path) -> int:
  b = path.read_bytes()[:84]
  if b[:5] == b"solid":
    return sum(1 for ln in path.read_text(errors="ignore").splitlines() if "facet" in ln)
  return int.from_bytes(b[80:84], "little")


def check(urdf: ET.Element, meshdir: Path) -> list[str]:
  """Return a list of problems (empty = looks valid)."""
  problems = []
  links = [ln.get("name") for ln in urdf.findall("link")]
  joints = urdf.findall("joint")
  print(f"robot '{urdf.get('name')}': {len(links)} links, {len(joints)} joints")

  parent_of: dict[str, str] = {}
  for j in joints:
    p, c = j.find("parent").get("link"), j.find("child").get("link")
    a = j.find("axis")
    lim = j.find("limit")
    rng = f" [{float(lim.get('lower')):+.3f}, {float(lim.get('upper')):+.3f}] rad" if lim is not None and lim.get("lower") else ""
    print(f"  {j.get('name'):16s} {j.get('type'):9s} {p} -> {c}  axis {a.get('xyz') if a is not None else '-'}{rng}")
    if p == c:
      problems.append(f"joint '{j.get('name')}' connects '{p}' to itself")
    if p not in links or c not in links:
      problems.append(f"joint '{j.get('name')}' references a missing link ({p} -> {c})")
    if c in parent_of:
      problems.append(f"link '{c}' has two parents ('{parent_of[c]}' and '{p}')")
    parent_of[c] = p

  roots = [ln for ln in links if ln not in parent_of]
  if len(roots) != 1:
    problems.append(f"expected one root link, found {len(roots)}: {roots} (unconnected parts?)")
  # cycles
  for ln in links:
    seen, cur = set(), ln
    while cur in parent_of:
      if cur in seen:
        problems.append(f"cycle in the tree through '{cur}'")
        break
      seen.add(cur)
      cur = parent_of[cur]

  used = set()
  for m in urdf.iter("mesh"):
    used.add(Path(m.get("filename")).name)
  on_disk = {p.name for p in meshdir.glob("*.stl")}
  for name in sorted(on_disk - used):
    problems.append(f"mesh '{name}' is not used by any link (missing part?)")
  for name in sorted(used - on_disk):
    problems.append(f"mesh '{name}' is referenced but not in {meshdir}")
  for name in sorted(used & on_disk):
    n = stl_faces(meshdir / name)
    if n > MAX_FACES:
      print(f"  note: {name} has {n} faces (> {MAX_FACES}); decimated for viewing")
  return problems


def repair(urdf: ET.Element, meshdir: Path) -> None:
  """Patch a broken tree just enough to view it (the export is not modified)."""
  print("\nviewing anyway with these view-only repairs:")
  links = {ln.get("name") for ln in urdf.findall("link")}
  parent_of: dict[str, str] = {}
  for j in urdf.findall("joint"):
    p, c = j.find("parent").get("link"), j.find("child").get("link")
    seen, cur = {c}, p
    while cur in parent_of and cur not in seen:
      seen.add(cur)
      cur = parent_of[cur]
    if p == c or p not in links or c not in links or c in parent_of or cur in seen:
      print(f"  - dropped joint '{j.get('name')}' ({p} -> {c})")
      urdf.remove(j)
    else:
      parent_of[c] = p
  roots = [ln for ln in links if ln not in parent_of]
  main_root = "base_link" if "base_link" in roots else roots[0]
  for r in roots:
    if r != main_root:
      print(f"  - welded unconnected link '{r}' to '{main_root}' (fixed joint)")
      j = ET.SubElement(urdf, "joint", name=f"weld_{r}", type="fixed")
      ET.SubElement(j, "parent", link=main_root)
      ET.SubElement(j, "child", link=r)
  used = {Path(m.get("filename")).name for m in urdf.iter("mesh")}
  for f in sorted(meshdir.glob("*.stl")):
    if f.name in used:
      continue
    name = "unused_" + f.stem
    print(f"  - added unused mesh '{f.name}' as massless link '{name}' welded to '{main_root}'")
    ln = ET.SubElement(urdf, "link", name=name)
    vis = ET.SubElement(ln, "visual")
    ET.SubElement(ET.SubElement(vis, "geometry"), "mesh", filename=f.name, scale="0.001 0.001 0.001")
    j = ET.SubElement(urdf, "joint", name=f"weld_{name}", type="fixed")
    ET.SubElement(j, "parent", link=main_root)
    ET.SubElement(j, "child", link=name)


def decompose_collisions(urdf: ET.Element, meshdir: Path, cache: Path) -> None:
  """Replace every link's collision mesh with convex pieces (cached per mesh).

  MuJoCo collides a mesh as its convex hull, which fills in concave parts: a
  shell-shaped base_link's hull swallows the legs, and a leg's hull can reach
  into base_link where the real CAD has clearance.
  """
  for link in urdf.findall("link"):
    col = link.find("collision")
    mesh = col.find("geometry/mesh") if col is not None else None
    if mesh is None:
      continue
    src = meshdir / Path(mesh.get("filename")).name
    parts_dir = cache / f"coacd_{src.stem}"
    if not any(parts_dir.glob("*.stl")):
      import coacd
      import trimesh

      print(f"decomposing {src.name} into convex pieces (first run only)...")
      tm = trimesh.load(src)
      parts = coacd.run_coacd(coacd.Mesh(tm.vertices, tm.faces), threshold=0.02 if link.get("name") == "base_link" else 0.05)  # base_link: tight fit to the shell
      parts_dir.mkdir(parents=True, exist_ok=True)
      for i, (v, f) in enumerate(parts):
        trimesh.Trimesh(v, f).export(parts_dir / f"{src.stem}_coacd_{i:03d}.stl")
    files = sorted(parts_dir.glob("*.stl"))
    link.remove(col)
    for f in files:
      c = ET.SubElement(link, "collision")
      c.append(col.find("origin"))
      ET.SubElement(ET.SubElement(c, "geometry"), "mesh", filename=str(f), scale=mesh.get("scale"))


def main() -> None:
  ap = argparse.ArgumentParser()
  ap.add_argument("package", type=Path, help="fusion2urdf package dir (has urdf/, meshes/)")
  ap.add_argument("--check-only", action="store_true")
  ap.add_argument("--zup", action="store_true", help="rotate CAD +Y up to world +Z")
  ap.add_argument("--height", type=float, default=0.5, help="base height in the viewer [m]")
  args = ap.parse_args()

  pkg = args.package.resolve()
  if not (pkg / "urdf").is_dir():
    sys.exit(f"{pkg} is not a fusion2urdf export (no urdf/ folder) - check the path")
  xacro = next((p for p in (pkg / "urdf").glob("*.xacro") if p.name != "materials.xacro"), None)
  if xacro is None:
    sys.exit(f"no robot .xacro in {pkg / 'urdf'}")
  meshdir = pkg / "meshes"
  text = xacro.read_text()
  urdf = ET.fromstring(text)

  problems = check(urdf, meshdir)
  if problems:
    print("\nPROBLEMS:")
    for p in problems:
      print("  - " + p)
  else:
    print("\nno structural problems found")
  if args.check_only:
    sys.exit(1 if problems else 0)
  if problems:
    repair(urdf, meshdir)

  # Strip xacro includes, point meshes at local (possibly decimated) files.
  cache = Path(tempfile.gettempdir()) / f"view_urdf_{pkg.name}"
  cache.mkdir(exist_ok=True)
  for inc in list(urdf):
    if inc.tag.endswith("include"):
      urdf.remove(inc)
  decompose_collisions(urdf, meshdir, cache)
  for m in urdf.iter("mesh"):
    if Path(m.get("filename")).is_absolute():
      continue  # already a cached file
    name = Path(m.get("filename")).name
    src = meshdir / name
    if stl_faces(src) > MAX_FACES:
      dst = cache / name
      if not dst.exists():
        import trimesh

        trimesh.load(src).simplify_quadric_decimation(face_count=100_000).export(dst)
      src = dst
    m.set("filename", str(src))
  for mat in urdf.iter("material"):
    for c in list(mat):
      mat.remove(c)
  ET.SubElement(ET.SubElement(urdf, "mujoco"), "compiler", discardvisual="false", fusestatic="false", strippath="false")
  urdf_xml = re.sub(r"xmlns:xacro=\"[^\"]*\"", "", ET.tostring(urdf, encoding="unicode"))

  spec = mujoco.MjSpec.from_string(urdf_xml)
  # Pin the root in the air, optionally Z-up, add a floor and position servos.
  root = spec.worldbody.first_body()
  # URDF root is welded to the world (no freejoint), so just place it.
  root.pos = [0, 0, args.height]
  if args.zup:
    root.quat = [0.7071068, 0.7071068, 0, 0]
  spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.1], rgba=[0.8, 0.8, 0.8, 1])
  spec.worldbody.add_light(pos=[0, 0, 3], dir=[0, 0, -1])
  # CAD parts overlap at the joints; robot-robot contacts would shove them apart.
  # CAD parts overlap at the joints, so robot parts don't collide with each
  # other - except base_link, which collides with everything. All collision
  # meshes are convex-decomposed above so contacts match the real CAD.
  for g in spec.geoms:
    if g.type == mujoco.mjtGeom.mjGEOM_PLANE:
      g.conaffinity = 2
    elif g.contype == 0 and g.conaffinity == 0:
      pass  # visual-only geom
    elif g.parent and g.parent.name == "base_link":
      g.contype, g.conaffinity = 2, 2
    else:
      g.contype, g.conaffinity = 2, 0
  for j in spec.joints:
    a = spec.add_actuator(name=j.name, target=j.name, trntype=mujoco.mjtTrn.mjTRN_JOINT)
    a.set_to_position(kp=1.0, kv=1.0)  # gains replaced after compile
    if j.range[1] > j.range[0]:
      a.ctrlrange = j.range
  spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST  # stable servo damping
  spec.memory = 256 << 20  # many convex pieces -> many contacts
  model = spec.compile()
  # Servo gains scaled to each joint's inertia (~5 Hz, critically damped); a fixed
  # kp is unstable for light links at the default timestep.
  data = mujoco.MjData(model)
  mujoco.mj_forward(model, data)
  for i in range(model.nu):
    inertia = data.qM[model.dof_Madr[model.jnt_dofadr[model.actuator_trnid[i, 0]]]]
    kp = inertia * (2 * np.pi * 5) ** 2
    kv = 2 * np.sqrt(kp * inertia)
    model.actuator_gainprm[i, 0] = kp
    model.actuator_biasprm[i, 1:3] = [-kp, -kv]
  print(f"compiled: {model.nbody - 1} bodies, {model.njnt} joints, total mass {model.body_subtreemass[0]:.3f} kg")

  mujoco.viewer.launch(model)


if __name__ == "__main__":
  main()
