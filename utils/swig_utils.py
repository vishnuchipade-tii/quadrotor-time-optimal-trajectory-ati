def find_swig_objects(obj, path="root", visited=None):
  if visited is None:
    visited = set()

  obj_id = id(obj)
  if obj_id in visited:
    return

  visited.add(obj_id)

  # SWIG detection
  if obj.__class__.__name__ == "SwigPyObject":
    print(f"SWIG FOUND at {path} -> {obj}")
    return

  # primitive type
  if isinstance(obj, (int, float, str, bool, type(None))):
    return

  # containers
  if isinstance(obj, (list, tuple)):
    for i, item in enumerate(obj):
      find_swig_objects(item, f"{path}[{i}]", visited)
    return

  if isinstance(obj, dict):
    for k, v in obj.items():
      find_swig_objects(v, f"{path}[{k}]", visited)
    return

  # objects
  if hasattr(obj, "__dict__"):
    for k, v in vars(obj).items():
      find_swig_objects(v, f"{path}.{k}", visited)