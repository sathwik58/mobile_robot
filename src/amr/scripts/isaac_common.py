"""Small helpers shared by the Isaac Sim scripts (run with Isaac Sim's python.sh)."""
import os

# Worlds saved from the Isaac GUI point at the NVIDIA cloud; the same files are on disk here.
CLOUD_ROOT = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1"
LOCAL_ROOT = os.path.expanduser("~/isaacsim_assets_5.1/Assets/Isaac/5.1")


def localize_layer(layer):
    """Point cloud payload/reference paths in one layer at the local asset copy.

    Only the layer in memory is changed; nothing is saved. Returns the number of paths changed.
    """
    from pxr import Sdf

    if layer is None or not os.path.isdir(LOCAL_ROOT):
        return 0
    count = 0

    def fix(items, make):
        nonlocal count
        out = []
        for it in items:
            if it.assetPath.startswith(CLOUD_ROOT):
                it = make(LOCAL_ROOT + it.assetPath[len(CLOUD_ROOT):], it.primPath, it.layerOffset)
                count += 1
            out.append(it)
        return out

    def visit(path):
        spec = layer.GetObjectAtPath(path)
        if not isinstance(spec, Sdf.PrimSpec):
            return
        for lst, make in ((spec.payloadList, Sdf.Payload), (spec.referenceList, Sdf.Reference)):
            for name in ("prependedItems", "appendedItems", "explicitItems"):
                items = list(getattr(lst, name))
                if any(i.assetPath.startswith(CLOUD_ROOT) for i in items):
                    setattr(lst, name, fix(items, make))

    with Sdf.ChangeBlock():
        layer.Traverse(Sdf.Path.absoluteRootPath, visit)
    return count


def localize_cloud_assets(stage):
    """localize_layer() on the root layer of an open stage."""
    return localize_layer(stage.GetRootLayer())
