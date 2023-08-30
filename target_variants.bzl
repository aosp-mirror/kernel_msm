la_targets = [
    # keep sorted
    "monaco",
]

la_variants = [
    # keep sorted
    "consolidate",
    "gki",
]

vm_types = [
]

vm_target_bases = [
]

vm_targets = ["{}-{}".format(t, vt) for t in vm_target_bases for vt in vm_types]

vm_variants = [
]

def get_all_la_variants():
    return [(t, v) for t in la_targets for v in la_variants]

def get_all_variants():
    return get_all_la_variants()
