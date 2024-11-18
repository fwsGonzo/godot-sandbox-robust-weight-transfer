extends Sandbox

const program: ELFScript = preload("../robust_weight_transfer.elf")
func _ready() -> void:
	set_program(program)
	set_redirect_stdout(_print)
	vmcall("run_tests")
	var mesh_source: BoxMesh = BoxMesh.new()
	var mesh_target: SphereMesh = SphereMesh.new()
	var source_arrays: Array = mesh_source.surface_get_arrays(0)
	var target_arrays: Array = mesh_target.surface_get_arrays(0)
	
	var matched: Array = []
	var target_weights: Array = []
	vmcall("find_matches_closest_surface_mesh", source_arrays, target_arrays, 8, matched, target_weights)
	print(matched.size())
	print(target_weights.size())

func _print(line) -> void:
	print(line)
