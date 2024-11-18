extends Sandbox

const program: ELFScript = preload("../robust_weight_transfer.elf")

func _ready() -> void:
	set_program(program)
	set_redirect_stdout(_print)
	vmcall("run_tests")
	var mesh_source: QuadMesh = QuadMesh.new()
	var mesh_target: QuadMesh = QuadMesh.new()
	var source_surface_index: int = 0
	var target_surface_index: int = 0
	var source_arrays: Array = mesh_source.surface_get_arrays(source_surface_index)
	var target_arrays: Array = mesh_target.surface_get_arrays(target_surface_index)
	var r_matched: Array = []
	var r_target_weights: Array = []
	var distance_threshold_squared: float = 0.1
	var move_distance: float = 1.0
	for i in range(source_arrays[Mesh.ARRAY_VERTEX].size()):
		source_arrays[Mesh.ARRAY_VERTEX][i] += Vector3(move_distance, 0, 0)
	vmcall("find_matches_closest_surface_mesh", distance_threshold_squared, source_arrays, target_arrays, r_matched, r_target_weights, 5.0)
	print(r_matched)
	print(r_target_weights)


func _print(line) -> void:
	print(line)
