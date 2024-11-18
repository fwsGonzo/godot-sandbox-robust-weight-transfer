extends Sandbox

const program: ELFScript = preload("../robust_weight_transfer.elf")

func _ready() -> void:
	set_program(program)
	set_redirect_stdout(_print)
	vmcall("run_tests")
	
	var quad_mesh_source: QuadMesh = QuadMesh.new()
	var quad_mesh_target: QuadMesh = QuadMesh.new()
	var source_surface_index: int = 0
	var target_surface_index: int = 0
	var r_matched: Array = []
	var r_target_weights: Array = []
	vmcall("find_matches_closest_surface_mesh", quad_mesh_source, source_surface_index, quad_mesh_target, target_surface_index, r_matched, r_target_weights)


func _print(line) -> void:
	print(line)
