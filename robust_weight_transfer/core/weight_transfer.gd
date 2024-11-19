extends Sandbox

const program: ELFScript = preload("../robust_weight_transfer.elf")


func _ready() -> void:
	set_program(program)
	set_redirect_stdout(_print)
	# vmcall("run_tests")
	var inputs: Dictionary = {
		"angle_threshold_degrees": 10.0,
		"distance_threshold": 10.0
	}
	var vrm_body: MeshInstance3D = $VRM1_Constraint_Twist_Sample/Root_/GeneralSkeleton/Body
	var source_mesh: ArrayMesh = vrm_body.mesh
	var target_mesh: SphereMesh = SphereMesh.new()
	var skin_weights: Array = []
	var surface_index: int = 0
	inputs["verbose"] = false
	inputs["source_mesh_surface"] = surface_index
	inputs["target_mesh_surface"] = surface_index
	var matched_array: Array = []
	var interpolated_weights_array: Array = []
	var inpainted_weights_array: Array = []
	var smoothed_weights_array: Array = []
	vmcall("robust_weight_transfer", source_mesh, target_mesh, inputs, matched_array, interpolated_weights_array, inpainted_weights_array, smoothed_weights_array)
	print("matched_array: ", matched_array)
	print("interpolated_weights_array: ", interpolated_weights_array.size())
	print("inpainted_weights_array: ", inpainted_weights_array.size())
	print("smoothed_weights_array: ", smoothed_weights_array.size())

func _print(line) -> void:
	print(line)
