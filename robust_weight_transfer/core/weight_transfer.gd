extends Sandbox

const program: ELFScript = preload("../robust_weight_transfer.elf")

func _ready() -> void:
	set_program(program)
	set_redirect_stdout(_print)
	# vmcall("run_tests")
	var inputs: Dictionary = {
		"angle_threshold_degrees": 10.0,
		"distance_threshold": 1.0
	}
	var quadmesh: QuadMesh = QuadMesh.new()
	var spheremesh: SphereMesh = SphereMesh.new()
	var skin_weights: Array = []
	for vertex in quadmesh.get_mesh_arrays()[Mesh.ARRAY_VERTEX]:
		skin_weights.append(PackedFloat32Array([1.0, 0.0]))
	inputs["skin_weights"] = skin_weights
	inputs["verbose"] = false
	var matched_array: Array = []
	var interpolated_weights_array: Array = []
	var inpainted_weights_array: Array = []
	var smoothed_weights_array: Array = []
	vmcall("robust_weight_transfer", quadmesh, spheremesh, inputs, matched_array, interpolated_weights_array, inpainted_weights_array, smoothed_weights_array)
	print("matched_array: ", matched_array.size())
	print("interpolated_weights_array: ", interpolated_weights_array.size())
	print("inpainted_weights_array: ", inpainted_weights_array.size())
	print("smoothed_weights_array: ", smoothed_weights_array.size())

func _print(line) -> void:
	print(line)
