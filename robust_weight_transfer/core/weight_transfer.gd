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
	var quadmesh_arrays: Array = quadmesh.surface_get_arrays(0)
	var spheremesh_arrays: Array = spheremesh.surface_get_arrays(0)
	var vertices_1: Array = []
	for i in range(quadmesh_arrays[Mesh.ARRAY_VERTEX].size()):
		vertices_1.append(quadmesh_arrays[Mesh.ARRAY_VERTEX][i])
	var faces_1: Array = []
	for i in range(quadmesh_arrays[Mesh.ARRAY_INDEX].size()):
		faces_1.append(quadmesh_arrays[Mesh.ARRAY_INDEX][i])
	var normals_1: Array = []
	for i in range(quadmesh_arrays[Mesh.ARRAY_NORMAL].size()):
		normals_1.append(quadmesh_arrays[Mesh.ARRAY_NORMAL][i])
	var vertices_2: Array = []
	for i in range(spheremesh_arrays[Mesh.ARRAY_VERTEX].size()):
		vertices_2.append(spheremesh_arrays[Mesh.ARRAY_VERTEX][i])
	var faces_2: Array = []
	for i in range(spheremesh_arrays[Mesh.ARRAY_INDEX].size()):
		faces_2.append(spheremesh_arrays[Mesh.ARRAY_INDEX][i])
	var normals_2: Array = []
	for i in range(spheremesh_arrays[Mesh.ARRAY_NORMAL].size()):
		normals_2.append(spheremesh_arrays[Mesh.ARRAY_NORMAL][i])
	inputs["vertices_1"] = vertices_1
	inputs["faces_1"] = faces_1
	inputs["normals_1"] = normals_1
	inputs["vertices_2"] = vertices_2
	inputs["faces_2"] = faces_2
	inputs["normals_2"] = normals_2
	var skin_weights: Array = []
	for vertex in inputs["vertices_1"]:
		skin_weights.append([1.0, 0.0])
	inputs["skin_weights"] = skin_weights
	inputs["verbose"] = false
	var matched_array: Array = []
	var interpolated_weights_array: Array = []
	var inpainted_weights_array: Array = []
	var smoothed_weights_array: Array = []
	vmcall("robust_weight_transfer", inputs, matched_array, interpolated_weights_array, inpainted_weights_array, smoothed_weights_array)
	print("matched_array: ", matched_array.size())
	print("interpolated_weights_array: ", interpolated_weights_array.size())
	print("inpainted_weights_array: ", inpainted_weights_array.size())
	print("smoothed_weights_array: ", smoothed_weights_array.size())

func _print(line) -> void:
	print(line)
