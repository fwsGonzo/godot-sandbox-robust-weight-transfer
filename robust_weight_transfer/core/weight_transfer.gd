extends Sandbox

const program: ELFScript = preload("../robust_weight_transfer.elf")

func _ready() -> void:
	set_program(program)
	set_redirect_stdout(_print)
	# vmcall("run_tests")
	var args: Dictionary = {
		"angle_threshold_degrees": 10.0,
		"distance_threshold": 1.0
	}
	var quadmesh: QuadMesh = QuadMesh.new()
	var spheremesh: QuadMesh = QuadMesh.new()
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
	args["vertices_1"] = vertices_1
	args["faces_1"] = faces_1
	args["normals_1"] = normals_1
	args["vertices_2"] = vertices_2
	args["faces_2"] = faces_2
	args["normals_2"] = normals_2
	var skin_weights: Array = []
	for vertex in args["vertices_1"]:
		skin_weights.append([1.0, 0.0])
	args["skin_weights"] = skin_weights
	vmcall("robust_weight_transfer", args)

func _print(line) -> void:
	print(line)
