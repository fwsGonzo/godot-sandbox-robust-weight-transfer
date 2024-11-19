extends Sandbox

const program: ELFScript = preload("../robust_weight_transfer.elf")

func _ready() -> void:
	set_program(program)
	set_redirect_stdout(_print)
	# vmcall("run_tests")
	var vertices_1 = [
		Vector3(0, 0, 0),
		Vector3(1, 0, 0),
		Vector3(0, 1, 0)
	]
	var faces_1 = [
		[0, 1, 2]
	]
	var normals_1 = [
		Vector3(0, 0, 1),
		Vector3(0, 0, 1),
		Vector3(0, 0, 1)
	]
	var skin_weights = [
		[1, 0],
		[0, 1],
		[0.5, 0.5]
	]
	var vertices_2 = [
		Vector3(0.1, 0.1, 0),
		Vector3(2, 2, 2)
	]
	var faces_2 = [
		[0, 1]
	]
	var normals_2 = [
		Vector3(0, 0, 1),
		Vector3(1, 0, 0)
	]
	var result = vmcallv("robust_weight_transfer", vertices_1, faces_1, normals_1, skin_weights, vertices_2, faces_2, normals_2, 0.01, 45.0)
	print(result)
	

func _print(line) -> void:
	print(line)
