extends Sandbox

const program: ELFScript = preload("../robust_weight_transfer.elf")

func _ready() -> void:
	set_program(program)
	set_redirect_stdout(_print)
	# vmcall("run_tests")
	var args: Dictionary = {
		"vertices_1": [
			Vector3(0, 0, 0),
			Vector3(1, 0, 0),
			Vector3(0, 1, 0)
		],
		"faces_1": [
			[0, 1, 2]
		],
		"normals_1": [
			Vector3(0, 0, 1),
			Vector3(0, 0, 1),
			Vector3(0, 0, 1)
		],
		"skin_weights": [
			[1, 0],
			[0, 1],
			[0.5, 0.5]
		],
		"vertices_2": [
			Vector3(0.1, 0.1, 0),
			Vector3(2, 2, 2)
		],
		"faces_2": [
			[0, 1]
		],
		"normals_2": [
			Vector3(0, 0, 1),
			Vector3(1, 0, 0)
		],
		"angle_threshold_degrees": 10.0,
		"distance_threshold_squared": 1.0
	}
	vmcall("robust_weight_transfer", args)

func _print(line) -> void:
	print(line)
