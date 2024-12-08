extends Node3D

func _init() -> void:
	Sandbox.load_binary_translation("res://bintr.so")
	pass

func _ready() -> void:
	var inputs: Dictionary = {
		"angle_threshold_degrees": 10.0,
		"distance_threshold": 1.0
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

	var vm = get_node("Sandbox") as Sandbox_RobustWeightTransferRobustWeightTransfer
	if !vm.is_binary_translated():
		vm.try_compile_binary_translation("res://bintr", "clang-19", "", true, true)

	vm.run_tests()

	var t0 = Time.get_ticks_msec()
	vm.robust_weight_transfer(source_mesh, target_mesh, inputs, matched_array, interpolated_weights_array, inpainted_weights_array, smoothed_weights_array)
	var t1 = Time.get_ticks_msec()
	print("It took ", t1 - t0, " milliseconds to compute")

	var true_count = 0
	for value in matched_array:
		if value == true:
			true_count += 1
	print("matched_array true count: ", true_count, " / ", matched_array.size())
	var finite_interpolated = 0
	var finite_interpolated_samples = []
	for i in range(interpolated_weights_array.size()):
		if is_finite(interpolated_weights_array[i]):
			finite_interpolated += 1
			if finite_interpolated <= 5:  # Print up to 5 samples
				finite_interpolated_samples.append([i, interpolated_weights_array[i]])
	print("interpolated_weights_array: ", interpolated_weights_array.size(), " finite: ", finite_interpolated)
	print("finite interpolated samples: ", finite_interpolated_samples)

	var finite_inpainted = 0
	var finite_inpainted_samples = []
	for i in range(inpainted_weights_array.size()):
		if is_finite(inpainted_weights_array[i]):
			finite_inpainted += 1
			if finite_inpainted <= 5:  # Print up to 5 samples
				finite_inpainted_samples.append([i, inpainted_weights_array[i]])
	print("inpainted_weights_array: ", inpainted_weights_array.size(), " finite: ", finite_inpainted)
	print("finite inpainted samples: ", finite_inpainted_samples)

	var finite_smoothed = 0
	var finite_smoothed_samples = []
	for i in range(smoothed_weights_array.size()):
		if is_finite(smoothed_weights_array[i]):
			finite_smoothed += 1
			if finite_smoothed <= 5:  # Print up to 5 samples
				finite_smoothed_samples.append([i, smoothed_weights_array[i]])
	print("smoothed_weights_array: ", smoothed_weights_array.size(), " finite: ", finite_smoothed)
	print("finite smoothed samples: ", finite_smoothed_samples)
