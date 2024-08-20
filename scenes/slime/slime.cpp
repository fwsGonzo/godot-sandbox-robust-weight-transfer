#include "api.hpp"

static float slime_speed = 50.0f;

extern "C" const Property prop0 {
	.name = "slime_speed",
	.type = Variant::FLOAT,
	.getter = []() -> Variant { return slime_speed; },
	.setter = [](Variant value) -> Variant { return slime_speed = value; },
	.default_value = Variant{slime_speed},
};

struct SlimeState {
	int direction = 1;
};
PER_OBJECT(SlimeState);

extern "C" Variant _physics_process(Variant delta) {
	if (is_editor())
		return {};

	Node2D slime(".");
	Node2D sprite("AnimatedSprite2D");
	auto& state = GetSlimeState(slime);

	Vector2 spd = Vector2(slime_speed, 0.0f) * float(delta) * state.direction;
	slime.set_position(slime.get_position() + spd);
	// Change direction if rays collide
	if (state.direction > 0 && Node("raycast_right")("is_colliding")) {
		state.direction = -1;
		sprite.set("flip_h", true);
		sprite("play", "spawn");
	} else if (state.direction < 0 && Node("raycast_left")("is_colliding")) {
		state.direction = 1;
		sprite.set("flip_h", false);
		sprite("play", "idle");
	}
	return {};
}
