shader_type spatial;

uniform vec3 color_a : source_color = vec3(0.5);
uniform vec3 color_b : source_color = vec3(0.42);
uniform vec3 line_color : source_color = vec3(0.7);
uniform sampler2D line_mask : hint_default_black, filter_linear_mipmap_anisotropic, repeat_enable;
uniform float check_size = 1.0;
uniform float roughness : hint_range(0.0, 1.0) = 0.85;

varying vec3 world_position;
varying vec3 world_normal;

void vertex() {
	world_position = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	world_normal = normalize((MODEL_MATRIX * vec4(NORMAL, 0.0)).xyz);
}

float checker(vec2 p) {
	vec2 q = p / check_size;
	vec2 w = fwidth(q) + 0.001;
	vec2 i = 2.0 * (abs(fract((q - 0.5 * w) * 0.5) - 0.5) - abs(fract((q + 0.5 * w) * 0.5) - 0.5)) / w;
	return 0.5 - 0.5 * i.x * i.y;
}

float lines(vec2 p) {
	return texture(line_mask, p / (check_size * 2.0)).r;
}

void fragment() {
	vec3 w = pow(abs(world_normal), vec3(8.0));
	w /= dot(w, vec3(1.0));
	vec3 p = world_position;
	float c = checker(p.zy) * w.x + checker(p.xz) * w.y + checker(p.xy) * w.z;
	float m = lines(p.zy) * w.x + lines(p.xz) * w.y + lines(p.xy) * w.z;
	ALBEDO = mix(mix(color_a, color_b, c), line_color, m);
	ROUGHNESS = roughness;
}
