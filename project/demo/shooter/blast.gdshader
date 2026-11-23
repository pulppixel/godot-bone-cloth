shader_type spatial;
render_mode unshaded, depth_prepass_alpha, cull_back;

uniform sampler2D noise : filter_linear_mipmap, repeat_enable;
uniform sampler2D heat_ramp : source_color, filter_linear, repeat_disable;
uniform float heat_energy = 3.0;
uniform vec3 smoke_color : source_color = vec3(0.3);
uniform float bumps = 0.2;
uniform float heat_start = 1.0;
uniform float cooling = 1.4;
uniform float dissolve_from = 0.4;
uniform float softness = 0.15;

varying float life;
varying vec2 seed;


float bump(vec2 uv) {
	return textureLod(noise, uv * vec2(2.0, 1.0) + seed + vec2(0.0, life * 0.3), 0.0).r;
}


void vertex() {
	life = clamp(INSTANCE_CUSTOM.y / max(INSTANCE_CUSTOM.w, 0.001), 0.0, 1.0);
	seed = vec2(cos(INSTANCE_CUSTOM.x), sin(INSTANCE_CUSTOM.x)) * 0.5;
	VERTEX += NORMAL * (bump(UV) - 0.5) * bumps;
}


void fragment() {
	float n = bump(UV);
	float facing = clamp(dot(NORMAL, VIEW), 0.0, 1.0);
	float up = (INV_VIEW_MATRIX * vec4(NORMAL, 0.0)).y;
	float light = mix(0.35, 1.0, smoothstep(-0.7, 0.9, up + (n - 0.5) * 0.6));
	float heat = clamp(heat_start - life * cooling + (n - 0.5) * 0.6 + (facing - 0.5) * 0.4, 0.0, 1.0);
	vec3 fire = texture(heat_ramp, vec2(heat, 0.0)).rgb * heat_energy;
	ALBEDO = mix(smoke_color * light, fire, smoothstep(0.05, 0.35, heat));
	float cut = smoothstep(dissolve_from, 1.0, life);
	ALPHA = smoothstep(cut - softness, cut + softness, n * mix(0.4, 1.0, facing));
}
