$FG_GLSL_VERSION

// The cockpit glass for the HDR pipeline, for Models/Effects/Glass/al3-glass.eff. FlightGear's model_transparent.frag lights a pane as a surface, with the sun and the shadow map: a pane lit by
// the sun through the other windows and cut by the shadows of the frame, which clear glass does not show. This one has only what a
// clear pane has: what is behind it, a little darkened (the tint: glass_alpha), and the reflection of its surroundings, the sky and
// the ground (the image-based light), by Fresnel: 4 % face on to all of it at a grazing angle. No sun term, no shadow lookup, no
// diffuse colour. The reflection only adds light: it is not also taken off what is behind the pane.
//
// The panes seen from inside reflect what HDR's light probe has for the cabin's side, no picture of the cabin of their own.

in VS_OUT {
    float flogz;
    vec2 texcoord;
    vec3 vertex_normal;
    vec3 view_vector;
    vec4 material_color;
    vec4 ap_color;
} fs_in;

uniform float pbr_roughness;
uniform float glass_alpha;          // the pane's own tint: how much of the view it takes (0.12: 88 % of it passes)

FG_VIEW_GLOBAL
uniform mat4 fg_ViewMatrixInverse[FG_NUM_VIEWS];

// the reflection's faint grey-green cast, the old glass's reflectance (0.60, 0.67, 0.64) relative to its largest
const vec3 REFLECT_TINT = vec3(0.60, 0.67, 0.64) / 0.67;

// ibl.glsl
vec3 get_reflected(vec3 N, vec3 V, mat4 view_matrix_inverse);
vec3 eval_ibl(vec3 base_color,
              float metallic,
              float roughness,
              vec3 f0,
              float occlusion,
              vec3 ws_N, vec3 ws_refl, float NdotV);
// aerial_perspective.glsl
vec3 mix_aerial_perspective(vec3 color, vec4 ap);
// exposure.glsl
vec3 apply_exposure(vec3 color);
// logarithmic_depth.glsl
float logdepth_encode(float z);
// forward_pack.glsl
void forward_pack(vec4 color, float flogz);

void main()
{
    vec3 N = normalize(fs_in.vertex_normal);
    vec3 V = normalize(-fs_in.view_vector);
    // a pane is seen from either side: the normal faces the viewer
    if (dot(N, V) < 0.0)
        N = -N;
    mat4 vmi = fg_ViewMatrixInverse[FG_VIEW_ID];
    float NdotV = max(dot(N, V), 1e-4);

    // the reflection of the sky and the ground, already weighted by the Fresnel term
    vec3 ws_N = (vmi * vec4(N, 0.0)).xyz;
    vec3 ws_refl = get_reflected(N, V, vmi);
    vec3 refl = eval_ibl(vec3(0.0), 0.0, pbr_roughness, vec3(0.04), 1.0, ws_N, ws_refl, NdotV) * REFLECT_TINT;

    refl = mix_aerial_perspective(refl, fs_in.ap_color);
    refl = apply_exposure(refl);

    // the blend is colour * alpha + what is behind * (1 - alpha): with colour = reflection / alpha it comes out as the reflection
    // plus the view through the pane times (1 - tint)
    float alpha = clamp(glass_alpha, 0.02, 1.0);

    forward_pack(vec4(refl / alpha, alpha), fs_in.flogz);
    gl_FragDepth = logdepth_encode(fs_in.flogz);
}
