$FG_GLSL_VERSION

// The cockpit screen, in the HDR forward pass: shows a render view's texture (/sim/rendering/render-views, a FlightGear with the render-view patch).
// For the effect Models/Effects/RenderView/al3-screen (fdm/renderviews.py); written by hand.
//   mode 2  a camera: the texture at the screen's texture coordinates
//   mode 3  the mirror's camera: the same, flipped left to right as a mirror reads, cut top and bottom to the screen's shape if its picture is taller

in VS_OUT {
    float flogz;
    vec2 texcoord;
    vec3 vertex_normal;
    vec3 view_vector;
    vec4 material_color;
    vec4 ap_color;
} fs_in;

uniform sampler2D render_view_tex;
uniform int render_view_mode;
uniform float render_view_gain;             // the screen's brightness
uniform float screen_aspect;                // the screen's height over its width
uniform float exposure_compensation;

// exposure.glsl
vec3 apply_exposure(vec3 color);
// logarithmic_depth.glsl
float logdepth_encode(float z);
// forward_pack.glsl
void forward_pack(vec4 color, float flogz);

void main()
{
    vec2 uv = fs_in.texcoord;
    if (render_view_mode == 3) {
        vec2 size = vec2(textureSize(render_view_tex, 0));
        uv = vec2(1.0 - uv.x, 0.5 + (uv.y - 0.5) * screen_aspect * size.x / size.y);
    }
    // the render view is pre-exposed by the exposure compensation alone (its automatic exposure stays 0 EV): take that off, and expose it as the main view
    vec3 radiance = textureLod(render_view_tex, clamp(uv, 0.0, 1.0), 0.0).rgb / exp2(exposure_compensation);
    vec3 color = apply_exposure(radiance * render_view_gain);

    forward_pack(vec4(color, 1.0), fs_in.flogz);
    gl_FragDepth = logdepth_encode(fs_in.flogz);
}
