$FG_GLSL_VERSION

// The rotor blur's shadow (Models/Effects/RotorBlur/rotor-blur.eff): Shaders/HDR/model_default_shadow.frag, but a
// fraction alpha x shadow_gain of the shadow map's texels cast (an ordered 4 x 4 dither, which the shadow filtering
// softens), alpha faded by the rotor's rpm as in rotor_blur.frag: a translucent object's partial shadow

layout(location = 0) out vec4 fragColor;

in VS_OUT {
    vec2 texcoord;
    float material_alpha;
} fs_in;

uniform sampler2D color_tex;
uniform float rotor_rpm;
uniform float fade_from;
uniform float fade_to;
uniform float shadow_gain;

const float BAYER[16] = float[16](0.0, 8.0, 2.0, 10.0, 12.0, 4.0, 14.0, 6.0, 3.0, 11.0, 1.0, 9.0, 15.0, 7.0, 13.0, 5.0);

void main()
{
    float alpha = texture(color_tex, fs_in.texcoord).a * fs_in.material_alpha
        * clamp((rotor_rpm - fade_from) / (fade_to - fade_from), 0.0, 1.0) * shadow_gain;
    ivec2 p = ivec2(gl_FragCoord.xy) & 3;
    if (alpha <= (BAYER[p.y * 4 + p.x] + 0.5) / 16.0)
        discard;
    fragColor = vec4(1.0);
}
