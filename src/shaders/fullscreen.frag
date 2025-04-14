#version 330 core

in vec2 vUV;
out vec4 FragColor;

uniform sampler2D screenImage;

void main() {
    vec3 color = texture(screenImage, vUV).rgb;
    FragColor = vec4(color, 1.0);
    // FragColor = vec4(vUV, 0.0, 1.0); 
}
