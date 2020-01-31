#version 150

in float distanceToCamera;
out vec4 fragColor;

void main() {
  fragColor = vec4(distanceToCamera, 0, 0, 1);
}