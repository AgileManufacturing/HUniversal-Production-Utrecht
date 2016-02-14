#version 330
uniform vec3 fragColor;
uniform int shading;
in vec3 outNormal;
in vec3 testColor;
out vec4 outColor;
void main() {
    vec3 lightDirection = vec3(0,0,-1);
    float diffuseFactor = max(dot(-lightDirection,outNormal), 0);
	if(shading == 1){
		outColor = vec4(fragColor,1.0) * diffuseFactor;
	}else{
		outColor = vec4(fragColor,1.0);
	}
}