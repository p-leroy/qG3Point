#version 330

// uniforms
uniform vec4 lightPosition;             // should be in the eye space
uniform vec4 lightAmbient;              // light ambient color
uniform vec4 lightDiffuse;              // light diffuse color
uniform vec4 lightSpecular;             // light specular color
uniform vec4 materialAmbient;           // material ambient color
uniform vec4 materialDiffuse;           // material diffuse color
uniform vec4 materialSpecular;          // material specular color
uniform float materialShininess;        // material specular shininess
uniform bool drawLines;
uniform bool drawPoints;

// in
in vec3 esVertex;
in vec3 esNormal;

// out
out vec4 fragColor;

void main()
{
	if (drawPoints)
	{
		fragColor = vec4(materialAmbient.rgb * vec3(0.8, 0.8, 0.8), materialDiffuse.a);
	}
	else
	{
		vec3 normal = normalize(esNormal);
		vec3 light;
		if(lightPosition.w == 0.0)
		{
			light = normalize(lightPosition.xyz);
		}
		else
		{
			light = normalize(lightPosition.xyz - esVertex);
		}
		vec3 view = normalize(-esVertex);
		vec3 reflectVec = reflect(-light, normal);  // 2 * N * (N dot L) - L
	
		vec3 color = lightAmbient.rgb * materialAmbient.rgb;        // begin with ambient
		float dotNL = max(dot(normal, light), 0.0);
		color += lightDiffuse.rgb * materialDiffuse.rgb * dotNL;    // add diffuse
		float dotVR = max(dot(view, reflectVec), 0.0);
		color += pow(dotVR, materialShininess) * lightSpecular.rgb * materialSpecular.rgb; // add specular
	
		// set frag color
		if (drawLines)
		{
			fragColor = vec4(materialAmbient.rgb * vec3(0.8, 0.8, 0.8), materialDiffuse.a);
		}
		else
		{
			//fragColor = vec4(materialAmbient.rgb, materialDiffuse.a);
			fragColor = vec4(color, materialDiffuse.a);
		}
	}
}
