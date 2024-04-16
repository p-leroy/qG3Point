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

uniform vec3 objectColor;

// in
in vec3 esVertex;
in vec3 esNormal;

// out
out vec4 fragColor;

void main()
{

	vec3 fixedLightPosition; 
	vec3 lightColor;
	fixedLightPosition = vec3(1., 1., 0.);
	lightColor = vec3(0.8, 0.8, 0.8);
	
	//Ambient lighting
	float ambientStrength = 0.5;
	vec3 ambient = ambientStrength * lightColor;
	
	// Diffuse lighting
	vec3 norm = normalize(esNormal);
	vec3 lightDir = normalize(fixedLightPosition - esVertex);  
	float diff = max(dot(norm, lightDir), 0.0);
	vec3 diffuse = diff * lightColor;
	
	// Specular lighting
	float specularStrength = 0.5;
	vec3 viewPos = vec3(0, 0, 0);
	vec3 viewDir = normalize(viewPos - esVertex);
	vec3 reflectDir = reflect(-lightDir, norm);
	float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
	vec3 specular = specularStrength * spec * lightColor;  
	
	//vec3 result = (ambient + diffuse + specular) * objectColor;
	//vec3 result = (ambient + diffuse) * objectColor;
	vec3 result = objectColor;

	if (drawPoints)
	{
		fragColor = vec4(result * vec3(0.8, 0.8, 0.8), 1.);
	}
	else
	{
		if (drawLines)
		{
			fragColor =  vec4(result * vec3(0.8, 0.8, 0.8), 1.);
		}
		else
		{
			fragColor =  vec4(result, transparency);
		}
	}
}
