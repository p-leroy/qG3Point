#version 330

// uniforms
uniform mat4 modelViewMatrix;
uniform mat4 normalMatrix;
uniform mat4 modelViewProjectionMatrix;
uniform int instanceId;
uniform int pointSize;

// in
in vec3 vertexPosition;
in vec3 vertexNormal;
in vec2 vertexTexCoord;

// out
out vec3 esVertex;
out vec3 esNormal;
//out vec2 texCoord0;

void main()
{
	if (pointSize !=0)
	{
		gl_PointSize = pointSize;
		esVertex = vec3(modelViewMatrix * vec4(vertexPosition, 1.0));
		esNormal = vec3(normalMatrix * vec4(0, 0, 1, 1.0));
		//texCoord0 = vec2(0, 0);
	}
	else
	{
		esVertex = vec3(modelViewMatrix * vec4(vertexPosition, 1.0));
		esNormal = vec3(normalMatrix * vec4(vertexNormal, 1.0));
		//texCoord0 = vertexTexCoord;
	}
	
	gl_Position = modelViewProjectionMatrix * vec4(vertexPosition, 1.0);
	
}
