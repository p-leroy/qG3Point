#version 330

// uniforms
uniform mat4 modelViewMatrix;
uniform mat4 normalMatrix;
uniform mat4 modelViewProjectionMatrix;
uniform vec3 center;

// vertex attribs (input)
in vec3 vertexPosition;
in vec3 vertexNormal;
in vec2 vertexTexCoord;

// varyings (output)
out vec3 esVertex;
out vec3 esNormal;
out vec2 texCoord0;

void main()
{
    esVertex = vec3(modelViewMatrix * vec4(vertexPosition + center, 1.0));
    esNormal = vec3(normalMatrix * vec4(vertexNormal, 1.0));
    texCoord0 = vertexTexCoord;
	gl_PointSize = 5;
	gl_Position = modelViewProjectionMatrix * vec4(vertexPosition + center, 1.0);
}
