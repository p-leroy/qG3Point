#version 120

// uniforms
uniform mat4 modelMatrix;
uniform mat4 modelViewMatrix;
uniform mat4 normalMatrix;
uniform mat4 modelViewProjectionMatrix;
uniform int instanceId;
uniform int pointSize;

// attributes
attribute vec3 vertexPosition;
attribute vec3 vertexNormal;
attribute vec2 vertexTexCoord;

// varyings
varying vec3 esVertex;
varying vec3 esNormal;
//varying vec2 texCoord0;

void main()
{
	if (pointSize !=0)
	{
		gl_PointSize = pointSize;
		esVertex = vec3(modelViewMatrix * vec4(vertexPosition, 1.0));
		esNormal = vec3(normalMatrix * vec4(0, 0, 1, 1.0));
	}
	else
	{
		esVertex = vec3(modelViewMatrix * vec4(vertexPosition, 1.0));
		esNormal = vec3(normalMatrix * vec4(vertexNormal, 1.0));
	}
	
	gl_Position = modelViewProjectionMatrix * vec4(vertexPosition, 1.0);
	
}
