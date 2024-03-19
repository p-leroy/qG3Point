attribute vec3 vertexIn;
attribute vec3 semiAxisLengths;

/*out Vertex
{
  vec3 semiAxisLengths;
} vertex;*/

uniform mat4 modelViewProjectionMatrix;
uniform vec3 center;

void main(void)
{
	gl_PointSize = 5;
	gl_Position = modelViewProjectionMatrix * vec4(vertexIn + center, 1.0);
	//vertex.semiAxisLengths = semiAxisLengths;
}
