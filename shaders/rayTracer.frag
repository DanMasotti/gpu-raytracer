#version 410 core
#define MAX_VAL  10000.0
#define MIN_VAL 0.00001
#define PI 3.1415926535897932384626433832795
#define PLANE   0
#define SPHERE  1
#define BOX 2

/*
*	IN From quad.vert
*/
in vec2 texCoord;
in vec4 position;

/*
*	Uniforms from C++
*/
uniform sampler2D tex;
//uniform mat4 M_film2World;
//uniform vec4 eye;
uniform int depth;
//uniform int width;
//uniform int height;
uniform float time;
uniform vec2 dimensions;


/*
*	Out to frame buffer
*/
out vec4 fragColor;


/*
*	Structs for scene represention
*/
struct Ray {
	vec4 P; 
	vec4 d;

	int type;
};


struct Material {
	vec4 diffuseColor;
	vec4 specularColor;
	vec4 reflectedColor;
	vec4 tranparencyColor;
};

struct Sphere {
	mat4 transformation;

	Material mat;
};

// Ground
struct Plane {
	vec4 point;
	vec4 normal;

	Material mat;
};


struct HitData {
	bool isIntersect;
	vec4 normal;
	Material mat;

	int rayType[];
	float tVals[];
};


struct Light {
	vec4 position;
	vec4 color;
};

/*
*	Scene representation
*
*	TODO: artistic vision
*/
// Three point lighting: key light: (2, 2, 2), brightest | fill light (-1, 0, 1) | backlight (0, 3, -2)
Light sceneLighting[]  = Light[](
								Light( vec4(2.f, 2.f, 2.f, 1.f), vec4(1.f, 1.f, 1.f, 1.f) ),
								Light( vec4(-1.f, 0.f, 1.f, 1.f), vec4(1.f, 1.f, 1.f, 0.5f) ),
								Light( vec4(0.f, 3.f, -2.f, 1.f), vec4(0.2f, 0.2f, 0.6f, 0.2f) )
								);

// 3 spheres: 
Material eggShell = Material(
							vec4(240.f, 234.f, 214.f, 255.f)/255.f, 
							vec4(1.f, 1.f, 1.f, 1.f), 
							vec4(1.f, 1.f, 0.9f, 1.f), 
							vec4(0)
							);
Material salmon = Material(
							vec4(250.f, 128.f, 114.f, 255.f)/255.f, 
							vec4(1.f, 1.f, 1.f, 1.f), 
							vec4(1.f, 1.f, 0.9f, 1.f), 
							vec4(0)
							);

Material forest = Material(
							vec4(34.f, 139.f, 34.f, 255.f)/255.f, 
							vec4(1.f, 1.f, 1.f, 1.f), 
							vec4(1.f, 1.f, 0.9f, 1.f), 
							vec4(0)
							);
// TODO: animate here? Hard coded translation scale 
mat4 leftSphereTransformation = transpose(mat4(
									0.25f, 0.f, 0.f, 1.f,
									0.f, 0.25f, 0.f, 0.f,
									0.f, 0.f, 0.25f, -1.f,
									0.f, 0.f, 0.f, 1.f
									));

mat4 rightSphereTransformation = transpose(mat4(
										1.25f, 0.f, 0.f, 0.5f,
										0.f, 1.25f, 0.f, 0.f,
										0.f, 0.f, 1.25f, -1.f,
										0.f, 0.f, 0.f, 1.f
									));


mat4 centerSphereTransformation = mat4(1.f);


Sphere leftSphere = Sphere(leftSphereTransformation, salmon);
Sphere rightSphere = Sphere(rightSphereTransformation, forest);
Sphere centerSphere = Sphere(mat4(1.f), eggShell);

Sphere sceneSpheres[] = Sphere[](
								centerSphere,
								rightSphere,
								leftSphere
								);

/*
*	Ray tracer code
*/

// TODO: shadows
int checkOcclusions(inout Light light) {

	return 0;
}


// TODO: reflections and refractions, no recursion, just use secondary rays unless the fancy wavefront pattern
vec4 estimateIndirectLight(inout Ray ray, inout HitData data) {

	return vec4(0.f);
}


// TODO: diffuse and specular
vec4 estimateDirectLight(inout Ray ray, inout HitData data) {
	vec4 radiance = vec4(0.f, 0.f, 0.f, 1.f);

	for (int i = 0; i < 3; i ++){
		Light light = sceneLighting[i];

		vec4 lightPosition = light.position;
		vec4 vertex = ray.P + data.tVals[0]*ray.d;
		vec4 n = data.normal;

		vec4 vertexToLight = lightPosition - vertex;

		float dist = pow(-vertexToLight.x, 2.f) + pow(-vertexToLight.y, 2.f) + pow(-vertexToLight.z, 2.f);
		float denom = 1.f + dist + pow(dist, 2.f);
		float attenuation = min(1.f, 1.f/denom);

		float cosTheta =  max(0.f, dot(normalize(n), vertexToLight));
		vec4 diffuseComponent = attenuation*data.mat.diffuseColor*cosTheta;

		vec4 reflected = -normalize(2.f*n*(dot(n, vertexToLight)) - vertexToLight);
		float cosPhi = max(0.f, dot(reflected, ray.d));
		vec4 specularComponent = data.mat.specularColor*pow(cosPhi, 1);

		radiance += light.color*(diffuseComponent + specularComponent);

		radiance.x = min(max(radiance.x, 0.f), 1.f);
		radiance.y = min(max(radiance.y, 0.f), 1.f);
		radiance.z = min(max(radiance.z, 0.f), 1.f);
	}

	return radiance;
}


vec2 getQuadradicRoots(float A, float B, float C) {
	float det = B*B - 4*A*C;
	
	if (det > 0) {
		float t1 = (-B + sqrt(det))/(2.f*A);
		float t2 = (-B - sqrt(det))/(2.f*A);

		if (t1 < 0.f) {
			t1 = MAX_VAL;
		} 
		if (t2 < 0.f) {
			t2 = MAX_VAL;
		}
		return vec2(t1, t2);
	}
	return vec2(MAX_VAL, MAX_VAL);
}


HitData sphereRayIntersect(inout Sphere sphere, inout Ray ray) {
	vec4 P = ray.P;
	vec4 d = ray.d;

	float R = 0.5f;

	float A = pow(d.x, 2.f) + pow(d.y, 2.f) + pow(d.z, 2.f);
	float B = 2.f*(P.x*d.x + P.y*d.y + P.z*d.z);
	float C = pow(P.x, 2.f) + pow(P.y, 2.f) + pow(P.z, 2.f) - R*R;

	vec2 tVals = getQuadradicRoots(A, B, C);

	HitData data;
	data.isIntersect = false; // false until otherwise proven true
	data.mat = sphere.mat;
	


	data.tVals[0] = min(tVals.x, tVals.y);
	data.normal = normalize(P + data.tVals[0]*d);
	return data;
}


// TODO: how to represent the scene, I'm thinking hardcoded SDF like lab 10
HitData intersect(inout Ray ray) {
	// TODO: restore
	HitData data; // blank data
	data.tVals[0] = MAX_VAL;
	data.normal = vec4(0.f, 0.f, 0.f, 0.f);
	vec4 darkness = vec4(0.f, 0.f, 0.f, 0.f);
	data.mat = Material(darkness, darkness, darkness, darkness);

	float t = MAX_VAL;

	for (int i = 0; i < 3; i++){ // for each Sphere
		// TODO: remove, simple sphere ray intersect test
		Sphere sphere = sceneSpheres[i];
		mat4 transformation = sphere.transformation;
		Ray rayInObjectSpace = Ray(
									inverse(transformation)*ray.P,
									inverse(transformation)*ray.d,
									1
									);

		HitData retrieved = sphereRayIntersect(sphere, rayInObjectSpace);

		float t_prime = retrieved.tVals[0]; // first 

		if (t_prime < t) {
			t = t_prime;
			retrieved.isIntersect = true;
			mat4 MtInv = inverse(transpose(transformation));
			retrieved.normal = MtInv*retrieved.normal;
			data = retrieved;
		}
	}
	return data;
}


// inout is how you pass by ref in glsl
vec4 traceRay(inout Ray ray) {
	HitData data = intersect(ray);

	bool isIntersect = data.isIntersect;
	vec4 radiance = vec4(0.f, 0.f, 0.f, 1.f);
	// TODO: restore when ready
	if (isIntersect) {
		radiance = estimateDirectLight(ray, data) + estimateIndirectLight(ray, data);
	}
	return radiance;
}


/*
*	Camera stuff, big TODO
*/
vec4 pos = vec4(2.f, 2.f, 2.f, 1.f);

float near = 0.1f;
float far = 30.f;
float c = -near/far;
mat4 projection = transpose(mat4(
                                 1.f, 0.f, 0.f, 0.f,
                                 0.f, 1.f, 0.f, 0.f,
                                 0.f, 0.f, -1.f/(c + 1.f), c/(c + 1.f),
                                 0.f, 0.f, -1.f, 0.f
                                 ));
float thetaH = 60.f;
float aspectRatio = dimensions.x/dimensions.y;
float h = 1.f/(far * tan(radians(thetaH/2.f)));
float w = h/aspectRatio;
mat4 scaleMatrix = transpose(mat4(
								w, 0.f, 0.f, 0.f,
								0.f, h, 0.f, 0.f,
								0.f, 0.f, 1.f/far, 0.f,
								0.f, 0.f, 0.f, 1.f
								));

mat4 translationMatrix = transpose(mat4(
										1.f, 0.f, 0.f, -pos.x,
										0.f, 1.f, 0.f, -pos.y,
										0.f, 0.f, 1.f, -pos.z,
										0.f, 0.f, 0.f, 1.f
										));
vec4 up = vec4(0.f, 1.f, 0.f, 0.f);
vec4 w_rot = vec4(1.f, 1.f, 1.f, 0.f);
vec4 v_rot = vec4(normalize(up - dot(up.xyz, w_rot.xyz)*w_rot));
vec4 u_rot = vec4(cross(v_rot.xyz, w_rot.xyz), 0.f);
mat4 rotationMatrix = transpose(mat4(
                                     u_rot.x, u_rot.y, u_rot.z, 0.f,
                                     v_rot.x, v_rot.y, v_rot.z, 0.f,
                                     w_rot.x, w_rot.y, w_rot.z, 0.f,
                                     0.f, 0.f, 0.f, 1.f
                                     ));

mat4 viewMatrix = rotationMatrix*translationMatrix;
mat4 M_film2World = inverse(scaleMatrix*viewMatrix);


/*
*  Driver code
*/

// TODO: Use timer for animation
void main() {
	fragColor = vec4(0.f, 0.f, 0.f, 1.f);
	float x = position.x; //in film
	float y = position.y; //in film

	// TODO: why isnt this working (perspective)	
	vec4 pt_film = vec4(x, y, -1.f, 1.f);
	vec4 pt_world = M_film2World*pt_film;

	vec4 d = vec4(0.f, 0.f, -1.f, 0.f);
	vec4 P = vec4(x, y, 0.f, 1.f);
//	vec4 eye = M_film2World*vec4(0.f, 0.f, 0.f, 1.f);
//	vec4 P = M_film2World*eye;
//	fragColor = P;
//	vec4 d = normalize(pt_world - eye);

	Ray primaryRay = Ray(P, d, 1);

	fragColor += traceRay(primaryRay);

	//fragColor += d*0.5f + 0.5f;
}
