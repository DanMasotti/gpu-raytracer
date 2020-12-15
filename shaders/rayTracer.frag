#version 410 core
#define MAX_VAL  10000.f
#define MIN_VAL 0.001f
#define PI 3.1415926535897932384626433832795
#define ks 0.5f
#define kd 0.5f
#define kt 0.5f

/*
*	In From quad.vert
*/
in vec2 texCoord;
in vec4 position;

/*
*	Uniforms from C++
*/
uniform sampler2D tex;
uniform mat4 M_film2World;
uniform int depth;
uniform float time;
uniform vec2 dimensions;
uniform float leftSpeed;
uniform float rightSpeed;
uniform float centerSpeed;

/*
*	Out to frame buffer
*/
layout(location = 0) out vec4 fragColor;

////////////////////////////////////////////////////////////////////////////////////////////////
/*
*	STRUCTS
*/
//////////////////////////////////////////////////////////////////////////////////////////////
struct Ray {
	vec4 P; 
	vec4 d;
};

struct Material {
	vec4 diffuseColor;
	vec4 specularColor;
	vec4 reflectedColor;
	vec4 tranparencyColor;
	
	float shininess;
	vec3 ior;
	float alpha;
};

struct Data {
	bool isIntersect;
	vec4 normal;
	Material mat;
	float t;
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

struct Light {
	vec4 position;
	vec4 color;
};

///////////////////////////////////////////////////////////////////////////////////////////
/*
*	SCENE REPRESENTATION
*/
//////////////////////////////////////////////////////////////////////////////////////////
Light sceneLighting[]  = Light[](
								Light( vec4(10.f, 10.f, -10.f, 1.f), vec4(0.2f, 0.2f, 0.2f, 1.f) ), // key light
								Light( vec4(-10.f, -10.f, -10.f, 1.f), vec4(0.2f, 0.2f, 0.2f, 1.f) ), // rim light
								Light( vec4(10.f, 10.f, 10.f, 1.f), vec4(0.2f, 0.2f, 0.2f, 1.f) ) // back light
								);
//Light sceneLighting[]  = Light[](
//								Light( vec4(10.f, 10.f, -10.f, 1.f), vec4(0.2f, 0.2f, 0.2f, 1.f) ) // key light
//								);

Material foggyGlass = Material(
							vec4(0.3f, 0.3f, 0.3f, 1.f), 
							vec4(0.4f, 0.4f, 0.2f, 1.f), 
							vec4(0.1f, 0.1f, 0.1f, 1.f), 
							vec4(0.1f, 0.1f, 0.1f, 1.f), 
							0.5f,
							vec3(1.3f, 1.32f, 1.34f),
							0.1f
							);

Material salmon = Material(
							vec4(0.7f, 0.2f,  0.1f, 1.f), 
							vec4(0.2f, 0.2f, 0.2f, 1.f), 
							vec4(0.1f, 0.1f, 0.1f, 1.f), 
							vec4(0.1f, 0.1f, 0.1f, 1.f), 
							0.5f,
							vec3(1.5f, 1.6f, 1.6f),
							0.7f
							);

Material forest = Material(
							vec4(0.1f, 0.7f, 0.2f, 1.f),
							vec4(0.2f, 0.3f, 0.2f, 1.f), 
							vec4(0.3f, 0.3f, 0.2f, 1.f), 
							vec4(0.2f, 0.2f, 0.2f, 1.f), 
							0.2f,
							vec3(2.f),
							0.9f
							);	
							
float smallRadius = 0.25f;
mat4 leftSphereTransformation = transpose(mat4(
  									smallRadius, 0.f, 0.f, -0.5f,
  									0.f, smallRadius, 0.f, 0.5f*sin((1.f/leftSpeed)*time),
  									0.f, 0.f, smallRadius, 0.5f,
  									0.f, 0.f, 0.f, 1.f
  									));

float bigRadius = 1.1f;
mat4 rightSphereTransformation = transpose(mat4(
  										bigRadius, 0.f, 0.f, 0.5f,
  										0.f, bigRadius, 0.f, 0.01f*cos((1.f/rightSpeed)*time),
  										0.f, 0.f, bigRadius, 1.f,
  										0.f, 0.f, 0.f, 1.f
  									));

mat4 centerSphereTransformation = transpose(mat4(
  											1.f, 0.f, 0.f, 0.f,
  											0.f, 1.f, 0.f, -0.25f*sin((1.f/centerSpeed)*time),
  											0.f, 0.f, 1.f, -1.f,
  											0.f, 0.f, 0.f, 1.f
  										));

Sphere leftSphere = Sphere(leftSphereTransformation, salmon);
Sphere rightSphere = Sphere(rightSphereTransformation, forest);
Sphere centerSphere = Sphere(centerSphereTransformation, foggyGlass);

Sphere sceneSpheres[] = Sphere[](leftSphere, centerSphere, rightSphere);


/////////////////////////////////////////////////////////////////////////////////
/*
*	RAY TRACER
*/
////////////////////////////////////////////////////////////////////////////////
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


Data planeRayIntersection(inout Plane plane, inout Ray ray) {
	vec4 P = ray.P;
	vec4 d = ray.d;

	Data data;

	float denom = dot(plane.normal, d);
	if (abs(denom) > MIN_VAL) {
		data.t = dot(plane.point - P, plane.normal)/denom;
	}
	return data;
}


Data sphereRayIntersect(inout Sphere sphere, inout Ray ray) {
	vec4 P = ray.P;
	vec4 d = ray.d;

	float R = 0.5f;

	float A = pow(d.x, 2.f) + pow(d.y, 2.f) + pow(d.z, 2.f);
	float B = 2.f*(P.x*d.x + P.y*d.y + P.z*d.z);
	float C = pow(P.x, 2.f) + pow(P.y, 2.f) + pow(P.z, 2.f) - R*R;

	vec2 tVals = getQuadradicRoots(A, B, C);

	Data data;
	data.isIntersect = false; // false until otherwise proven true
	data.mat = sphere.mat;
	
	data.t = min(tVals.x, tVals.y);

	vec4 vertex = P + data.t*d;
	data.normal = vec4(normalize(vertex.xyz), 0.f);
	return data;
}


Data intersect(inout Ray ray) {
	Data data; // blank data
	data.t = MAX_VAL;
	data.normal = vec4(0.f, 0.f, 0.f, 0.f);
	vec4 darkness = vec4(0.f, 0.f, 0.f, 0.f);
	data.mat = Material(darkness, darkness, darkness, darkness, 0.f, vec3(0.f), 1.f);

	float t = MAX_VAL;

	for (int i = 0; i < 3; i++){ // for each Sphere
		Sphere sphere = sceneSpheres[i];
		mat4 transformation = sphere.transformation;

		vec4 P_tilde = inverse(transformation)*ray.P;
		vec4 d_tilde = inverse(transformation)*ray.d;

		Ray rayInObjectSpace = Ray(vec4(P_tilde.xyz, 1.f), vec4(d_tilde.xyz, 0.f));

		Data retrieved = sphereRayIntersect(sphere, rayInObjectSpace);

		float t_prime = retrieved.t; // first 

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


// Shadow handling
bool checkOcclusions(inout Ray ray, inout Data data, inout Light light) {
	bool isOccluded = false;

	vec4 lightPosition = light.position;
	vec4 vertex = ray.P + data.t*ray.d;
	vec4 vertexToLight = lightPosition  - vertex;
	Ray shadowRay = Ray(vertex + MIN_VAL*vertexToLight, vertexToLight);

	Data shadowData = intersect(shadowRay);

	if (shadowData.isIntersect) {
		vec4 occluderVertex = shadowRay.P + shadowData.t*shadowRay.d;
		float distanceFromOccluder = distance(occluderVertex, vertex);
		float distanceFromLight = distance(lightPosition, vertex);
		if (distanceFromOccluder < distanceFromLight) {
			isOccluded = true;
		}
	}

	return isOccluded;
}

// Radiance compute
vec4 computeLighting(inout Ray ray, inout Data data) {
	vec4 radiance = vec4(0.f, 0.f, 0.f, 0.f);

	for (int i = 0; i < 4; i++) { // for each light in the scene
		Light light = sceneLighting[i];

		if (checkOcclusions(ray, data, light)) {
			continue;
		}

		vec4 I = light.color;
		vec4 lightPosition = light.position;
		vec4 vertex = ray.P + data.t*ray.d;
		vec4 normal = data.normal;
		vec4 vertexToLight = lightPosition - vertex;

		float dist = distance(light.position, vertex);
		float denom = exp(dist*dist);
		float attenuation = min(1.f, 1.f/denom);
		
		float cosTheta =  min(1.f, max(0.f, dot(normalize(normal.xyz), normalize(vertexToLight.xyz))));
		vec4 diffuseComponent = attenuation*I*kd*data.mat.diffuseColor*cosTheta;

		vec4 reflected = -normalize(2.f*normal*(min(max(dot(normal, vertexToLight), 0.f), 1.f)) - vertexToLight);
		float cosPhi = min(1.f, max(0.f, dot(reflected.xyz, ray.d.xyz)));
		vec4 specularComponent = I*ks*data.mat.specularColor*pow(cosPhi, data.mat.shininess);

		vec4 ambientComponent = vec4(0.f);

		radiance += ambientComponent;
		radiance += diffuseComponent;
		radiance += specularComponent;

		radiance.x = min(max(radiance.x, 0.f), 1.f);
		radiance.y = min(max(radiance.y, 0.f), 1.f);
		radiance.z = min(max(radiance.z, 0.f), 1.f);
		radiance.w = min(max(radiance.w, 0.f), 1.f);
	}
	return radiance;
}


vec4 traceRays(inout Ray primaryRay) {

	vec4 radiance = vec4(0.f); // ambient lighting
	Data primaryData = intersect(primaryRay);
	if (primaryData.isIntersect) { // directLight
		radiance += computeLighting(primaryRay, primaryData);
	} 

	Ray currentRay = primaryRay;
	Data currentData = primaryData;

	for (int i = 0; i < depth; i++) { // How many layers we want 

		float t = currentData.t;
		vec4 vertex = currentRay.P + t*currentRay.d;

		vec4 normal = currentData.normal;
		vec4 v = currentRay.P + currentRay.d;

		vec3 ior = currentData.mat.ior;
		float ior_red = ior.x;
		float ior_green = ior.y;
		float ior_blue = ior.z;

		float proxy = (ior_red + ior_green + ior_blue)/3.f;

		float r0 = pow((1.f - proxy)/(1.f + proxy), 2.f); // assume everything else is just air
		float F = r0 + (1.f - r0)*pow((1.f - dot(normal, v)), 5.f);

		vec4 reflected = normalize(2.f*normal*(min(max(dot(normal, v), 0.f), 1.f)) - v);
		Ray reflectionRay = Ray(vertex + MIN_VAL*reflected, reflected);
		Data reflectionData = intersect(reflectionRay);

		if (reflectionData.isIntersect) {
			vec4 reflectionColor = reflectionData.mat.reflectedColor;
			radiance += max(vec4(0.f), F*(ks*reflectionColor*computeLighting(reflectionRay, reflectionData)));
		}

		vec4 p = vertex;
		vec4 n_p = currentData.normal;
		p = p + MIN_VAL*(-n_p);
		vec4 hitPoint = p + currentData.t*currentRay.d;
		
		/*
		* 
		*	Refract different wavelengths
		*	All red wavelength:
		*
		*/
		vec4 d_p_r = vec4(normalize(refract(n_p.xyz, hitPoint.xyz, ior_red)), 0.f);
		Ray throughRay_r = Ray(p, d_p_r);
		Data throughData_r = intersect(throughRay_r);

		vec4 q_r = p + throughData_r.t*d_p_r;
		vec4 n_q_r = throughData_r.normal;

		q_r = q_r + MIN_VAL*n_q_r;
		vec4 otherside_r = q_r + throughData_r.t*throughRay_r.d;

		vec4 d_q_r = vec4(normalize(refract(-n_q_r.xyz, otherside_r.xyz, 1.f/ior_red)), 0.f);
		Ray refractedRay_r = Ray(q_r, d_q_r);
		Data refractionData_r = intersect(refractedRay_r);

		/*
		* All green wavelength
		*
		*/
		vec4 d_p_g = vec4(normalize(refract(n_p.xyz, hitPoint.xyz, ior_green)), 0.f);
		Ray throughRay_g = Ray(p, d_p_g);
		Data throughData_g = intersect(throughRay_g);

		vec4 q_g = p + throughData_g.t*d_p_g;
		vec4 n_q_g = throughData_g.normal;

		q_g = q_g + MIN_VAL*n_q_g;
		vec4 otherside_g = q_g + throughData_g.t*throughRay_g.d;

		vec4 d_q_g = vec4(normalize(refract(-n_q_g.xyz, otherside_g.xyz, 1.f/ior_green)), 0.f);
		Ray refractedRay_g = Ray(q_g, d_q_g);
		Data refractionData_g = intersect(refractedRay_g);

		/*
		*  All blue wavelength
		*
		*/
		vec4 d_p_b = vec4(normalize(refract(n_p.xyz, hitPoint.xyz, ior_blue)), 0.f);
		Ray throughRay_b = Ray(p, d_p_b);
		Data throughData_b = intersect(throughRay_b);

		vec4 q_b = p + throughData_b.t*d_p_b;
		vec4 n_q_b = throughData_b.normal;

		q_b = q_b + MIN_VAL*n_q_b;
		vec4 otherside_b = q_b + throughData_b.t*throughRay_b.d;

		vec4 d_q_b = vec4(normalize(refract(-n_q_b.xyz, otherside_b.xyz, 1.f/ior_blue)), 0.f);
		Ray refractedRay_b = Ray(q_b, d_q_b);
		Data refractionData_b = intersect(refractedRay_b);

		if (false) {
			/*
			* Red
			*/
			vec4 retrievedLighting_r  = computeLighting(refractedRay_r, refractionData_r);
			vec4 transparencyColor_r = refractionData_r.mat.tranparencyColor;
			vec4 refractionColor_r = transparencyColor_r*retrievedLighting_r;

			/*
			* Green
			*/
			vec4 retrievedLighting_g  = computeLighting(refractedRay_g, refractionData_g);
			vec4 transparencyColor_g = refractionData_g.mat.tranparencyColor;
			vec4 refractionColor_g = transparencyColor_g*retrievedLighting_g;

			/*
			*  Blue
			*/
			vec4 retrievedLighting_b  = computeLighting(refractedRay_b, refractionData_b);
			vec4 transparencyColor_b = refractionData_b.mat.tranparencyColor;
			vec4 refractionColor_b = transparencyColor_b*retrievedLighting_b; 

			/*
			* Reassemble
			*/
			vec4 refractedContrib = kt*vec4(
											refractionColor_r.x,
											refractionColor_g.y,
											refractionColor_b.z,
											currentData.mat.alpha
										);

			radiance += refractedContrib;
		}

		currentRay = reflectionRay; // relfections and refractions of reflections
		currentData = reflectionData; // but not reflections and refractions of refractions
	}

	radiance.x = min(max(radiance.x, 0.f), 1.f);
	radiance.y = min(max(radiance.y, 0.f), 1.f);
	radiance.z = min(max(radiance.z, 0.f), 1.f);
	radiance.w = min(max(radiance.w, 0.f), 1.f);

	return radiance;
};

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
*  Driver code
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////
void main() {
	vec4 radiance = vec4(0.1f, 0.1f, 0.1f, 1.f);
	float x = position.x; //in film
	float y = position.y; //in film

//	vec4 d = vec4(0.f, 0.f, 1.f, 0.f); // Ortho
//	vec4 P = vec4(x, y, -1.f, 1.f);

	vec4 eye = M_film2World*vec4(0.f, 0.f, 0.f, 1.f);
	
	vec4 pt_film = vec4(x, y, -1.f, 1.f);

    vec4 pt_world = M_film2World*pt_film;

    vec4 d = normalize(pt_world - eye);

	Ray primaryRay = Ray(eye, d);

	radiance += traceRays(primaryRay);
	fragColor = radiance;
}
