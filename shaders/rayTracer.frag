#version 410 core
#define MAX_VAL  10000.f
#define MIN_VAL 0.01f
#define PI 3.1415926535897932384626433832795
#define ks 1.f
#define kd 1.f
#define kt 1.f

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
								Light( vec4(-100.f, 100.f, -100.f, 1.f), vec4(1.f, 1.f, 1.f, 1.f) ), // key light
								Light( vec4(100.f, -100.f, -100.f, 1.f), vec4(1.f, 0.9f, 0.9f, 1.f) ), // rim light
								Light( vec4(-100.f, 100.f, 100.f, 1.f), vec4(0.9f, 0.9f, 1.f, 1.f) ) // back light
								);

Material fog = Material(
							vec4(0.8353f, 0.7804f, 0.9098f, 0.1f), 
							vec4(0.8f, 0.8f, 1.f, 0.1f),
							vec4(0.8f, 0.8f, 0.9f, 0.1f),
							vec4(0.8f, 0.9f, 1.f, 0.1f),
							3.f,
							vec3(1.3f, 1.5f, 1.7f),
							0.1f
							);

Material salmon = Material(
							vec4(1.f, 0.549f,  0.412f, 0.5f), 
							vec4(1.f, 0.8f,  0.8f, 0.5f), 
							vec4(1.f, 0.6f,  0.5f, 0.5f), 
							vec4(1.f, 0.549f,  0.412f, 0.5f), 
							4.f,
							vec3(1.5f, 1.6f, 1.7f),
							0.5f
							);

Material forest = Material(
							vec4(0.133f, 0.545f, 0.133f, 0.8f),
							vec4(0.8f, 1.f, 0.8f, 0.8f),
							vec4(0.2f, 0.5f, 0.2f, 0.8f),
							vec4(0.2f, 0.6f, 0.2f, 0.8f),
							5.f,
							vec3(1.8f, 1.9f, 2.f),
							0.6f
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
Sphere centerSphere = Sphere(centerSphereTransformation, fog);

Sphere sceneSpheres[] = Sphere[](leftSphere, centerSphere, rightSphere);


/////////////////////////////////////////////////////////////////////////////////
/*
*	RAY TRACER
*/
////////////////////////////////////////////////////////////////////////////////
float clip(float x, float a, float b) {
	// x -> [a, b]
	return min(b, max(x, a));
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
	data.isIntersect = false; 
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

	for (int i = 0; i < sceneSpheres.length(); i++){ // for each Sphere
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

	for (int i = 0; i < sceneLighting.length(); i++) { // for each light in the scene
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
		float denom = 100.f*exp(dist*dist);
		float attenuation = min(1.f, 1.f/denom);


		float cosTheta = clip(dot(normalize(normal.xyz), normalize(vertexToLight.xyz)), 0.f, 1.f);	
		vec4 diffuseComponent = attenuation*I*kd*data.mat.diffuseColor*cosTheta;

		vec4 reflected = -normalize(2.f*normal*(clip(dot(normal.xyz, vertexToLight.xyz), 0.f, 1.f)) - vertexToLight);
		float cosPhi = clip(dot(reflected.xyz, ray.d.xyz), 0.f, 1.f);
		vec4 specularComponent = I*ks*data.mat.specularColor*pow(cosPhi, data.mat.shininess);

		vec4 ambientComponent = vec4(0.f);

		radiance += ambientComponent;
		radiance += diffuseComponent;
		radiance += specularComponent;

		radiance.x = clip(radiance.x, 0.f, 1.f);
		radiance.y = clip(radiance.y, 0.f, 1.f);
		radiance.z = clip(radiance.z, 0.f, 1.f);
		radiance.w = clip(radiance.w, 0.f, 1.f);
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
		
		vec4 reflectionComponent = vec4(0.f);
		
		// reflection pass
		vec4 reflected = normalize(2.f*normal*(clip(dot(normal.xyz, v.xyz), 0.f, 1.f)) - v);
		Ray reflectionRay = Ray(vertex + MIN_VAL*reflected, reflected);
		Data reflectionData = intersect(reflectionRay);

		if (reflectionData.isIntersect) {
			vec4 reflectionColor = reflectionData.mat.reflectedColor;
			reflectionComponent += max(vec4(0.f), ks*reflectionColor*computeLighting(reflectionRay, reflectionData));
		}

		vec4 indirectComponent = vec4(0.f);

		{// refraction pass
			vec3 ior = currentData.mat.ior;
			float ior_red = ior.x;
			float ior_green = ior.y;
			float ior_blue = ior.z;

			{ // red wavelength
				float r0 = pow((1.f - ior_red)/(1.f + ior_red), 2.f); // assume everything else is just air
				float F = r0 + (1.f - r0)*pow((1.f - clip(dot(normal.xyz, v.xyz), 0.f, 1.f)), 5.f);

				vec4 p = vertex;
				vec4 n_p = currentData.normal;
				p = p + MIN_VAL*(-n_p);
				vec4 hitPoint = p + currentData.t*currentRay.d;

				vec4 d_p = vec4(normalize(refract(n_p.xyz, hitPoint.xyz, ior_red)), 0.f);
				Ray throughRay = Ray(p, d_p);
				Data throughData = intersect(throughRay);

				vec4 q = p + throughData.t*d_p;
				vec4 n_q = throughData.normal;

				q = q + MIN_VAL*n_q;
				vec4 otherside = q + throughData.t*throughRay.d;

				vec4 d_q = vec4(normalize(refract(-n_q.xyz, otherside.xyz, 1.f/ior_red)), 0.f);
				Ray refractedRay = Ray(q, d_q);
				Data refractionData = intersect(refractedRay);

				if (refractionData.isIntersect) {
					vec4 retrievedLighting  = computeLighting(refractedRay, refractionData);
					vec4 transparencyColor = refractionData.mat.tranparencyColor;
					vec4 refractionComponent = max(vec4(0.f), kt*transparencyColor*retrievedLighting);

					indirectComponent.x += F*reflectionComponent.x + (1.f - F)*refractionComponent.x;
				}
			}


			{ // green wavelength
				float r0 = pow((1.f - ior_green)/(1.f + ior_green), 2.f); // assume everything else is just air
				float F = r0 + (1.f - r0)*pow((1.f - clip(dot(normal.xyz, v.xyz), 0.f, 1.f)), 5.f);

				vec4 p = vertex;
				vec4 n_p = currentData.normal;
				p = p + MIN_VAL*(-n_p);
				vec4 hitPoint = p + currentData.t*currentRay.d;

				vec4 d_p = vec4(normalize(refract(n_p.xyz, hitPoint.xyz, ior_green)), 0.f);
				Ray throughRay = Ray(p, d_p);
				Data throughData = intersect(throughRay);

				vec4 q = p + throughData.t*d_p;
				vec4 n_q = throughData.normal;

				q = q + MIN_VAL*n_q;
				vec4 otherside = q + throughData.t*throughRay.d;

				vec4 d_q = vec4(normalize(refract(-n_q.xyz, otherside.xyz, 1.f/ior_green)), 0.f);
				Ray refractedRay = Ray(q, d_q);
				Data refractionData = intersect(refractedRay);

				if (refractionData.isIntersect) {
					vec4 retrievedLighting  = computeLighting(refractedRay, refractionData);
					vec4 transparencyColor = refractionData.mat.tranparencyColor;
					vec4 refractionComponent = max(vec4(0.f), kt*transparencyColor*retrievedLighting);

					indirectComponent.y += F*reflectionComponent.y + (1.f - F)*refractionComponent.y;
				}
			}


			{ // blue wavelength
				float r0 = pow((1.f - ior_blue)/(1.f + ior_blue), 2.f); // assume everything else is just air
				float F = r0 + (1.f - r0)*pow((1.f - clip(dot(normal.xyz, v.xyz), 0.f, 1.f)), 5.f);

				vec4 p = vertex;
				vec4 n_p = currentData.normal;
				p = p + MIN_VAL*(-n_p);
				vec4 hitPoint = p + currentData.t*currentRay.d;

				vec4 d_p = vec4(normalize(refract(n_p.xyz, hitPoint.xyz, ior_blue)), 0.f);
				Ray throughRay = Ray(p, d_p);
				Data throughData = intersect(throughRay);

				vec4 q = p + throughData.t*d_p;
				vec4 n_q = throughData.normal;

				q = q + MIN_VAL*n_q;
				vec4 otherside = q + throughData.t*throughRay.d;

				vec4 d_q = vec4(normalize(refract(-n_q.xyz, otherside.xyz, 1.f/ior_blue)), 0.f);
				Ray refractedRay = Ray(q, d_q);
				Data refractionData = intersect(refractedRay);

				if (refractionData.isIntersect) {
					vec4 retrievedLighting  = computeLighting(refractedRay, refractionData);
					vec4 transparencyColor = refractionData.mat.tranparencyColor;
					vec4 refractionComponent = max(vec4(0.f), kt*transparencyColor*retrievedLighting);

					indirectComponent.z += F*reflectionComponent.z + (1.f - F)*refractionComponent.z;
				}
			}
		}
		
		radiance += indirectComponent;

		currentData = reflectionData;
		currentRay = reflectionRay;
	}

	radiance.x = clip(radiance.x, 0.f, 1.f);
	radiance.y = clip(radiance.y, 0.f, 1.f);
	radiance.z = clip(radiance.z, 0.f, 1.f);
	radiance.w = clip(radiance.w, 0.f, 1.f);

	return radiance;
};

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
*  Driver code
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////
void main() {
	vec4 radiance = vec4(0.1f, 0.1f, 0.1f, 0.1f);
	float x = position.x; //in film
	float y = position.y; //in film

	vec4 eye = M_film2World*vec4(0.f, 0.f, 0.f, 1.f);
	
	vec4 pt_film = vec4(x, y, -1.f, 1.f);

    vec4 pt_world = M_film2World*pt_film;

    vec4 d = normalize(pt_world - eye);

	Ray primaryRay = Ray(eye, d);

	radiance += traceRays(primaryRay);
	fragColor = radiance;
}
