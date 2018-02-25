#ifndef TEST_MODEL_CORNEL_BOX_H
#define TEST_MODEL_CORNEL_BOX_H

// Defines a simple test model: The Cornel Box

#include <glm/glm.hpp>
#include <vector>
#include <fstream>
#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>      // std::istringstream

#define DIF 0.6
#define SPC 0.3
#define AMB 0.1

enum lightType_t { Pointlight, Spotlight, Directional};

class Phong
{
public:
	float diffuse;
	float specular;
	float ambient;

	Phong(float diffuse, float specular, float ambient)
	: diffuse(diffuse), specular(specular), ambient(ambient) {}

};

class Reflective
{
public:
	float ioreflection;
};

class Refractive
{
public:
	float ior;
};


class Light
{
public:
	glm::vec4 pos;
	glm::vec3 color;
	float diffuse_power;
	float specular_power;
	float ambient_power;
	lightType_t lightType;

	Light(glm::vec4 pos,glm::vec3 color, float diffuse_power, float specular_power, float ambient_power,lightType_t lightType)
	: pos(pos), color(color),diffuse_power(diffuse_power), specular_power(specular_power),ambient_power(ambient_power), lightType(lightType)
	{
		std::cout << "New Light Created" << '\n';
	}
};



// Used to describe a triangular surface:
class Triangle
{
public:
	glm::vec4 v0;
	glm::vec4 v1;
	glm::vec4 v2;
	glm::vec4 normal;
	glm::vec3 color;
	Phong phong;
	// Reflective reflective;
	// Refractive refractive;

	Triangle( glm::vec4 v0, glm::vec4 v1, glm::vec4 v2, glm::vec3 color,
		float diffuse, float specular, float ambient )
		: v0(v0), v1(v1), v2(v2), color(color), phong(diffuse, specular, ambient)
	{
		ComputeNormal();
	}

	void ComputeNormal()
	{
	  glm::vec3 e1 = glm::vec3(v1.x-v0.x,v1.y-v0.y,v1.z-v0.z);
	  glm::vec3 e2 = glm::vec3(v2.x-v0.x,v2.y-v0.y,v2.z-v0.z);
	  glm::vec3 normal3 = glm::normalize( glm::cross( e2, e1 ) );
	  normal.x = normal3.x;
	  normal.y = normal3.y;
	  normal.z = normal3.z;
	  normal.w = 1.0;
	}
};


bool loadObj(std::string path, std::vector<Triangle>& triangles, glm::vec3 color ){
	glm::vec3 white(  0.75f, 0.75f, 0.75f );
	std::vector<glm::vec4> vertices;
	std::vector<glm::vec4> faces;

	std::ifstream file(path); // pass file name as argment
	if( !file ){
	    std::cout << "Can't open file !" << "\n" ;
	    return false;
	}

	std::string line;
	while(getline(file, line)) {
		if (line[0] == '#' ){
			//comment so do nothing
		}
		else if (line.substr(0,2) == "v "){	//checking for v by substring as there is also vt vn
			std::istringstream linestream(line.substr(2)); //istringstream input stream
			glm::vec4 vector;
			linestream >> vector.x;
			linestream >> vector.y;
			linestream >> vector.z;
			vector.w = 1;
			vector.y *=-1;
			vector.z *=-1;
			vertices.push_back(vector);
		}
		// else if (line.substr(0,2) == "vt"){
    //
		// }
		// else if (line.substr(0,2) == "vn"){
    //
		// }

		else if (line[0] == 'f'){
			glm::vec4 face;
		  const char* line_ptr=line.c_str();
			int x,y,z,temp1,temp2,temp3;
	    sscanf (line_ptr, "f %d//%d %d//%d %d//%d",&x,&temp1,&y,&temp2,&z,&temp3); //here it read the line start with f and store the corresponding values in the variables
			face.x = x;
			face.y = y;
			face.z = z;
			face.w = 1;
			//obj starts from 1 index not zero
			face.x--;
			face.y--;
			face.z--;
			faces.push_back(face);
		}

		// std::cout << line << "\n";
	}

	for (unsigned int i = 0 ; i < faces.size(); i++){
		triangles.push_back(Triangle(vertices[faces[i].x], vertices[faces[i].y], vertices[faces[i].z], color, DIF,SPC,AMB) );
	}
	return true;
}


// Loads the Cornell Box. It is scaled to fill the volume:
// -1 <= x <= +1
// -1 <= y <= +1
// -1 <= z <= +1


void LoadTestModel( std::vector<Triangle>& triangles )
{
	using glm::vec3;
	using glm::vec4;

	// Defines colors:
	vec3 red(    0.75f, 0.15f, 0.15f );
	vec3 yellow( 0.75f, 0.75f, 0.15f );
	vec3 green(  0.15f, 0.75f, 0.15f );
	vec3 cyan(   0.15f, 0.75f, 0.75f );
	vec3 blue(   0.15f, 0.15f, 0.75f );
	vec3 purple( 0.75f, 0.15f, 0.75f );
	vec3 white(  0.75f, 0.75f, 0.75f );

	triangles.clear();
	triangles.reserve( 5*2*3 );

	// ---------------------------------------------------------------------------
	// Room

	float L = 555;			// Length of Cornell Box side.

	vec4 A(L,0,0,1);
	vec4 B(0,0,0,1);
	vec4 C(L,0,L,1);
	vec4 D(0,0,L,1);

	vec4 E(L,L,0,1);
	vec4 F(0,L,0,1);
	vec4 G(L,L,L,1);
	vec4 H(0,L,L,1);

	// Floor:
	triangles.push_back( Triangle( C, B, A, white,DIF,SPC,AMB) );
	triangles.push_back( Triangle( C, D, B, white,DIF,SPC,AMB ) );

	// Left wall
	triangles.push_back( Triangle( A, E, C, red,DIF ,SPC,AMB) );
	triangles.push_back( Triangle( C, E, G, red,DIF ,SPC,AMB) );

	// Right wall
	triangles.push_back( Triangle( F, B, D, green,DIF ,SPC,AMB) );
	triangles.push_back( Triangle( H, F, D, green,DIF ,SPC,AMB) );

	// Ceiling
	triangles.push_back( Triangle( E, F, G, white,DIF,SPC,AMB ) );
	triangles.push_back( Triangle( F, H, G, white,DIF ,SPC,AMB) );

	// Back wall
	triangles.push_back( Triangle( G, D, C, white,DIF,SPC ,AMB) );
	triangles.push_back( Triangle( G, H, D, white,DIF,SPC ,AMB) );

	// ---------------------------------------------------------------------------
	// Short block

	A = vec4(290,0,114,1);
	B = vec4(130,0, 65,1);
	C = vec4(240,0,272,1);
	D = vec4( 82,0,225,1);

	E = vec4(290,165,114,1);
	F = vec4(130,165, 65,1);
	G = vec4(240,165,272,1);
	H = vec4( 82,165,225,1);

	// Front
	triangles.push_back( Triangle(E,B,A,yellow, DIF,SPC,AMB) );
	triangles.push_back( Triangle(E,F,B,yellow,DIF,SPC ,AMB) );

	// Front
	triangles.push_back( Triangle(F,D,B,yellow, DIF,SPC,AMB) );
	triangles.push_back( Triangle(F,H,D,yellow,DIF,SPC,AMB) );

	// BACK
	triangles.push_back( Triangle(H,C,D,yellow,DIF,SPC,AMB) );
	triangles.push_back( Triangle(H,G,C,yellow,DIF,SPC,AMB) );

	// LEFT
	triangles.push_back( Triangle(G,E,C,yellow,DIF,SPC,AMB) );
	triangles.push_back( Triangle(E,A,C,yellow,DIF,SPC,AMB) );

	// TOP
	triangles.push_back( Triangle(G,F,E,yellow,DIF,SPC,AMB) );
	triangles.push_back( Triangle(G,H,F,yellow,DIF,SPC,AMB) );

	// ---------------------------------------------------------------------------
	// Tall block

	A = vec4(423,0,247,1);
	B = vec4(265,0,296,1);
	C = vec4(472,0,406,1);
	D = vec4(314,0,456,1);

	E = vec4(423,330,247,1);
	F = vec4(265,330,296,1);
	G = vec4(472,330,406,1);
	H = vec4(314,330,456,1);

	// Front
	triangles.push_back( Triangle(E,B,A,blue,DIF,SPC,AMB) );
	triangles.push_back( Triangle(E,F,B,blue,DIF,SPC,AMB) );

	// Front
	triangles.push_back( Triangle(F,D,B,blue,DIF,SPC,AMB) );
	triangles.push_back( Triangle(F,H,D,blue,DIF,SPC,AMB) );

	// BACK
	triangles.push_back( Triangle(H,C,D,blue,DIF,SPC,AMB) );
	triangles.push_back( Triangle(H,G,C,blue,DIF,SPC,AMB) );

	// LEFT
	triangles.push_back( Triangle(G,E,C,blue,DIF,SPC,AMB) );
	triangles.push_back( Triangle(E,A,C,blue,DIF,SPC,AMB) );

	// TOP
	triangles.push_back( Triangle(G,F,E,blue,DIF,SPC,AMB) );
	triangles.push_back( Triangle(G,H,F,blue,DIF,SPC,AMB) );


	// ----------------------------------------------
	// Scale to the volume [-1,1]^3

	for( size_t i=0; i<triangles.size(); ++i )
	{
		triangles[i].v0 *= 2/L;
		triangles[i].v1 *= 2/L;
		triangles[i].v2 *= 2/L;

		triangles[i].v0 -= vec4(1,1,1,1);
		triangles[i].v1 -= vec4(1,1,1,1);
		triangles[i].v2 -= vec4(1,1,1,1);

		triangles[i].v0.x *= -1;
		triangles[i].v1.x *= -1;
		triangles[i].v2.x *= -1;

		triangles[i].v0.y *= -1;
		triangles[i].v1.y *= -1;
		triangles[i].v2.y *= -1;

		triangles[i].v0.w = 1.0;
		triangles[i].v1.w = 1.0;
		triangles[i].v2.w = 1.0;

		triangles[i].ComputeNormal();
	}
}

#endif
