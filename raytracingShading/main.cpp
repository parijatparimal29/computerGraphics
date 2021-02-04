////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <stack>

// Eigen for matrix operations
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"
#include "utils.h"

// JSON parser library (https://github.com/nlohmann/json)
#include "json.hpp"
using json = nlohmann::json;

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
// Define types & classes
////////////////////////////////////////////////////////////////////////////////

struct Ray {
	Vector3d origin;
	Vector3d direction;
	Ray() { }
	Ray(Vector3d o, Vector3d d) : origin(o), direction(d) { }
};

struct Light {
	Vector3d position;
	Vector3d intensity;
};

struct Intersection {
	Vector3d position;
	Vector3d normal;
	double ray_param;
};

struct Camera {
	bool is_perspective;
	Vector3d position;
	double field_of_view; // between 0 and PI
	double focal_length;
	double lens_radius; // for depth of field
};

struct Material {
	Vector3d ambient_color;
	Vector3d diffuse_color;
	Vector3d specular_color;
	double specular_exponent; // Also called "shininess"

	Vector3d reflection_color;
	Vector3d refraction_color;
	double refraction_index;
};

struct Object {
	Material material;
	virtual ~Object() = default; // Classes with virtual methods should have a virtual destructor!
	virtual bool intersect(const Ray &ray, Intersection &hit) = 0;
};

// We use smart pointers to hold objects as this is a virtual class
typedef std::shared_ptr<Object> ObjectPtr;

struct Sphere : public Object {
	Vector3d position;
	double radius;

	virtual ~Sphere() = default;
	virtual bool intersect(const Ray &ray, Intersection &hit) override;
};

struct Parallelogram : public Object {
	Vector3d origin;
	Vector3d u;
	Vector3d v;

	virtual ~Parallelogram() = default;
	virtual bool intersect(const Ray &ray, Intersection &hit) override;
};

struct AABBTree {
	struct Node {
		AlignedBox3d bbox;
		int parent; // Index of the parent node (-1 for root)
		int left; // Index of the left child (-1 for a leaf)
		int right; // Index of the right child (-1 for a leaf)
		int triangle; // Index of the node triangle (-1 for internal nodes)
	};

	std::vector<Node> nodes;
	int root;

	AABBTree() = default; // Default empty constructor
	AABBTree(const MatrixXd &V, const MatrixXi &F); // Build a BVH from an existing mesh

	// Print the bvh tree to verify intermediate implementation details.
	void printTree(Node &node){
		if(node.triangle==-1){
			std::cout<<node.parent<<" "<<node.left<<" "<<node.right<<"\n";
			printTree(nodes[node.left]);
			printTree(nodes[node.right]);
		}
		else{
			std::cout<<node.parent<<" "<<node.left<<" "<<node.right<<" Triangle: "<<node.triangle<<"\n";
		}
	}


};

struct Mesh : public Object {
	MatrixXd vertices; // n x 3 matrix (n points)
	MatrixXi facets; // m x 3 matrix (m triangles)

	AABBTree bvh;

	Mesh() = default; // Default empty constructor
	Mesh(const std::string &filename);
	virtual ~Mesh() = default;
	virtual bool intersect(const Ray &ray, Intersection &hit) override;

	// Recursive function to traverse in the tree to find the nearest hit
	virtual bool traverse(const Ray &ray, Intersection &hit, int curr, double &nearest);
};

struct Scene {
	Vector3d background_color;
	Vector3d ambient_light;

	Camera camera;
	std::vector<Material> materials;
	std::vector<Light> lights;
	std::vector<ObjectPtr> objects;
};

////////////////////////////////////////////////////////////////////////////////

// Read a triangle mesh from an off file
void load_off(const std::string &filename, MatrixXd &V, MatrixXi &F) {
	std::ifstream in(filename);
	std::string token;
	in >> token;
	int nv, nf, ne;
	in >> nv >> nf >> ne;
	V.resize(nv, 3);
	F.resize(nf, 3);
	for (int i = 0; i < nv; ++i) {
		in >> V(i, 0) >> V(i, 1) >> V(i, 2);
	}
	for (int i = 0; i < nf; ++i) {
		int s;
		in >> s >> F(i, 0) >> F(i, 1) >> F(i, 2);
		assert(s == 3);
	}
}

Mesh::Mesh(const std::string &filename) {
	// Load a mesh from a file (assuming this is a .off file), and create a bvh
	load_off(filename, vertices, facets);
	bvh = AABBTree(vertices, facets);
}

////////////////////////////////////////////////////////////////////////////////
// BVH Implementation
////////////////////////////////////////////////////////////////////////////////

// Bounding box of a triangle
AlignedBox3d bbox_triangle(const Vector3d &a, const Vector3d &b, const Vector3d &c) {
	AlignedBox3d box;
	box.extend(a);
	box.extend(b);
	box.extend(c);
	return box;
}

AABBTree::AABBTree(const MatrixXd &V, const MatrixXi &F) {
	// Compute the centroids of all the triangles in the input mesh
	
	// Adding bounding boxes to triangles and creating nodes for bvh tree
	
	MatrixXd centroids(F.rows(), V.cols());
	centroids.setZero();
	for (int i = 0; i < F.rows(); ++i) {
		for (int k = 0; k < F.cols(); ++k) {
			centroids.row(i) += V.row(F(i, k));
		}
		centroids.row(i) /= F.cols();
		Node node;
		node.bbox = bbox_triangle(V.row(F(i, 0)),V.row(F(i, 1)),V.row(F(i, 2)));
		node.left = -1;
		node.right = -1;
		node.parent = -1;
		node.triangle = i;
		nodes.push_back(node);
	}

	// TODO (Assignment 3)

	// Method (1): Top-down approach.
	// Split each set of primitives into 2 sets of roughly equal size,
	// based on sorting the centroids along one direction or another.

	

	// Method (2): Bottom-up approach.
	// Merge nodes 2 by 2, starting from the leaves of the forest, until only 1 tree is left.

	// Bottum-up approach with adjacency criteria to merge boxes to make the algorithm run in quadratic runtime instead of cubic (needed to compute least cost)
	// Overall runtime to render the dragon was just 4.4 seconds with this implementation.

	int oldsize = 0;
	int newsize = nodes.size();
	while(newsize - oldsize > 1){
		for(unsigned i=oldsize;i<newsize-1;i+=2){
			Node* a = &nodes[i];
			Node* b = &nodes[i+1];
			Node c;
			c.bbox = a->bbox.merged(b->bbox);
			c.triangle = -1;
			c.left = i;
			c.right = i+1;
			c.parent = -1;
			nodes.push_back(c);
			a->parent = nodes.size() - 1;
			b->parent = a->parent;
		}
		if((newsize - oldsize)>1 && (newsize - oldsize)%2==1){
			oldsize = newsize - 1;
			newsize = nodes.size();
		}
		else{
			oldsize = newsize;
			newsize = nodes.size();
		}
	}
	root = nodes.size() - 1;
	//printTree(nodes[root]);
}


////////////////////////////////////////////////////////////////////////////////

bool Sphere::intersect(const Ray &ray, Intersection &hit) {
	// TODO - done (Assignment 2)

	Vector3d ec = ray.origin - position;
	double a = ray.direction.dot(ray.direction);
    double b = 2 * ec.dot(ray.direction);
    double c = ec.dot(ec) - radius*radius;
    double D = (b * b) - (4 * a * c);
    if( D<0.0 ) 
		return false;
    D = sqrt(D);
	double t = std::max(((-b-D)/(2*a)),((-b-D)/(2*a)));
	hit.position = ray.origin + t*ray.direction;
	hit.normal = (hit.position - position).normalized();
	hit.ray_param = hit.position(2);
	return true;
	
}

bool Parallelogram::intersect(const Ray &ray, Intersection &hit) {
	// TODO - done (Assignment 2)
	
	Matrix3d UVt;
	Vector3d AE;
	Vector3d ray_direction = ray.direction;

	UVt << u(0),  v(0), -ray_direction(0), 
			u(1),  v(1), -ray_direction(1), 
			u(2),  v(2), -ray_direction(2);
	AE << (ray.origin - origin);
	Vector3d x = UVt.colPivHouseholderQr().solve(AE);
	if (x(0)>=0 && x(1)>=0 && x(2)>0 && x(0)<=1 && x(1)<=1){
		Vector3d ray_intersection(ray.origin + x(2) * ray_direction);
		Vector3d ray_normal = (u.cross(v));
		hit.position = ray_intersection;
		hit.normal = ray_normal.normalized();
		hit.ray_param = x(2);
		return true;
	}
	return false;
}

// -----------------------------------------------------------------------------

bool intersect_triangle(const Ray &ray, const Vector3d &a, const Vector3d &b, const Vector3d &c, Intersection &hit) {
	// TODO (Assignment 3)
	//
	// Compute whether the ray intersects the given triangle.
	// If you have done the parallelogram case, this should be very similar to it.
	
	//Ray triangle intersection by solving ray and triangle equations

	Vector3d u = b - a;
	Vector3d v = c - a;
	Matrix3d UVt;
	Vector3d AE;
	Vector3d ray_direction = ray.direction;

	UVt << u(0),  v(0), -ray_direction(0), 
			u(1),  v(1), -ray_direction(1), 
			u(2),  v(2), -ray_direction(2);
	AE << (ray.origin - a);
	Vector3d x = UVt.colPivHouseholderQr().solve(AE);
	if (x(0)>=0.0 && x(1)>=0.0 && x(2)>0.0 && (x(0) + x(1)<=1.0)){
		hit.position = Vector3d(x(0),x(1),x(2));//ray.origin + x(2) * ray.direction;
		hit.normal = (u.cross(v)).normalized();
		hit.ray_param = x(2);
		return true;
	}
	return false;

	// Another implementation of intersection based from shadertoy algorithm for intersection
/*
	Vector3d ba = b - a;
    Vector3d ca = c - a;
    Vector3d ra = ray.origin - a;
    Vector3d n = ba.cross(ca);
    Vector3d q = ra.cross(ray.direction);
    double d = 1.0/(ray.direction.dot(n));
    double u = d*((-q).dot(ca));
    double v = d*((q).dot(ba));
    double t = d*((-n).dot(ra));
    if( u<0.0 || u>1.0 || v<0.0 || (u+v)>1.0 ){
		return false;
	} 
    hit.position = Vector3d(u,v,t);
	hit.normal = (ba.cross(ca)).normalized();
	hit.ray_param = t;
	return true;
	*/
}

bool intersect_box(const Ray &ray, const AlignedBox3d &box) {
	// TODO (Assignment 3)
	//
	// Compute whether the ray intersects the given box.
	// There is no need to set the resulting normal and ray parameter, since
	// we are not testing with the real surface here anyway.

	// Ray box intersection algorithm based on coursebook chapter 12 (extended to 3d)

	double x = 1.0 / ray.direction[0];
	double y = 1.0 / ray.direction[1];
	double z = 1.0 / ray.direction[2];
	
	double xmin = (box.min())[0];
	double ymin = (box.min())[1];
	double zmin = (box.min())[2];
	double xmax = (box.max())[0];
	double ymax = (box.max())[1];
	double zmax = (box.max())[2];

	double txmin = (xmin - ray.origin[0])*x;
	double txmax = (xmax - ray.origin[0])*x;
	double tymin = (ymin - ray.origin[1])*y;
	double tymax = (ymax - ray.origin[1])*y;
	double tzmin = (zmin - ray.origin[2])*z;
	double tzmax = (zmax - ray.origin[2])*z;

	double tmin = std::max(std::max(std::min(txmin, txmax), std::min(tymin, tymax)), std::min(tzmin, tzmax));
	double tmax = std::min(std::min(std::max(txmin, txmax), std::max(tymin, tymax)), std::max(tzmin, tzmax));

	if (tmax < 0){
    	return false;
	}

	if (tmin > tmax){
    	return false;
	}

	return true;
}

bool Mesh::intersect(const Ray &ray, Intersection &closest_hit) {
	// TODO (Assignment 3)

	// Method (1): Traverse every triangle and return the closest hit.
	bool isHit = false;
	double nearest = MAXFLOAT;

	// Bruteforce implementation of finding nearest intersection by testing intersections with all triangles
	/*
	for (int i = 0;i < facets.size()/3;i++){
		Intersection hit = Intersection();
		
		Vector3d a (vertices(facets(i,0),0),vertices(facets(i,0),1),vertices(facets(i,0),2));
		Vector3d b (vertices(facets(i,1),0),vertices(facets(i,1),1),vertices(facets(i,1),2));
		Vector3d c (vertices(facets(i,2),0),vertices(facets(i,2),1),vertices(facets(i,2),2));

		if(intersect_triangle(ray, a, b, c, hit)){
			if(hit.ray_param < nearest){
				nearest = hit.ray_param;
				closest_hit = hit;
				isHit = true;
			}
		}
	}
	*/
	
	// Method (2): Traverse the BVH tree and test the intersection with a
	// triangles at the leaf nodes that intersects the input ray.
	
	int curr = bvh.root;
	// Faster optimization by traversing across AABB tree and only checking intersection of triangle when a box is hit
	isHit = traverse(ray, closest_hit, curr, nearest);

	return isHit;
}

//Function to traverse in bvh tree to find the nearest hit and return closest hit.
bool Mesh::traverse(const Ray &ray, Intersection &closest_hit, int curr, double &nearest) {
	bool isHit = false;

	// Only need to check for an intersection when box intersects with ray and the node is a leaf node
	if(intersect_box(ray, bvh.nodes[curr].bbox)){
		if(bvh.nodes[curr].triangle!=-1){
			int i = bvh.nodes[curr].triangle;
			Intersection hit = Intersection();
	
			Vector3d a (vertices(facets(i,0),0),vertices(facets(i,0),1),vertices(facets(i,0),2));
			Vector3d b (vertices(facets(i,1),0),vertices(facets(i,1),1),vertices(facets(i,1),2));
			Vector3d c (vertices(facets(i,2),0),vertices(facets(i,2),1),vertices(facets(i,2),2));

			// If ray intersects triangle, store the nearest hit in closest_hit.
			if(intersect_triangle(ray, a, b, c, hit)){
				if(hit.ray_param < nearest){
					nearest = hit.ray_param;
					closest_hit = hit;
					isHit = true;
				}
			}
		}
		else{
			//If box hit but node is not leaf, traverse left and right subtrees
			int currLeft = bvh.nodes[curr].left;
			int currRight = bvh.nodes[curr].right;
			bool left = traverse(ray, closest_hit, currLeft, nearest);
			bool right = traverse(ray, closest_hit, currRight, nearest);
			
			//If any triangle is hit (left subtree or right subtree), isHit is set to true
			if(left||right)
				isHit = true;
		}
	}
	return isHit;
}

////////////////////////////////////////////////////////////////////////////////
// Define ray-tracing functions
////////////////////////////////////////////////////////////////////////////////

// Function declaration here (could be put in a header file)
Vector3d ray_color(const Scene &scene, const Ray &ray, const Object &object, const Intersection &hit, int max_bounce);
Object * find_nearest_object(const Scene &scene, const Ray &ray, Intersection &closest_hit);
bool is_light_visible(const Scene &scene, const Ray &ray, const Light &light);
Vector3d shoot_ray(const Scene &scene, const Ray &ray, int max_bounce);

// -----------------------------------------------------------------------------

Vector3d ray_color(const Scene &scene, const Ray &ray, const Object &obj, const Intersection &hit, int max_bounce) {
	// Material for hit object
	const Material &mat = obj.material;

	// Ambient light contribution
	Vector3d ambient_color = obj.material.ambient_color.array() * scene.ambient_light.array();

	// Punctual lights contribution (direct lighting)
	Vector3d lights_color(0, 0, 0);
	for (const Light &light : scene.lights) {
		Vector3d Li = (light.position - hit.position).normalized();
		Vector3d N = hit.normal;

		// TODO - not required (Assignment 2, shadow rays)
		/*
		Ray shadow_ray;
		//shadow_ray.origin = hit.position;
		shadow_ray.direction = (light.position - hit.position);
		shadow_ray.origin = hit.position + 0.0001 * shadow_ray.direction;
		bool flag = false;
		for (const ObjectPtr &sobj : scene.objects){
			Intersection h = Intersection();
			if(sobj->intersect(shadow_ray, h)){
				flag = true;
				break;
			}
		}
		if(flag){
			break;
		}
		*/
		
		// Diffuse contribution
		Vector3d diffuse = mat.diffuse_color * std::max(Li.dot(N), 0.0);

		// TODO - done (Assignment 2, specular contribution)
		//Vector3d specular(0, 0, 0);
		Vector3d h = (ray.origin - hit.position).normalized() + (light.position-hit.position).normalized(); //normalize each
		
		Vector3d specular = mat.specular_color * pow(h.normalized().transpose() * N, mat.specular_exponent);
		specular(0) = std::max(specular(0),0.0);
		specular(1) = std::max(specular(1),0.0);
		specular(2) = std::max(specular(2),0.0);
		// Attenuate lights according to the squared distance to the lights
		Vector3d D = light.position - hit.position;
		lights_color += (diffuse + specular).cwiseProduct(light.intensity) /  D.squaredNorm();
	}
	
	// TODO - not required (Assignment 2, reflected ray)
	Vector3d reflection_color(0, 0, 0);

	// TODO - not required (Assignment 2, refracted ray)
	Vector3d refraction_color(0, 0, 0);

	// Rendering equation
	Vector3d C = ambient_color + lights_color + reflection_color + refraction_color;

	return C;
}

// -----------------------------------------------------------------------------

Object * find_nearest_object(const Scene &scene, const Ray &ray, Intersection &closest_hit) {
	int closest_index = -1;
	// TODO (Assignment 2, find nearest hit)
	double nearest = MAXFLOAT;
	for (int i = 0;i < scene.objects.size();i++){
		Intersection hit = Intersection();
		
		if(scene.objects[i]->intersect(ray, hit)){
			if(hit.ray_param < nearest){
				closest_index = i;
				nearest = hit.ray_param;
				closest_hit = hit;
			}
		}
	}

	if (closest_index < 0) {
		// Return a NULL pointer
		return nullptr;
	} else {
		// Return a pointer to the hit object. Don't forget to set 'closest_hit' accordingly!
		return scene.objects[closest_index].get();
	}
}

bool is_light_visible(const Scene &scene, const Ray &ray, const Light &light) {
	// TODO (Assignment 2, shadow ray)
	return true;
}

Vector3d shoot_ray(const Scene &scene, const Ray &ray, int max_bounce) {
	Intersection hit;
	if (Object * obj = find_nearest_object(scene, ray, hit)) {
		// 'obj' is not null and points to the object of the scene hit by the ray
		return ray_color(scene, ray, *obj, hit, max_bounce);
	} else {
		// 'obj' is null, we must return the background color
		return scene.background_color;
	}
}

////////////////////////////////////////////////////////////////////////////////

void render_scene(const Scene &scene) {
	std::cout << "Simple ray tracer." << std::endl;

	int w = 640;
	int h = 480;
	MatrixXd R = MatrixXd::Zero(w, h);
	MatrixXd G = MatrixXd::Zero(w, h);
	MatrixXd B = MatrixXd::Zero(w, h);
	MatrixXd A = MatrixXd::Zero(w, h); // Store the alpha mask

	// The camera always points in the direction -z
	// The sensor grid is at a distance 'focal_length' from the camera center,
	// and covers an viewing angle given by 'field_of_view'.
	double aspect_ratio = double(w) / double(h);
	double scale_y = scene.camera.focal_length * tan(scene.camera.field_of_view / 2);
	// TODO: Stretch the pixel grid by the proper amount here
	double scale_x = aspect_ratio*scale_y; //

	// The pixel grid through which we shoot rays is at a distance 'focal_length'
	// from the sensor, and is scaled from the canonical [-1,1] in order
	// to produce the target field of view.
	Vector3d grid_origin(-scale_x, scale_y, -scene.camera.focal_length);
	Vector3d x_displacement(2.0/w*scale_x, 0, 0);
	Vector3d y_displacement(0, -2.0/h*scale_y, 0);

	for (unsigned i = 0; i < w; ++i) {
		std::cout << std::fixed << std::setprecision(2);
		std::cout << "Ray tracing: " << (100.0 * i) / w << "%\r" << std::flush;
		for (unsigned j = 0; j < h; ++j) {
			// TODO - not required (Assignment 2, depth of field)
			Vector3d shift = grid_origin + (i+0.5)*x_displacement + (j+0.5)*y_displacement;

			// Prepare the ray
			Ray ray;

			if (scene.camera.is_perspective) {
				// Perspective camera
				// TODO - done (Assignment 2, perspective camera)
				ray.origin = scene.camera.position;
				ray.direction = shift;
			} else {
				// Orthographic camera
				ray.origin = scene.camera.position + Vector3d(shift[0], shift[1], 0);
				ray.direction = Vector3d(0, 0, -1);
			}

			int max_bounce = 5;
			Vector3d C = shoot_ray(scene, ray, max_bounce);
			R(i, j) = C(0);
			G(i, j) = C(1);
			B(i, j) = C(2);
			A(i, j) = 1;
		}
	}

	std::cout << "Ray tracing: 100%  " << std::endl;

	// Save to png
	const std::string filename("raytrace.png");
	write_matrix_to_png(R, G, B, A, filename);
}

////////////////////////////////////////////////////////////////////////////////

Scene load_scene(const std::string &filename) {
	Scene scene;

	// Load json data from scene file
	json data;
	std::ifstream in(filename);
	in >> data;

	// Helper function to read a Vector3d from a json array
	auto read_vec3 = [] (const json &x) {
		return Vector3d(x[0], x[1], x[2]);
	};

	// Read scene info
	scene.background_color = read_vec3(data["Scene"]["Background"]);
	scene.ambient_light = read_vec3(data["Scene"]["Ambient"]);

	// Read camera info
	scene.camera.is_perspective = data["Camera"]["IsPerspective"];
	scene.camera.position = read_vec3(data["Camera"]["Position"]);
	scene.camera.field_of_view = data["Camera"]["FieldOfView"];
	scene.camera.focal_length = data["Camera"]["FocalLength"];
	scene.camera.lens_radius = data["Camera"]["LensRadius"];

	// Read materials
	for (const auto &entry : data["Materials"]) {
		Material mat;
		mat.ambient_color = read_vec3(entry["Ambient"]);
		mat.diffuse_color = read_vec3(entry["Diffuse"]);
		mat.specular_color = read_vec3(entry["Specular"]);
		mat.reflection_color = read_vec3(entry["Mirror"]);
		mat.refraction_color = read_vec3(entry["Refraction"]);
		mat.refraction_index = entry["RefractionIndex"];
		mat.specular_exponent = entry["Shininess"];
		scene.materials.push_back(mat);
	}

	// Read lights
	for (const auto &entry : data["Lights"]) {
		Light light;
		light.position = read_vec3(entry["Position"]);
		light.intensity = read_vec3(entry["Color"]);
		scene.lights.push_back(light);
	}

	// Read objects
	for (const auto &entry : data["Objects"]) {
		ObjectPtr object;
		if (entry["Type"] == "Sphere") {
			auto sphere = std::make_shared<Sphere>();
			sphere->position = read_vec3(entry["Position"]);
			sphere->radius = entry["Radius"];
			object = sphere;
		} else if (entry["Type"] == "Parallelogram") {
			// TODO - done
			auto parallelogram = std::make_shared<Parallelogram>();
			parallelogram->origin = read_vec3(entry["Origin"]);
			parallelogram->u = read_vec3(entry["U"]);
			parallelogram->v = read_vec3(entry["V"]);
			object = parallelogram;
		} else if (entry["Type"] == "Mesh") {
			// Load mesh from a file
			std::string filename = std::string(DATA_DIR) + entry["Path"].get<std::string>();
			object = std::make_shared<Mesh>(filename);
		}
		object->material = scene.materials[entry["Material"]];
		scene.objects.push_back(object);
	}

	return scene;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {
	if (argc < 2) {
		std::cerr << "Usage: " << argv[0] << " scene.json" << std::endl;
		return 1;
	}
	Scene scene = load_scene(argv[1]);
	render_scene(scene);
	return 0;
}
