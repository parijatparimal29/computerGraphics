// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"


// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

void raytrace_sphere() {
	std::cout << "Simple ray tracer, one sphere with orthographic projection" << std::endl;

	const std::string filename("sphere_orthographic.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// Single light source
	const Vector3d light_position(-1,1,1);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_direction = RowVector3d(0,0,-1);

			// Intersect with the sphere
			// NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis
			Vector2d ray_on_xy(ray_origin(0),ray_origin(1));
			const double sphere_radius = 0.9;
			
			if (ray_on_xy.norm() < sphere_radius) {
				// The ray hit the sphere, compute the exact intersection point
				Vector3d ray_intersection(ray_on_xy(0),ray_on_xy(1),sqrt(sphere_radius*sphere_radius - ray_on_xy.squaredNorm()));

				// Compute normal at the intersection point
				Vector3d ray_normal = ray_intersection.normalized();

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);

}

void raytrace_parallelogram() {
	std::cout << "Simple ray tracer, one parallelogram with orthographic projection" << std::endl;

	const std::string filename("plane_orthographic.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
	Vector3d pgram_origin(-0.1,-0.1,0.2);
	Vector3d pgram_u(0.5,0,0.2);
	Vector3d pgram_v(-0.2,0.4,0.2);

	// Single light source
	const Vector3d light_position(-1,1,1);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_direction = RowVector3d(0,0,-1);

			// TODO: Check if the ray intersects with the parallelogram
			Matrix3d UVt;
			Vector3d AE;
			UVt << -pgram_u(0),  -pgram_v(0), ray_direction(0), 
					-pgram_u(1),  -pgram_v(1), ray_direction(1), 
					-pgram_u(2),  -pgram_v(2), ray_direction(2);
			AE << (pgram_origin - ray_origin);
			Vector3d x = UVt.colPivHouseholderQr().solve(AE);
			
			if (x(0)>=0 && x(1)>=0 && x(2)>0 && x(0)<=1 && x(1)<=1) {
				// TODO: The ray hit the parallelogram, compute the exact intersection point
				Vector3d ray_intersection(ray_origin + x(2) * ray_direction);
				
				// TODO: Compute normal at the intersection point
				Vector3d ray_normal = ray_intersection.normalized();

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);
}

void raytrace_perspective() {
	std::cout << "Simple ray tracer, one parallelogram with perspective projection" << std::endl;

	const std::string filename("plane_perspective.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
	Vector3d pgram_origin(-0.3,0,0);
	Vector3d pgram_u(0.5,0,0);
	Vector3d pgram_v(0.2,0.6,0);

	// Single light source
	const Vector3d light_position(-1,1,1);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// TODO: Prepare the ray (origin point and direction)
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_direction = RowVector3d(0,0,-1);
			//In perspective view direction = p - e
			Vector3d perspective_dir = ray_origin - origin;

			// TODO: Check if the ray intersects with the parallelogram
			Matrix3d UVt;
			Vector3d AE;
			UVt << -pgram_u(0),  -pgram_v(0), perspective_dir(0), 
					-pgram_u(1),  -pgram_v(1), perspective_dir(1), 
					-pgram_u(2),  -pgram_v(2), perspective_dir(2);
			AE << (pgram_origin - ray_origin);
			Vector3d x = UVt.colPivHouseholderQr().solve(AE);

			if (x(0)>=0 && x(1)>=0 && x(2)>0 && x(0)<=1 && x(1)<=1) {
				// TODO: The ray hit the parallelogram, compute the exact intersection point
				Vector3d ray_intersection(ray_origin + x(2) * perspective_dir);

				// TODO: Compute normal at the intersection point
				Vector3d ray_normal = ray_intersection.normalized();

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);
}

void raytrace_shading(){
	std::cout << "Simple ray tracer, one sphere with different shading" << std::endl;

	const std::string filename("shading.png");
	// Store the color
	//Using RGB instead of single matrix for C to have more flexibility while choosing colors in different shadings
	MatrixXd R = MatrixXd::Zero(800,800);
	MatrixXd G = MatrixXd::Zero(800,800);
	MatrixXd B = MatrixXd::Zero(800,800);
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/A.cols(),0,0);
	Vector3d y_displacement(0,-2.0/A.rows(),0);

	// Single light source
	const Vector3d light_position(-1,1,1);
	double ambient = 0.1;
	MatrixXd diffuse = MatrixXd::Zero(800, 800);
	MatrixXd specular = MatrixXd::Zero(800, 800);

	for (unsigned i=0; i < A.cols(); ++i) {
		for (unsigned j=0; j < A.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_direction = RowVector3d(0,0,-1);

			// Intersect with the sphere
			// NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis
			Vector2d ray_on_xy(ray_origin(0),ray_origin(1));
			const double sphere_radius = 0.9;

			if (ray_on_xy.norm() < sphere_radius) {
				// The ray hit the sphere, compute the exact intersection point
				Vector3d ray_intersection(ray_on_xy(0),ray_on_xy(1),sqrt(sphere_radius*sphere_radius - ray_on_xy.squaredNorm()));

				// Compute normal at the intersection point
				Vector3d ray_normal = ray_intersection.normalized();

				// TODO: Add shading parameter here
				//Vector h ( bisector of v and l) for specular shading
				Vector3d h = (origin - ray_intersection) + (light_position-ray_intersection);
				
				//Experimenting wirh k_d = 5, k)s = 10 and p = 16
				diffuse(i,j) = 5*(light_position-ray_intersection).normalized().transpose() * ray_normal;
				specular(i,j) = 10*pow(h.normalized().transpose() * ray_normal,16);
				diffuse(i,j) = std::max(diffuse(i,j),0.);
				specular(i,j) = std::max(specular(i,j),0.);
				// Simple diffuse model
				// Storing colors as RGB
				//Adding different intensities for different colors and shades
				R(i,j) = 0.1*ambient + 0.1*diffuse(i,j) + 0.3*specular(i,j);
				G(i,j) = 0.5*ambient + 0.1*diffuse(i,j) + 0.1*specular(i,j);
				B(i,j) = 0.1*ambient + 0.3*diffuse(i,j) + 0.1*specular(i,j);

				// Clamp to zero
				//Clamped diffuse and specular values before adding to RGB matrices (as per the formulae in lecture slides)

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(R,G,B,A,filename);
}

int main() {
	raytrace_sphere();
	raytrace_parallelogram();
	raytrace_perspective();
	raytrace_shading();

	return 0;
}
