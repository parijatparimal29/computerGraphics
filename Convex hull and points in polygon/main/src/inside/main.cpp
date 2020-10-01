////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>
////////////////////////////////////////////////////////////////////////////////

typedef std::complex<double> Point;
typedef std::vector<Point> Polygon;

double inline det(const Point &u, const Point &v) {
	// TODO
	double determinant = (u.real()*v.imag()) - (u.imag()*v.real());
	return determinant;
}

// Return true iff [a,b] intersects [c,d], and store the intersection in ans
bool intersect_segment(const Point &a, const Point &b, const Point &c, const Point &d, Point &ans) {
	// TODO
	double a1 = b.imag() - a.imag();
	double b1 = a.real() - b.real();
	double c1 = a1*a.real() + b1*a.imag();

	double a2 = d.imag() - c.imag();
	double b2 = c.real() - d.real();
	double c2 = a2*c.real() + b2*c.imag();

	double determinant = det(Point(a1,b1),Point(a2,b2));
	if(determinant==0){
		return false;
	}
	else{
		double x = det(Point(c1,c2),Point(b1,b2))/determinant;
		double y = det(Point(a1,a2),Point(c1,c2))/determinant;
		ans = Point(x,y);
		return true;
	}
	return false;
}

////////////////////////////////////////////////////////////////////////////////

bool is_inside(const Polygon &poly, const Point &query) {
	// 1. Compute bounding box and set coordinate of a point outside the polygon
	// TODO
	double maxX = 1;
	double maxY = 1;
	for(int i=0;i<poly.size();i++){
		maxX = std::max(poly[i].real(),maxX);
		maxY = std::max(poly[i].imag(),maxY);
	}
	Point outside(maxX+1, maxY+1);
	//std::cout<<outside.real()<<" "<<outside.imag()<<"\n";
	// 2. Cast a ray from the query point to the 'outside' point, count number of intersections
	// TODO

	int count = 0;	
	for(int i=1;i<=poly.size();i++){
		Point a;
		Point b;
		if(i==poly.size()){
			a = poly[0];
			b = poly[i-1];
		}
		else{
			a = poly[i-1];
			b = poly[i];
		}
		//std::cout<<a.real()<<" "<<a.imag()<<" "<<b.real()<<" "<<b.imag()<<"\n";
		Point intersection;
		if(intersect_segment(a,b,query,outside,intersection)){
			double minx = std::min(a.real(),b.real());
			double miny = std::min(a.imag(),b.imag());
			double maxx = std::max(a.real(),b.real());
			double maxy = std::max(a.imag(),b.imag());
			double minx2 = std::min(query.real(),outside.real());
			double miny2 = std::min(query.imag(),outside.imag());
			double maxx2 = std::max(query.real(),outside.real());
			double maxy2 = std::max(query.imag(),outside.imag());
			double x = intersection.real();
			double y = intersection.imag();
			if(minx<x && maxx>x && miny<y && maxy>y &&
				minx2<x && maxx2>x && miny2<y && maxy2>y){
				count++;

			}
		}
	}
	
	if(count%2==0){
		return false;
	}
	else{
		//std::cout<<query.real()<<" "<<query.imag()<<" "<<count<<"\n";
		return true;
	}
	
}

////////////////////////////////////////////////////////////////////////////////

std::vector<Point> load_xyz(const std::string &filename) {
	std::vector<Point> points;
	std::ifstream in(filename);
	// TODO
	std::string temp;
	getline(in,temp);
	int n = std::stoi(temp);
	while(getline(in,temp)){
		int pos1 = temp.find(" ");
		int pos2 = (temp.substr(pos1+1)).find(" ");
		double x = std::stod(temp.substr(0,pos1));
		double y = std::stod(temp.substr(pos1+1,pos2));
		points.push_back(Point(x,y));
	}
	return points;
}

Polygon load_obj(const std::string &filename) {
	std::ifstream in(filename);
	// TODO
	Polygon obj;
	std::string temp;
	while(getline(in,temp)){
		if(temp[0]!='v'){
			break;
		}
		int pos1 = temp.find(" ");
		int pos2 = (temp.substr(pos1+1)).find(" ");
		int pos3 = (temp.substr(pos2+1)).find(" ");
		double x = std::stod(temp.substr(pos1+1,pos2));
		double y = std::stod(temp.substr(pos1+pos2+1,pos1+pos2+pos3));
		obj.push_back(Point(x,y));
	}
	/*
	for(int i=0;i<obj.size();i++){
		std::cout<<obj[i].real()<<" "<<obj[i].imag()<<std::endl;
	}
	*/
	return obj;
}

void save_xyz(const std::string &filename, const std::vector<Point> &points) {
	// TODO
	std::ofstream out(filename);
	if (!out.is_open()) {
		throw std::runtime_error("failed to open file " + filename);
	}
	out << std::fixed;
	out << points.size()<<"\n";
	for (const auto &v : points) {
		out << v.real() << ' ' << v.imag() << " 0\n";
	}
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[]) {
	if (argc <= 3) {
		std::cerr << "Usage: " << argv[0] << " points.xyz poly.obj result.xyz" << std::endl;
	}
	std::vector<Point> points = load_xyz(argv[1]);
	Polygon poly = load_obj(argv[2]);
	std::vector<Point> result;
	for (size_t i = 0; i < points.size(); ++i) {
		if (is_inside(poly, points[i])) {
			result.push_back(points[i]);
		}
	}
	save_xyz(argv[3], result);
	return 0;
}
