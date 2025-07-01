#pragma once

#include <vector>

namespace geometry_contract {
	struct Point2D {
		double x;
		double y;
	};

	struct Contour {
		std::vector<Point2D> points;
	};

	struct SlicedLayer {
		double ZHeight;
		std::vector<Contour> contours;
	};
}