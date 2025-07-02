#include "GeometryContract.h"
#include <cmath>
#include <algorithm>
#include <new>
#include <stdexcept>

namespace geometry_contract {

    // =============================================================================
    // CURVE SEGMENT IMPLEMENTATION
    // =============================================================================
    
    CurveSegment::CurveSegment(const std::vector<Point2D>& points) : type_(CurveType::LineSequence) {
        new (&data_.points) std::vector<Point2D>(points);
    }
    
    CurveSegment::CurveSegment(const Arc& arc) : type_(CurveType::Arc) {
        new (&data_.arc) Arc(arc);
    }
    
    CurveSegment::CurveSegment(const Ellipse& ellipse) : type_(CurveType::Ellipse) {
        new (&data_.ellipse) Ellipse(ellipse);
    }
    
    CurveSegment::CurveSegment(const CurveSegment& other) : type_(other.type_) {
        CopyData(other);
    }
    
    CurveSegment& CurveSegment::operator=(const CurveSegment& other) {
        if (this != &other) {
            ClearData();
            type_ = other.type_;
            CopyData(other);
        }
        return *this;
    }
    
    CurveSegment::~CurveSegment() {
        ClearData();
    }
    
    void CurveSegment::ClearData() {
        switch (type_) {
            case CurveType::LineSequence:
                data_.points.~vector();
                break;
            case CurveType::Arc:
                // Arc is trivially destructible
                break;
            case CurveType::Ellipse:
                // Ellipse is trivially destructible
                break;
        }
    }
    
    void CurveSegment::CopyData(const CurveSegment& other) {
        switch (other.type_) {
            case CurveType::LineSequence:
                new (&data_.points) std::vector<Point2D>(other.data_.points);
                break;
            case CurveType::Arc:
                new (&data_.arc) Arc(other.data_.arc);
                break;
            case CurveType::Ellipse:
                new (&data_.ellipse) Ellipse(other.data_.ellipse);
                break;
        }
    }
    
    const std::vector<Point2D>& CurveSegment::GetPoints() const {
        if (type_ != CurveType::LineSequence) {
            throw std::runtime_error("CurveSegment is not a LineSequence");
        }
        return data_.points;
    }
    
    const Arc& CurveSegment::GetArc() const {
        if (type_ != CurveType::Arc) {
            throw std::runtime_error("CurveSegment is not an Arc");
        }
        return data_.arc;
    }
    
    const Ellipse& CurveSegment::GetEllipse() const {
        if (type_ != CurveType::Ellipse) {
            throw std::runtime_error("CurveSegment is not an Ellipse");
        }
        return data_.ellipse;
    }
    
    std::vector<Point2D> CurveSegment::ToPoints(double tolerance) const {
        switch (type_) {
            case CurveType::LineSequence:
                return data_.points;
                
            case CurveType::Arc:
                return curve_utils::DiscretizeArc(data_.arc, tolerance);
                
            case CurveType::Ellipse:
                return curve_utils::DiscretizeEllipse(data_.ellipse, tolerance);
                
            default:
                return {}; // Should never happen
        }
    }

    // =============================================================================
    // CURVE UTILITIES IMPLEMENTATION
    // =============================================================================
    
    namespace curve_utils {
        
        double NormalizeAngle(double angle) {
            const double TWO_PI = 2.0 * 3.14159265359;
            while (angle < 0.0) angle += TWO_PI;
            while (angle >= TWO_PI) angle -= TWO_PI;
            return angle;
        }
        
        Point2D CalculateArcPoint(const Arc& arc, double parameter) {
            double angle = arc.start_angle + parameter * arc.sweep_angle;
            return {
                arc.center.x + arc.radius * std::cos(angle),
                arc.center.y + arc.radius * std::sin(angle)
            };
        }
        
        Point2D CalculateEllipsePoint(const Ellipse& ellipse, double parameter) {
            double angle = ellipse.start_angle + parameter * ellipse.sweep_angle;
            
            // Calculate point on canonical ellipse
            double x_canonical = ellipse.semi_major_axis * std::cos(angle);
            double y_canonical = ellipse.semi_minor_axis * std::sin(angle);
            
            // Apply rotation
            double cos_rot = std::cos(ellipse.rotation_angle);
            double sin_rot = std::sin(ellipse.rotation_angle);
            
            return {
                ellipse.center.x + x_canonical * cos_rot - y_canonical * sin_rot,
                ellipse.center.y + x_canonical * sin_rot + y_canonical * cos_rot
            };
        }
        
        std::vector<Point2D> DiscretizeArc(const Arc& arc, double tolerance) {
            std::vector<Point2D> points;
            
            if (arc.radius <= 0.0 || std::abs(arc.sweep_angle) < 1e-10) {
                return points;
            }
            
            // Calculate number of segments based on tolerance
            // Use chord height error as criterion: h = r * (1 - cos(?/2))
            // Solve for ? when h = tolerance: ? = 2 * acos(1 - tolerance/r)
            double max_chord_angle = 2.0 * std::acos(std::max(0.0, 1.0 - tolerance / arc.radius));
            
            // Ensure minimum number of segments for smooth curves
            int num_segments = std::max(4, static_cast<int>(std::ceil(std::abs(arc.sweep_angle) / max_chord_angle)));
            
            // Generate points
            points.reserve(num_segments + 1);
            for (int i = 0; i <= num_segments; ++i) {
                double parameter = static_cast<double>(i) / num_segments;
                points.push_back(CalculateArcPoint(arc, parameter));
            }
            
            return points;
        }
        
        std::vector<Point2D> DiscretizeEllipse(const Ellipse& ellipse, double tolerance) {
            std::vector<Point2D> points;
            
            if (ellipse.semi_major_axis <= 0.0 || ellipse.semi_minor_axis <= 0.0 || 
                std::abs(ellipse.sweep_angle) < 1e-10) {
                return points;
            }
            
            // For ellipses, use adaptive discretization based on curvature
            // Approximate using the smaller semi-axis for conservative estimation
            double effective_radius = std::min(ellipse.semi_major_axis, ellipse.semi_minor_axis);
            double max_chord_angle = 2.0 * std::acos(std::max(0.0, 1.0 - tolerance / effective_radius));
            
            // Ensure minimum number of segments
            int num_segments = std::max(8, static_cast<int>(std::ceil(std::abs(ellipse.sweep_angle) / max_chord_angle)));
            
            // Generate points
            points.reserve(num_segments + 1);
            for (int i = 0; i <= num_segments; ++i) {
                double parameter = static_cast<double>(i) / num_segments;
                points.push_back(CalculateEllipsePoint(ellipse, parameter));
            }
            
            return points;
        }
        
    } // namespace curve_utils
    
} // namespace geometry_contract