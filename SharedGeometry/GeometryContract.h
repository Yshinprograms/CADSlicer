#pragma once

#include <vector>
#include <memory>

namespace geometry_contract {
    struct Point2D {
        double x;
        double y;
    };

    // =============================================================================
    // NATIVE CURVE TYPES - Enhanced geometry support for OVF format
    // =============================================================================
    
    struct Arc {
        Point2D center;
        double radius;
        double start_angle;     // Start angle in radians
        double sweep_angle;     // Sweep angle in radians (positive = counterclockwise)
        
        // Helper to check if arc is a full circle
        bool IsFullCircle() const {
            return std::abs(std::abs(sweep_angle) - 2.0 * 3.14159265359) < 1e-6;
        }
    };

    struct Ellipse {
        Point2D center;
        double semi_major_axis;
        double semi_minor_axis;
        double rotation_angle;  // Rotation of ellipse in radians
        double start_angle;     // Start angle in radians
        double sweep_angle;     // Sweep angle in radians
        
        // Helper to check if ellipse is a full ellipse
        bool IsFullEllipse() const {
            return std::abs(std::abs(sweep_angle) - 2.0 * 3.14159265359) < 1e-6;
        }
    };

    enum class CurveType {
        LineSequence,
        Arc,
        Ellipse
    };

    // =============================================================================
    // CURVE SEGMENT - C++14 compatible version using polymorphism
    // =============================================================================
    
    class CurveSegment {
    public:
        // Constructors for different curve types
        explicit CurveSegment(const std::vector<Point2D>& points);
        explicit CurveSegment(const Arc& arc);
        explicit CurveSegment(const Ellipse& ellipse);
        
        // Copy constructor and assignment
        CurveSegment(const CurveSegment& other);
        CurveSegment& operator=(const CurveSegment& other);
        
        // Destructor
        ~CurveSegment();

        CurveType GetType() const { return type_; }
        
        // Accessors for different curve types
        const std::vector<Point2D>& GetPoints() const;
        const Arc& GetArc() const;
        const Ellipse& GetEllipse() const;

        // Convert any curve segment to points for fallback compatibility
        std::vector<Point2D> ToPoints(double tolerance = 0.01) const;

    private:
        CurveType type_;
        
        // Union to hold different curve data types
        union CurveData {
            std::vector<Point2D> points;
            Arc arc;
            Ellipse ellipse;
            
            // Constructors for union members
            CurveData() {}
            ~CurveData() {}
        } data_;
        
        void ClearData();
        void CopyData(const CurveSegment& other);
    };

    // =============================================================================
    // ENHANCED CONTOUR - Supports both legacy points and native curves
    // =============================================================================
    
    struct Contour {
        std::vector<CurveSegment> segments;
        
        // Legacy constructor - converts points to line sequence
        explicit Contour(const std::vector<Point2D>& points) {
            if (!points.empty()) {
                segments.emplace_back(points);
            }
        }
        
        // Default constructor for modern usage
        Contour() = default;
        
        // Legacy support - converts entire contour to point-based representation
        std::vector<Point2D> ToPoints(double tolerance = 0.01) const {
            std::vector<Point2D> result;
            for (const auto& segment : segments) {
                auto segment_points = segment.ToPoints(tolerance);
                result.insert(result.end(), segment_points.begin(), segment_points.end());
            }
            return result;
        }
        
        // Legacy compatibility - access points from first segment if it's a line sequence
        const std::vector<Point2D>& points() const {
            static const std::vector<Point2D> empty_points;
            if (!segments.empty() && segments[0].GetType() == CurveType::LineSequence) {
                return segments[0].GetPoints();
            }
            return empty_points;
        }
        
        // Check if contour contains only line sequences (for backward compatibility)
        bool IsLegacyPointsOnly() const {
            return segments.size() == 1 && segments[0].GetType() == CurveType::LineSequence;
        }
        
        // Add convenience methods for building contours
        void AddLineSequence(const std::vector<Point2D>& points) {
            segments.emplace_back(points);
        }
        
        void AddArc(const Arc& arc) {
            segments.emplace_back(arc);
        }
        
        void AddEllipse(const Ellipse& ellipse) {
            segments.emplace_back(ellipse);
        }
    };

    struct SlicedLayer {
        double ZHeight;
        std::vector<Contour> contours;
    };

    // =============================================================================
    // HELPER FUNCTIONS - Utility functions for curve operations
    // =============================================================================
    
    namespace curve_utils {
        // Convert angle from degrees to radians
        inline double DegreesToRadians(double degrees) {
            return degrees * 3.14159265359 / 180.0;
        }
        
        // Convert angle from radians to degrees
        inline double RadiansToDegrees(double radians) {
            return radians * 180.0 / 3.14159265359;
        }
        
        // Normalize angle to [0, 2?]
        double NormalizeAngle(double angle);
        
        // Calculate point on arc at given parameter
        Point2D CalculateArcPoint(const Arc& arc, double parameter);
        
        // Calculate point on ellipse at given parameter
        Point2D CalculateEllipsePoint(const Ellipse& ellipse, double parameter);
        
        // Discretize arc into points
        std::vector<Point2D> DiscretizeArc(const Arc& arc, double tolerance);
        
        // Discretize ellipse into points
        std::vector<Point2D> DiscretizeEllipse(const Ellipse& ellipse, double tolerance);
    }
}
