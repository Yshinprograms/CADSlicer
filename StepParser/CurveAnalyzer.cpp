#include "CurveAnalyzer.h"

// --- OCCT Includes ---
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <gp_Pnt.hxx>
#include <gp_Circ.hxx>
#include <gp_Elips.hxx>
#include <gp_Vec.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax2.hxx>
#include <Standard_Real.hxx>
#include <GCPnts_UniformDeflection.hxx>

#include <cmath>
#include <algorithm>
#include <iostream>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace cad_slicer {

    // =============================================================================
    // PUBLIC INTERFACE IMPLEMENTATION
    // =============================================================================
    
    std::optional<geometry_contract::CurveSegment> CurveAnalyzer::AnalyzeCurve(
        const TopoDS_Edge& edge, double tolerance) {
        
        Handle(Geom_Curve) curve;
        double first_param, last_param;
        
        if (!ExtractCurveFromEdge(edge, curve, first_param, last_param)) {
            return std::nullopt;
        }
        
        // Handle trimmed curves
        Handle(Geom_TrimmedCurve) trimmed_curve = Handle(Geom_TrimmedCurve)::DownCast(curve);
        if (!trimmed_curve.IsNull()) {
            curve = trimmed_curve->BasisCurve();
            first_param = trimmed_curve->FirstParameter();
            last_param = trimmed_curve->LastParameter();
        }
        
        GeomAbs_CurveType curve_type = IdentifyCurveType(curve);
        
        switch (curve_type) {
            case GeomAbs_Circle: {
                auto arc = ConvertToArc(curve, first_param, last_param);
                geometry_contract::CurveSegment segment(arc);
                
                // Validate the conversion
                if (ValidateFittingAccuracy(curve, first_param, last_param, segment, tolerance)) {
                    std::cout << "[CurveAnalyzer] Converted circle to arc (radius: " 
                              << arc.radius << ", sweep: " << arc.sweep_angle << " rad)" << std::endl;
                    return segment;
                }
                break;
            }
            
            case GeomAbs_Ellipse: {
                auto ellipse = ConvertToEllipse(curve, first_param, last_param);
                geometry_contract::CurveSegment segment(ellipse);
                
                // Validate the conversion
                if (ValidateFittingAccuracy(curve, first_param, last_param, segment, tolerance)) {
                    std::cout << "[CurveAnalyzer] Converted ellipse (a: " 
                              << ellipse.semi_major_axis << ", b: " << ellipse.semi_minor_axis << ")" << std::endl;
                    return segment;
                }
                break;
            }
            
            case GeomAbs_BSplineCurve: {
                Handle(Geom_BSplineCurve) bspline = Handle(Geom_BSplineCurve)::DownCast(curve);
                if (!bspline.IsNull()) {
                    // Try to fit B-spline to circle first
                    auto fitted_arc = FitBSplineToCircle(bspline, tolerance);
                    if (fitted_arc.has_value()) {
                        std::cout << "[CurveAnalyzer] Fitted B-spline to circle (radius: " 
                                  << fitted_arc->radius << ")" << std::endl;
                        return geometry_contract::CurveSegment(*fitted_arc);
                    }
                    
                    // Try to fit B-spline to ellipse
                    auto fitted_ellipse = FitBSplineToEllipse(bspline, tolerance);
                    if (fitted_ellipse.has_value()) {
                        std::cout << "[CurveAnalyzer] Fitted B-spline to ellipse" << std::endl;
                        return geometry_contract::CurveSegment(*fitted_ellipse);
                    }
                }
                break;
            }
            
            default:
                break;
        }
        
        // Fallback: discretize to points
        auto points = DiscretizeCurveToPoints(curve, first_param, last_param, tolerance);
        if (!points.empty()) {
            std::cout << "[CurveAnalyzer] Discretized curve to " << points.size() << " points" << std::endl;
            return geometry_contract::CurveSegment(points);
        }
        
        return std::nullopt;
    }
    
    std::vector<geometry_contract::CurveSegment> CurveAnalyzer::AnalyzeWire(
        const TopoDS_Wire& wire, double tolerance) {
        
        std::vector<geometry_contract::CurveSegment> segments;
        
        for (TopExp_Explorer edge_explorer(wire, TopAbs_EDGE); edge_explorer.More(); edge_explorer.Next()) {
            TopoDS_Edge edge = TopoDS::Edge(edge_explorer.Current());
            
            auto segment = AnalyzeCurve(edge, tolerance);
            if (segment.has_value()) {
                segments.push_back(*segment);
            }
        }
        
        return segments;
    }
    
    std::optional<geometry_contract::Contour> CurveAnalyzer::AnalyzeWireToContour(
        const TopoDS_Wire& wire, double tolerance) {
        
        auto segments = AnalyzeWire(wire, tolerance);
        if (segments.empty()) {
            return std::nullopt;
        }
        
        geometry_contract::Contour contour;
        contour.segments = std::move(segments);
        
        return contour;
    }

    // =============================================================================
    // CURVE TYPE DETECTION IMPLEMENTATION
    // =============================================================================
    
    GeomAbs_CurveType CurveAnalyzer::IdentifyCurveType(const Handle(Geom_Curve)& curve) {
        if (curve.IsNull()) {
            return GeomAbs_OtherCurve;
        }
        
        GeomAdaptor_Curve adaptor(curve);
        return adaptor.GetType();
    }
    
    std::optional<GeomAbs_CurveType> CurveAnalyzer::AnalyzeBSplineGeometry(
        const Handle(Geom_BSplineCurve)& bspline, double tolerance) {
        
        if (bspline.IsNull()) {
            return std::nullopt;
        }
        
        gp_Circ fitted_circle;
        if (IsBSplineCircular(bspline, tolerance, fitted_circle)) {
            return GeomAbs_Circle;
        }
        
        gp_Elips fitted_ellipse;
        if (IsBSplineElliptical(bspline, tolerance, fitted_ellipse)) {
            return GeomAbs_Ellipse;
        }
        
        return std::nullopt;
    }

    // =============================================================================
    // CURVE CONVERSION METHODS IMPLEMENTATION
    // =============================================================================
    
    geometry_contract::Arc CurveAnalyzer::ConvertToArc(
        const Handle(Geom_Curve)& curve, double first, double last) {
        
        Handle(Geom_Circle) circle = Handle(Geom_Circle)::DownCast(curve);
        if (!circle.IsNull()) {
            return ConvertCircleToArc(circle->Circ(), first, last);
        }
        
        // Should not reach here if curve type detection worked correctly
        throw std::runtime_error("CurveAnalyzer::ConvertToArc called on non-circle curve");
    }
    
    geometry_contract::Arc CurveAnalyzer::ConvertCircleToArc(
        const gp_Circ& circle, double first, double last) {
        
        // Get circle properties
        gp_Pnt center = circle.Location();
        double radius = circle.Radius();
        
        // For simplicity, use the parametric curve representation instead of gp_Circ directly
        // Convert to start and end points using simple trigonometry
        double start_angle = first;
        double sweep_angle = last - first;
        
        return geometry_contract::Arc{
            {center.X(), center.Y()},
            radius,
            start_angle,
            sweep_angle
        };
    }
    
    geometry_contract::Ellipse CurveAnalyzer::ConvertToEllipse(
        const Handle(Geom_Curve)& curve, double first, double last) {
        
        Handle(Geom_Ellipse) ellipse = Handle(Geom_Ellipse)::DownCast(curve);
        if (!ellipse.IsNull()) {
            return ConvertEllipseToEllipse(ellipse->Elips(), first, last);
        }
        
        // Should not reach here if curve type detection worked correctly
        throw std::runtime_error("CurveAnalyzer::ConvertToEllipse called on non-ellipse curve");
    }
    
    geometry_contract::Ellipse CurveAnalyzer::ConvertEllipseToEllipse(
        const gp_Elips& ellipse, double first, double last) {
        
        gp_Pnt center = ellipse.Location();
        double major_radius = ellipse.MajorRadius();
        double minor_radius = ellipse.MinorRadius();
        
        // Get rotation from ellipse coordinate system
        gp_Dir x_dir = ellipse.XAxis().Direction();
        double rotation_angle = std::atan2(x_dir.Y(), x_dir.X());
        
        return geometry_contract::Ellipse{
            {center.X(), center.Y()},
            major_radius,
            minor_radius,
            rotation_angle,
            first,
            last - first
        };
    }
    
    std::optional<geometry_contract::Arc> CurveAnalyzer::FitBSplineToCircle(
        const Handle(Geom_BSplineCurve)& bspline, double tolerance) {
        
        gp_Circ fitted_circle;
        if (!IsBSplineCircular(bspline, tolerance, fitted_circle)) {
            return std::nullopt;
        }
        
        double first = bspline->FirstParameter();
        double last = bspline->LastParameter();
        
        return ConvertCircleToArc(fitted_circle, first, last);
    }
    
    std::optional<geometry_contract::Ellipse> CurveAnalyzer::FitBSplineToEllipse(
        const Handle(Geom_BSplineCurve)& bspline, double tolerance) {
        
        gp_Elips fitted_ellipse;
        if (!IsBSplineElliptical(bspline, tolerance, fitted_ellipse)) {
            return std::nullopt;
        }
        
        double first = bspline->FirstParameter();
        double last = bspline->LastParameter();
        
        return ConvertEllipseToEllipse(fitted_ellipse, first, last);
    }

    // =============================================================================
    // VALIDATION AND HELPER METHODS IMPLEMENTATION
    // =============================================================================
    
    std::vector<geometry_contract::Point2D> CurveAnalyzer::DiscretizeCurveToPoints(
        const Handle(Geom_Curve)& curve, double first, double last, double tolerance) {
        
        std::vector<geometry_contract::Point2D> points;
        
        if (curve.IsNull()) {
            return points;
        }
        
        // Use OCCT's uniform deflection discretization
        GeomAdaptor_Curve adaptor(curve, first, last);
        GCPnts_UniformDeflection discretizer;
        discretizer.Initialize(adaptor, tolerance);
        
        if (!discretizer.IsDone()) {
            return points;
        }
        
        for (int i = 1; i <= discretizer.NbPoints(); ++i) {
            gp_Pnt point = discretizer.Value(i);
            points.push_back({point.X(), point.Y()});
        }
        
        return points;
    }
    
    bool CurveAnalyzer::ValidateFittingAccuracy(
        const Handle(Geom_Curve)& original_curve,
        double first_param, double last_param,
        const geometry_contract::CurveSegment& fitted_segment, 
        double tolerance) {
        
        double max_deviation = CalculateMaxDeviation(
            original_curve, first_param, last_param, fitted_segment);
        
        return max_deviation <= tolerance;
    }
    
    double CurveAnalyzer::CalculateMaxDeviation(
        const Handle(Geom_Curve)& original_curve,
        double first_param, double last_param,
        const geometry_contract::CurveSegment& fitted_segment,
        int num_samples) {
        
        double max_deviation = 0.0;
        
        for (int i = 0; i <= num_samples; ++i) {
            double param = first_param + (last_param - first_param) * i / num_samples;
            gp_Pnt original_point = original_curve->Value(param);
            
            // Calculate corresponding point on fitted curve
            double fit_param = static_cast<double>(i) / num_samples;
            
            geometry_contract::Point2D fitted_point;
            switch (fitted_segment.GetType()) {
                case geometry_contract::CurveType::Arc:
                    fitted_point = geometry_contract::curve_utils::CalculateArcPoint(
                        fitted_segment.GetArc(), fit_param);
                    break;
                case geometry_contract::CurveType::Ellipse:
                    fitted_point = geometry_contract::curve_utils::CalculateEllipsePoint(
                        fitted_segment.GetEllipse(), fit_param);
                    break;
                default:
                    continue; // Skip unsupported types
            }
            
            double deviation = CalculateDistance(
                {original_point.X(), original_point.Y()}, fitted_point);
            max_deviation = std::max(max_deviation, deviation);
        }
        
        return max_deviation;
    }
    
    bool CurveAnalyzer::ExtractCurveFromEdge(
        const TopoDS_Edge& edge,
        Handle(Geom_Curve)& curve,
        double& first_param,
        double& last_param) {
        
        curve = BRep_Tool::Curve(edge, first_param, last_param);
        return !curve.IsNull();
    }
    
    geometry_contract::Point2D CurveAnalyzer::ConvertPoint(const gp_Pnt& pnt) {
        return {pnt.X(), pnt.Y()};
    }
    
    double CurveAnalyzer::CalculateDistance(
        const geometry_contract::Point2D& p1, 
        const geometry_contract::Point2D& p2) {
        
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    // =============================================================================
    // GEOMETRIC ANALYSIS HELPERS IMPLEMENTATION - ENHANCED
    // =============================================================================
    
    bool CurveAnalyzer::IsBSplineCircular(
        const Handle(Geom_BSplineCurve)& bspline, 
        double tolerance,
        gp_Circ& fitted_circle) {
        
        if (bspline.IsNull()) {
            return false;
        }
        
        // Try geometric analysis approach - this is more reliable than GeomConvert
        try {
            return AnalyzeBSplineCircularGeometry(bspline, tolerance, fitted_circle);
        }
        catch (const Standard_Failure&) {
            // Analysis failed
            return false;
        }
    }
    
    bool CurveAnalyzer::IsBSplineElliptical(
        const Handle(Geom_BSplineCurve)& bspline, 
        double tolerance,
        gp_Elips& fitted_ellipse) {
        
        if (bspline.IsNull()) {
            return false;
        }
        
        // Try geometric analysis approach
        try {
            return AnalyzeBSplineEllipticalGeometry(bspline, tolerance, fitted_ellipse);
        }
        catch (const Standard_Failure&) {
            // Analysis failed
            return false;
        }
    }
    
    bool CurveAnalyzer::AnalyzeControlPointPattern(
        const Handle(Geom_BSplineCurve)& bspline,
        double tolerance) {
        
        if (bspline.IsNull() || bspline->NbPoles() < 3) {
            return false;
        }
        
        // Analyze control point distribution for circular/elliptical patterns
        std::vector<gp_Pnt> control_points;
        for (int i = 1; i <= bspline->NbPoles(); ++i) {
            control_points.push_back(bspline->Pole(i));
        }
        
        // Check for uniform angular distribution (circular pattern)
        if (IsControlPointPatternCircular(control_points, tolerance)) {
            return true;
        }
        
        // Check for elliptical pattern
        if (IsControlPointPatternElliptical(control_points, tolerance)) {
            return true;
        }
        
        return false;
    }

    // =============================================================================
    // ADVANCED GEOMETRIC ANALYSIS METHODS
    // =============================================================================
    
    bool CurveAnalyzer::AnalyzeBSplineCircularGeometry(
        const Handle(Geom_BSplineCurve)& bspline,
        double tolerance,
        gp_Circ& fitted_circle) {
        
        if (bspline.IsNull() || bspline->NbPoles() < 3) {
            return false;
        }
        
        // Sample points along the curve
        std::vector<gp_Pnt> sample_points;
        double first = bspline->FirstParameter();
        double last = bspline->LastParameter();
        
        int num_samples = std::max(bspline->NbPoles(), 8);
        for (int i = 0; i <= num_samples; ++i) {
            double param = first + (last - first) * i / num_samples;
            sample_points.push_back(bspline->Value(param));
        }
        
        // Try to fit a circle through sample points
        return FitCircleToPoints(sample_points, tolerance, fitted_circle);
    }
    
    bool CurveAnalyzer::AnalyzeBSplineEllipticalGeometry(
        const Handle(Geom_BSplineCurve)& bspline,
        double tolerance,
        gp_Elips& fitted_ellipse) {
        
        if (bspline.IsNull() || bspline->NbPoles() < 5) {
            return false; // Need at least 5 points for ellipse fitting
        }
        
        // Sample points along the curve
        std::vector<gp_Pnt> sample_points;
        double first = bspline->FirstParameter();
        double last = bspline->LastParameter();
        
        int num_samples = std::max(bspline->NbPoles(), 10);
        for (int i = 0; i <= num_samples; ++i) {
            double param = first + (last - first) * i / num_samples;
            sample_points.push_back(bspline->Value(param));
        }
        
        // Try to fit an ellipse through sample points
        return FitEllipseToPoints(sample_points, tolerance, fitted_ellipse);
    }
    
    bool CurveAnalyzer::FitCircleToPoints(
        const std::vector<gp_Pnt>& points,
        double tolerance,
        gp_Circ& fitted_circle) {
        
        if (points.size() < 3) {
            return false;
        }
        
        // Use first, middle, and last points for initial circle
        gp_Pnt p1 = points[0];
        gp_Pnt p2 = points[points.size() / 2];
        gp_Pnt p3 = points.back();
        
        try {
            // Calculate circle center using perpendicular bisectors
            gp_Vec v1(p1, p2);
            gp_Vec v2(p2, p3);
            gp_Vec normal = v1 ^ v2;
            
            if (normal.Magnitude() < Precision::Confusion()) {
                return false; // Points are collinear
            }
            
            // Create coordinate system
            gp_Dir z_dir(normal);
            gp_Ax2 axis(p2, z_dir);
            
            // Calculate radius (simplified approach)
            double radius = p1.Distance(p2); // Initial approximation
            
            gp_Circ test_circle(axis, radius);
            
            // Validate against all points
            double max_deviation = 0.0;
            for (const auto& point : points) {
                double distance_to_center = test_circle.Location().Distance(point);
                double deviation = std::abs(distance_to_center - test_circle.Radius());
                max_deviation = std::max(max_deviation, deviation);
            }
            
            if (max_deviation <= tolerance) {
                fitted_circle = test_circle;
                std::cout << "[CurveAnalyzer] Successfully fitted circle (radius: " 
                          << fitted_circle.Radius() << ", deviation: " << max_deviation << ")" << std::endl;
                return true;
            }
        }
        catch (...) {
            // Circle fitting failed
        }
        
        return false;
    }
    
    bool CurveAnalyzer::FitEllipseToPoints(
        const std::vector<gp_Pnt>& points,
        double tolerance,
        gp_Elips& fitted_ellipse) {
        
        // Ellipse fitting is complex - for now, return false
        // This would require implementing least-squares ellipse fitting
        // or using more advanced OCCT utilities
        
        // TODO: Implement robust ellipse fitting algorithm
        // Could use algebraic fitting methods or geometric approaches
        
        std::cout << "[CurveAnalyzer] Ellipse fitting not yet implemented - falling back to discretization" << std::endl;
        return false;
    }
    
    bool CurveAnalyzer::IsControlPointPatternCircular(
        const std::vector<gp_Pnt>& control_points,
        double tolerance) {
        
        if (control_points.size() < 4) {
            return false;
        }
        
        // Check if control points follow a circular pattern
        // This is a simplified check - could be enhanced
        
        gp_Pnt center;
        if (!CalculateControlPointCenter(control_points, center)) {
            return false;
        }
        
        // Check if all points are approximately equidistant from center
        double reference_distance = center.Distance(control_points[0]);
        
        if (reference_distance < Precision::Confusion()) {
            return false; // All points at center
        }
        
        for (size_t i = 1; i < control_points.size(); ++i) {
            double distance = center.Distance(control_points[i]);
            double relative_deviation = std::abs(distance - reference_distance) / reference_distance;
            if (relative_deviation > tolerance) {
                return false;
            }
        }
        
        std::cout << "[CurveAnalyzer] Control points follow circular pattern (radius: " 
                  << reference_distance << ")" << std::endl;
        return true;
    }
    
    bool CurveAnalyzer::IsControlPointPatternElliptical(
        const std::vector<gp_Pnt>& control_points,
        double tolerance) {
        
        // TODO: Implement elliptical pattern recognition
        // This would involve checking for elliptical distribution of control points
        // For now, return false to use fallback discretization
        
        return false;
    }
    
    bool CurveAnalyzer::CalculateControlPointCenter(
        const std::vector<gp_Pnt>& control_points,
        gp_Pnt& center) {
        
        if (control_points.empty()) {
            return false;
        }
        
        // Calculate centroid
        double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
        
        for (const auto& point : control_points) {
            sum_x += point.X();
            sum_y += point.Y();
            sum_z += point.Z();
        }
        
        center = gp_Pnt(
            sum_x / control_points.size(),
            sum_y / control_points.size(),
            sum_z / control_points.size()
        );
        
        return true;
    }

} // namespace cad_slicer
