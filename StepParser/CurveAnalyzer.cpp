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
    // GEOMETRIC ANALYSIS HELPERS IMPLEMENTATION - SIMPLIFIED
    // =============================================================================
    
    bool CurveAnalyzer::IsBSplineCircular(
        const Handle(Geom_BSplineCurve)& bspline, 
        double tolerance,
        gp_Circ& fitted_circle) {
        
        // For now, return false to avoid complex geometric calculations
        // This will fall back to discretization, which is safer
        // TODO: Implement advanced circle fitting algorithms
        return false;
    }
    
    bool CurveAnalyzer::IsBSplineElliptical(
        const Handle(Geom_BSplineCurve)& bspline, 
        double tolerance,
        gp_Elips& fitted_ellipse) {
        
        // For now, return false to avoid complex geometric calculations  
        // This will fall back to discretization, which is safer
        // TODO: Implement advanced ellipse fitting algorithms
        return false;
    }
    
    bool CurveAnalyzer::AnalyzeControlPointPattern(
        const Handle(Geom_BSplineCurve)& bspline,
        double tolerance) {
        
        // TODO: Implement control point pattern analysis
        return false;
    }

} // namespace cad_slicer