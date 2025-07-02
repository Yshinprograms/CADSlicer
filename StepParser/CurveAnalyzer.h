#pragma once

#include <vector>
#include <optional>
#include "../SharedGeometry/GeometryContract.h"

// --- OCCT Forward Declarations ---
#include <Standard_Handle.hxx>
#include <Standard_Real.hxx>
#include <GeomAbs_CurveType.hxx>

class TopoDS_Edge;
class TopoDS_Wire;
class Geom_Curve;
class Geom_Circle;
class Geom_Ellipse;
class Geom_BSplineCurve;
class gp_Circ;
class gp_Elips;
class gp_Pnt;

namespace cad_slicer {

    /**
     * @brief Analyzes OCCT curves and converts B-splines to native arcs and ellipses when possible.
     * 
     * This class provides the core functionality for the post-processing module that
     * utilizes the resolution of B-splines from STEP files to recreate native geometric
     * primitives that are supported by the OVF format.
     */
    class CurveAnalyzer {
    public:
        // =============================================================================
        // PUBLIC INTERFACE - High-level curve analysis
        // =============================================================================
        
        /// Analyzes an edge and attempts to convert B-splines to native curves
        static std::optional<geometry_contract::CurveSegment> AnalyzeCurve(
            const TopoDS_Edge& edge, double tolerance = 0.01);
        
        /// Processes a complete wire and extracts optimal curve representations
        static std::vector<geometry_contract::CurveSegment> AnalyzeWire(
            const TopoDS_Wire& wire, double tolerance = 0.01);
            
        /// Enhanced contour extraction that preserves curve geometry
        static std::optional<geometry_contract::Contour> AnalyzeWireToContour(
            const TopoDS_Wire& wire, double tolerance = 0.01);

    private:
        // =============================================================================
        // CURVE TYPE DETECTION
        // =============================================================================
        
        /// Determines the geometric type of a curve
        static GeomAbs_CurveType IdentifyCurveType(const Handle(Geom_Curve)& curve);
        
        /// Checks if a B-spline can be converted to a simpler form
        static std::optional<GeomAbs_CurveType> AnalyzeBSplineGeometry(
            const Handle(Geom_BSplineCurve)& bspline, double tolerance);

        // =============================================================================
        // CURVE CONVERSION METHODS
        // =============================================================================
        
        /// Converts a detected circle to Arc representation
        static geometry_contract::Arc ConvertToArc(
            const Handle(Geom_Curve)& curve, double first, double last);
            
        /// Converts a circle directly from gp_Circ
        static geometry_contract::Arc ConvertCircleToArc(
            const gp_Circ& circle, double first, double last);
        
        /// Converts a detected ellipse to Ellipse representation
        static geometry_contract::Ellipse ConvertToEllipse(
            const Handle(Geom_Curve)& curve, double first, double last);
            
        /// Converts an ellipse directly from gp_Elips
        static geometry_contract::Ellipse ConvertEllipseToEllipse(
            const gp_Elips& ellipse, double first, double last);
        
        /// Attempts to fit a B-spline to a circle
        static std::optional<geometry_contract::Arc> FitBSplineToCircle(
            const Handle(Geom_BSplineCurve)& bspline, double tolerance);
        
        /// Attempts to fit a B-spline to an ellipse
        static std::optional<geometry_contract::Ellipse> FitBSplineToEllipse(
            const Handle(Geom_BSplineCurve)& bspline, double tolerance);

        // =============================================================================
        // VALIDATION AND HELPER METHODS
        // =============================================================================
        
        /// Discretizes curve to points as fallback
        static std::vector<geometry_contract::Point2D> DiscretizeCurveToPoints(
            const Handle(Geom_Curve)& curve, double first, double last, double tolerance);
            
        /// Validates curve fitting accuracy by sampling points
        static bool ValidateFittingAccuracy(
            const Handle(Geom_Curve)& original_curve,
            double first_param, double last_param,
            const geometry_contract::CurveSegment& fitted_segment, 
            double tolerance);
            
        /// Calculates the maximum deviation between original curve and fitted curve
        static double CalculateMaxDeviation(
            const Handle(Geom_Curve)& original_curve,
            double first_param, double last_param,
            const geometry_contract::CurveSegment& fitted_segment,
            int num_samples = 20);
            
        /// Helper to extract curve geometry from edge
        static bool ExtractCurveFromEdge(
            const TopoDS_Edge& edge,
            Handle(Geom_Curve)& curve,
            double& first_param,
            double& last_param);
            
        /// Convert OCCT point to geometry_contract point
        static geometry_contract::Point2D ConvertPoint(const gp_Pnt& pnt);
        
        /// Calculate distance between two 2D points
        static double CalculateDistance(
            const geometry_contract::Point2D& p1, 
            const geometry_contract::Point2D& p2);

        // =============================================================================
        // GEOMETRIC ANALYSIS HELPERS
        // =============================================================================
        
        /// Check if B-spline represents a circular arc within tolerance
        static bool IsBSplineCircular(
            const Handle(Geom_BSplineCurve)& bspline, 
            double tolerance,
            gp_Circ& fitted_circle);
            
        /// Check if B-spline represents an elliptical arc within tolerance
        static bool IsBSplineElliptical(
            const Handle(Geom_BSplineCurve)& bspline, 
            double tolerance,
            gp_Elips& fitted_ellipse);
            
        /// Analyze control points pattern for geometric recognition
        static bool AnalyzeControlPointPattern(
            const Handle(Geom_BSplineCurve)& bspline,
            double tolerance);
    };

} // namespace cad_slicer