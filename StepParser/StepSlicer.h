#pragma once

#include <string>
#include <vector>
#include <optional>

// --- OCCT Includes and Forward Declarations ---

// Include the definition of the Handle template class
#include <Standard_Handle.hxx>
#include <Standard_Real.hxx>

// Forward-declare the classes we use in pointers or references
class TopoDS_Shape;
class TopoDS_Edge;
class TopoDS_Wire;
class Geom_Curve;

#include "GeometryContract.h"

namespace geometry {

    class StepSlicer {
    public:
        explicit StepSlicer(const std::string& step_file_path);
        std::vector<geometry_contract::SlicedLayer> Slice(double layer_height, double deflection_tolerance = 0.1);

    private:
        struct SlicingParameters {
            double z_min;
            double z_max;
            double layer_height;
            double deflection_tolerance;
        };

        // --- Private Helper Methods ---
        TopoDS_Shape LoadModel() const;
        SlicingParameters CalculateSlicingParameters(const TopoDS_Shape& model, double layer_height, double deflection_tolerance) const;
        std::vector<geometry_contract::SlicedLayer> GenerateAllLayers(const TopoDS_Shape& model, const SlicingParameters& params) const;
        geometry_contract::SlicedLayer SliceSingleLayer(const TopoDS_Shape& model, double z_height, double deflection_tolerance) const;
        TopoDS_Shape PerformSection(const TopoDS_Shape& model, double z_height) const;
        std::vector<geometry_contract::Contour> BuildContoursFromSection(const TopoDS_Shape& section_shape, double deflection_tolerance) const;
        std::optional<geometry_contract::Contour> BuildContourFromWire(const TopoDS_Wire& wire, double deflection_tolerance) const;
        std::vector<TopoDS_Wire> GetSectionWires(const TopoDS_Shape& section_shape) const;
        std::vector<TopoDS_Edge> GetWireEdges(const TopoDS_Wire& wire) const;
        std::optional<std::vector<geometry_contract::Point2D>> DiscretizeEdge(const TopoDS_Edge& edge, double deflection_tolerance) const;

        // CORRECTED: Use the full template syntax with the forward-declared class
        std::vector<geometry_contract::Point2D> DiscretizeCurve(const opencascade::handle<Geom_Curve>& curve, Standard_Real first_param, Standard_Real last_param, double tolerance) const;

        std::string m_file_path;
    };

} // namespace geometry