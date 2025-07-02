#pragma once

#include <string>
#include <vector>
#include <optional>

// --- OCCT Forward Declarations ---
#include <Standard_Handle.hxx>
#include <Standard_Real.hxx>
#include <TopAbs_ShapeEnum.hxx>

class TopoDS_Shape;
class TopoDS_Edge;
class TopoDS_Wire;
class TopoDS_Vertex;
class Geom_Curve;
class Bnd_Box;

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

        // =============================================================================
        // SLAP Level 2: Mid-level operations
        // =============================================================================
        TopoDS_Shape LoadAndValidateModel() const;
        SlicingParameters CalculateSlicingBounds(const TopoDS_Shape& model, double layer_height, double deflection_tolerance) const;
        std::vector<geometry_contract::SlicedLayer> GenerateAllSlices(const TopoDS_Shape& model, const SlicingParameters& params) const;
        geometry_contract::SlicedLayer CreateSingleSlice(const TopoDS_Shape& model, double slice_z, double layer_z, double tolerance) const;

        // =============================================================================
        // SLAP Level 3: Low-level implementations
        // =============================================================================
        TopoDS_Shape LoadStepFile() const;
        bool ValidateShape(const TopoDS_Shape& shape) const;
        TopoDS_Shape ProcessShapeForSlicing(const TopoDS_Shape& shape) const;
        Bnd_Box CalculateBoundingBox(const TopoDS_Shape& model) const;
        SlicingParameters ExtractSlicingParameters(const Bnd_Box& box, double layer_height, double tolerance) const;
        TopoDS_Shape PerformSliceOperation(const TopoDS_Shape& model, double z_height) const;
        std::vector<geometry_contract::Contour> ExtractContoursFromSection(const TopoDS_Shape& section, double tolerance) const;
        std::vector<TopoDS_Wire> ExtractWires(const TopoDS_Shape& section) const;
        std::optional<geometry_contract::Contour> ConvertWireToContour(const TopoDS_Wire& wire, double tolerance) const;

        // =============================================================================
        // SLAP Level 4: Basic operations
        // =============================================================================
        bool IsDirectlySliceable(const TopoDS_Shape& shape) const;
        TopoDS_Shape ProcessCompoundShape(const TopoDS_Shape& compound) const;
        TopoDS_Shape ConvertShellToSolid(const TopoDS_Shape& shell) const;
        TopoDS_Shape TryRepairAndConvert(const TopoDS_Shape& shell) const;
        std::vector<TopoDS_Wire> FindExistingWires(const TopoDS_Shape& section) const;
        std::vector<TopoDS_Wire> BuildWiresFromLooseEdges(const TopoDS_Shape& section) const;
        std::vector<TopoDS_Edge> FindLooseEdges(const TopoDS_Shape& section) const;
        std::vector<TopoDS_Wire> AssembleWiresFromEdges(const std::vector<TopoDS_Edge>& edges) const;
        std::optional<TopoDS_Wire> BuildWireFromStartingEdge(const std::vector<TopoDS_Edge>& edges, std::vector<bool>& used, size_t start_idx) const;
        std::vector<TopoDS_Edge> GetOrderedEdges(const TopoDS_Wire& wire) const;
        std::optional<std::vector<geometry_contract::Point2D>> DiscretizeEdge(const TopoDS_Edge& edge, double tolerance) const;
        std::vector<geometry_contract::Point2D> DiscretizeCurve(const opencascade::handle<Geom_Curve>& curve, Standard_Real first, Standard_Real last, double tolerance) const;

        // =============================================================================
        // SLAP Level 5: Basic utilities
        // =============================================================================
        size_t EstimateLayerCount(const SlicingParameters& params) const;
        bool HasValidContours(const geometry_contract::SlicedLayer& layer) const;
        int CountSubShapes(const TopoDS_Shape& shape, TopAbs_ShapeEnum type) const;
        TopoDS_Vertex GetWireEndVertex(const TopoDS_Wire& wire) const;
        bool EdgeConnectsToVertex(const TopoDS_Edge& edge, const TopoDS_Vertex& vertex) const;
        void AppendPointsToContour(geometry_contract::Contour& contour, const std::vector<geometry_contract::Point2D>& points) const;

        // =============================================================================
        // Logging functions (separated for clarity)
        // =============================================================================
        void LogSlicingStart() const;
        void LogSlicingComplete(size_t layer_count) const;
        void LogSlicingError(const std::string& message) const;
        void LogShapeValidationError() const;
        void LogShapeInfo(const TopoDS_Shape& shape) const;

        std::string m_file_path;
    };

} // namespace geometry