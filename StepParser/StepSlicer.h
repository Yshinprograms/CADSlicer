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
class BRepBuilderAPI_MakeWire;

#include "GeometryContract.h"

namespace cad_slicer {

    class StepSlicer {
    public:
        explicit StepSlicer(const std::string& step_file_path);
        
        // =============================================================================
        // SLAP Level 1: Slice()
        // =============================================================================
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
        std::optional<geometry_contract::SlicedLayer> TryCreateSliceAtHeight(const TopoDS_Shape& model, double z_base, const SlicingParameters& params) const;
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

        // =============================================================================
        // SLAP Level 4: Basic operations
        // =============================================================================
        bool IsDirectlySliceable(const TopoDS_Shape& shape) const;
        TopoDS_Shape ProcessCompoundShape(const TopoDS_Shape& compound) const;
        TopoDS_Shape ConvertShellToSolid(const TopoDS_Shape& shell) const;
        TopoDS_Shape TryRepairAndConvert(const TopoDS_Shape& shell) const;

        // =============================================================================
        // SLAP Level 5: Basic utilities
        // =============================================================================
        size_t EstimateLayerCount(const SlicingParameters& params) const;
        bool HasValidContours(const geometry_contract::SlicedLayer& layer) const;
        double CalculateSlicePosition(double z_base, double layer_height) const;
        bool IsSlicePositionValid(double slice_z, double z_max) const;
        int CountSubShapes(const TopoDS_Shape& shape, TopAbs_ShapeEnum type) const;

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

} // namespace cad_slicer