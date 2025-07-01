// StepSlicer.h

#pragma once

#include <string>
#include <vector>
#include <optional>

// Forward declarations for OCCT classes to reduce header dependencies
class TopoDS_Shape;
class TopoDS_Edge;

#include "GeometryContract.h"

namespace geometry {

    class StepSlicer {
    public:
        explicit StepSlicer(const std::string& step_file_path);
        std::vector<geometry_contract::SlicedLayer> Slice(double layer_height);

    private:
        struct SlicingParameters {
            double z_min;
            double z_max;
            double layer_height;
        };

        TopoDS_Shape LoadModel() const;
        SlicingParameters CalculateSlicingParameters(const TopoDS_Shape& model, double layer_height) const;
        std::vector<geometry_contract::SlicedLayer> GenerateAllLayers(const TopoDS_Shape& model, const SlicingParameters& params) const;


        geometry_contract::SlicedLayer SliceSingleLayer(const TopoDS_Shape& model, double z_height) const;
        std::optional<geometry_contract::Contour> DiscretizeEdge(const TopoDS_Edge& edge) const;

        // --- Member Variables ---
        std::string m_file_path;
    };

} // namespace geometry