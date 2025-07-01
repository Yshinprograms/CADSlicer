// StepSlicer.cpp

#include "StepSlicer.h"
#include "GeometryContract.h"

// --- OCCT Includes ---
#include <STEPControl_Reader.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <BRep_Tool.hxx>
#include <gp_Pln.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <Geom_Curve.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GCPnts_UniformDeflection.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>

#include <optional>

namespace geometry {

    constexpr double kDeflectionTolerance = 0.1;

    StepSlicer::StepSlicer(const std::string& step_file_path)
        : m_file_path(step_file_path) {
    }

    std::vector<geometry_contract::SlicedLayer> StepSlicer::Slice(double layer_height) {
        // Chapter 1: Get the main character, our 3D model.
        TopoDS_Shape model = LoadModel();
        if (model.IsNull()) {
            return {}; // The story ends if the model can't be loaded.
        }

        // Chapter 2: Create the plan for our adventure.
        SlicingParameters params = CalculateSlicingParameters(model, layer_height);

        // Chapter 3: Execute the plan and get the treasure.
        return GenerateAllLayers(model, params);
    }

    //================================================================
    // Chapter 1: Loading the Model
    //================================================================
    TopoDS_Shape StepSlicer::LoadModel() const {
        STEPControl_Reader reader;
        if (reader.ReadFile(m_file_path.c_str()) != IFSelect_RetDone) {
            return {};
        }
        reader.TransferRoots();
        return reader.OneShape();
    }

    //================================================================
    // Chapter 2: Calculating the Slicing Plan
    //================================================================
    StepSlicer::SlicingParameters StepSlicer::CalculateSlicingParameters(const TopoDS_Shape& model, double layer_height) const {
        Bnd_Box bounding_box;
        BRepBndLib::Add(model, bounding_box);
        Standard_Real x_min, y_min, z_min, x_max, y_max, z_max;
        bounding_box.Get(x_min, y_min, z_min, x_max, y_max, z_max);

        return { z_min, z_max, layer_height };
    }

    //================================================================
    // Chapter 3: Generating All the Layers
    // This method orchestrates the main loop, delegating the actual work.
    //================================================================
    std::vector<geometry_contract::SlicedLayer> StepSlicer::GenerateAllLayers(const TopoDS_Shape& model, const SlicingParameters& params) const {
        std::vector<geometry_contract::SlicedLayer> all_layers;

        // The loop is an implementation detail, now hidden from the main Slice() method.
        for (double z = params.z_min; z <= params.z_max + 1e-9; z += params.layer_height) {
            auto current_layer = SliceSingleLayer(model, z);
            if (!current_layer.contours.empty()) {
                all_layers.push_back(current_layer);
            }
        }
        return all_layers;
    }

    //================================================================
    // Lower-Level Implementation Details
    // These functions are the "how", not the "what".
    //================================================================
    geometry_contract::SlicedLayer StepSlicer::SliceSingleLayer(const TopoDS_Shape& model, double z_height) const {
        geometry_contract::SlicedLayer layer;
        layer.ZHeight = z_height;

        gp_Pln slicing_plane(gp_Pnt(0, 0, z_height), gp_Dir(0, 0, 1));
        BRepAlgoAPI_Section section_algorithm(model, slicing_plane);
        section_algorithm.Build();

        if (section_algorithm.IsDone()) {
            TopExp_Explorer explorer(section_algorithm.Shape(), TopAbs_EDGE);
            while (explorer.More()) {
                auto edge = TopoDS::Edge(explorer.Current());
                if (auto contour = DiscretizeEdge(edge)) {
                    layer.contours.push_back(*contour);
                }
                explorer.Next();
            }
        }
        return layer;
    }

    std::optional<geometry_contract::Contour> StepSlicer::DiscretizeEdge(const TopoDS_Edge& edge) const {
        Standard_Real first, last;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);

        if (curve.IsNull()) {
            return std::nullopt;
        }

        GeomAdaptor_Curve adaptor(curve);
        GCPnts_UniformDeflection discretizer;
        discretizer.Initialize(adaptor, kDeflectionTolerance, first, last);

        if (!discretizer.IsDone()) {
            return std::nullopt;
        }

        geometry_contract::Contour contour;
        for (int i = 1; i <= discretizer.NbPoints(); ++i) {
            gp_Pnt point = discretizer.Value(i);
            contour.points.push_back({ point.X(), point.Y() });
        }
        return contour;
    }

} // namespace geometry