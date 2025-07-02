#include "StepSlicer.h"
#include "WireBuilder.h"
#include "GeometryContract.h"

// --- OCCT Includes ---
#include <STEPControl_Reader.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Vertex.hxx>
#include <BRep_Tool.hxx>
#include <gp_Pln.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <TopExp.hxx>
#include <TopAbs_ShapeEnum.hxx>

#include <optional>
#include <utility>
#include <iostream>

namespace cad_slicer {

    // =============================================================================
    // PUBLIC INTERFACE - SLAP Level 1: High-level orchestration
    // =============================================================================
    
    StepSlicer::StepSlicer(const std::string& step_file_path)
        : m_file_path(step_file_path) {
    }

    std::vector<geometry_contract::SlicedLayer> StepSlicer::Slice(double layer_height, double deflection_tolerance) {
        LogSlicingStart();
        
        TopoDS_Shape model = LoadAndValidateModel();
        if (model.IsNull()) {
            LogSlicingError("Model loading failed");
            return {};
        }

        SlicingParameters params = CalculateSlicingBounds(model, layer_height, deflection_tolerance);
        std::vector<geometry_contract::SlicedLayer> layers = GenerateAllSlices(model, params);
        
        LogSlicingComplete(layers.size());
        return layers;
    }

    // =============================================================================
    // PRIVATE HELPERS - SLAP Level 2: Mid-level operations
    // =============================================================================

    TopoDS_Shape StepSlicer::LoadAndValidateModel() const {
        TopoDS_Shape shape = LoadStepFile();
        if (shape.IsNull()) return {};
        
        if (!ValidateShape(shape)) return {};
        
        return ProcessShapeForSlicing(shape);
    }

    StepSlicer::SlicingParameters StepSlicer::CalculateSlicingBounds(const TopoDS_Shape& model, double layer_height, double deflection_tolerance) const {
        Bnd_Box bounds = CalculateBoundingBox(model);
        return ExtractSlicingParameters(bounds, layer_height, deflection_tolerance);
    }

    std::vector<geometry_contract::SlicedLayer> StepSlicer::GenerateAllSlices(const TopoDS_Shape& model, const SlicingParameters& params) const {
        std::vector<geometry_contract::SlicedLayer> layers;
        layers.reserve(EstimateLayerCount(params));

        for (double z = params.z_min; z < params.z_max; z += params.layer_height) {
            if (auto layer = TryCreateSliceAtHeight(model, z, params)) {
                layers.push_back(std::move(*layer));
            }
        }
        
        return layers;
    }

    std::optional<geometry_contract::SlicedLayer> StepSlicer::TryCreateSliceAtHeight(const TopoDS_Shape& model, double z_base, const SlicingParameters& params) const {
        double slice_z = CalculateSlicePosition(z_base, params.layer_height);
        
        if (IsSlicePositionValid(slice_z, params.z_max)) {
            auto layer = CreateSingleSlice(model, slice_z, z_base, params.deflection_tolerance);
            return HasValidContours(layer) ? std::make_optional(layer) : std::nullopt;
        }
        
        return std::nullopt;
    }

    geometry_contract::SlicedLayer StepSlicer::CreateSingleSlice(const TopoDS_Shape& model, double slice_z, double layer_z, double tolerance) const {
        geometry_contract::SlicedLayer layer;
        layer.ZHeight = layer_z;

        TopoDS_Shape section = PerformSliceOperation(model, slice_z);
        if (!section.IsNull()) {
            // Use WireBuilder to extract contours from the section
            layer.contours = WireBuilder::ExtractContoursFromSection(section, tolerance);
        }

        return layer;
    }

    // =============================================================================
    // PRIVATE HELPERS - SLAP Level 3: Low-level implementations
    // =============================================================================

    TopoDS_Shape StepSlicer::LoadStepFile() const {
        STEPControl_Reader reader;
        
        if (reader.ReadFile(m_file_path.c_str()) != IFSelect_RetDone) {
            return {};
        }
        
        reader.TransferRoots();
        return reader.OneShape();
    }

    bool StepSlicer::ValidateShape(const TopoDS_Shape& shape) const {
        if (shape.IsNull()) return false;
        
        BRepCheck_Analyzer analyzer(shape);
        if (!analyzer.IsValid()) {
            LogShapeValidationError();
            return false;
        }
        
        LogShapeInfo(shape);
        return true;
    }

    TopoDS_Shape StepSlicer::ProcessShapeForSlicing(const TopoDS_Shape& shape) const {
        if (IsDirectlySliceable(shape)) {
            return shape;
        }
        
        if (shape.ShapeType() == TopAbs_COMPOUND) {
            return ProcessCompoundShape(shape);
        }
        
        if (shape.ShapeType() == TopAbs_SHELL) {
            return ConvertShellToSolid(shape);
        }
        
        return {};
    }

    Bnd_Box StepSlicer::CalculateBoundingBox(const TopoDS_Shape& model) const {
        Bnd_Box box;
        BRepBndLib::Add(model, box);
        return box;
    }

    StepSlicer::SlicingParameters StepSlicer::ExtractSlicingParameters(const Bnd_Box& box, double layer_height, double tolerance) const {
        Standard_Real x_min, y_min, z_min, x_max, y_max, z_max;
        box.Get(x_min, y_min, z_min, x_max, y_max, z_max);
        
        return { z_min, z_max, layer_height, tolerance };
    }

    TopoDS_Shape StepSlicer::PerformSliceOperation(const TopoDS_Shape& model, double z_height) const {
        gp_Pln slicing_plane(gp_Pnt(0, 0, z_height), gp_Dir(0, 0, 1));
        BRepAlgoAPI_Section section_algorithm(model, slicing_plane);
        
        section_algorithm.Build();
        return section_algorithm.IsDone() ? section_algorithm.Shape() : TopoDS_Shape{};
    }

    // =============================================================================
    // UTILITY FUNCTIONS - SLAP Level 4: Basic operations
    // =============================================================================

    bool StepSlicer::IsDirectlySliceable(const TopoDS_Shape& shape) const {
        return shape.ShapeType() == TopAbs_SOLID || shape.ShapeType() == TopAbs_COMPSOLID;
    }

    TopoDS_Shape StepSlicer::ProcessCompoundShape(const TopoDS_Shape& compound) const {
        int solid_count = CountSubShapes(compound, TopAbs_SOLID);
        return solid_count > 0 ? compound : TopoDS_Shape{};
    }

    TopoDS_Shape StepSlicer::ConvertShellToSolid(const TopoDS_Shape& shell) const {
        BRepBuilderAPI_MakeSolid builder;
        builder.Add(TopoDS::Shell(shell));
        
        if (builder.IsDone()) {
            return builder.Solid();
        }
        
        return TryRepairAndConvert(shell);
    }

    TopoDS_Shape StepSlicer::TryRepairAndConvert(const TopoDS_Shape& shell) const {
        BRepBuilderAPI_Sewing sewer;
        sewer.Add(shell);
        sewer.Perform();
        
        TopoDS_Shape repaired = sewer.SewedShape();
        if (repaired.IsNull() || repaired.ShapeType() != TopAbs_SHELL) {
            return {};
        }
        
        BRepBuilderAPI_MakeSolid builder;
        builder.Add(TopoDS::Shell(repaired));
        
        if (builder.IsDone()) {
            return TopoDS_Shape(builder.Solid());
        } else {
            return TopoDS_Shape{};
        }
    }

    // =============================================================================
    // HELPER FUNCTIONS - SLAP Level 5: Basic utilities
    // =============================================================================

    size_t StepSlicer::EstimateLayerCount(const SlicingParameters& params) const {
        return static_cast<size_t>((params.z_max - params.z_min) / params.layer_height) + 1;
    }

    bool StepSlicer::HasValidContours(const geometry_contract::SlicedLayer& layer) const {
        return !layer.contours.empty();
    }

    double StepSlicer::CalculateSlicePosition(double z_base, double layer_height) const {
        return z_base + (layer_height / 2.0);
    }

    bool StepSlicer::IsSlicePositionValid(double slice_z, double z_max) const {
        return slice_z <= z_max;
    }

    int StepSlicer::CountSubShapes(const TopoDS_Shape& shape, TopAbs_ShapeEnum type) const {
        int count = 0;
        for (TopExp_Explorer exp(shape, type); exp.More(); exp.Next()) {
            count++;
        }
        return count;
    }

    // =============================================================================
    // LOGGING FUNCTIONS - Separated for clarity
    // =============================================================================

    void StepSlicer::LogSlicingStart() const {
        std::cout << "[StepSlicer] Starting slicing process..." << std::endl;
    }

    void StepSlicer::LogSlicingComplete(size_t layer_count) const {
        std::cout << "[StepSlicer] Slicing complete. Generated " << layer_count << " layers." << std::endl;
    }

    void StepSlicer::LogSlicingError(const std::string& message) const {
        std::cerr << "[StepSlicer] ERROR: " << message << std::endl;
    }

    void StepSlicer::LogShapeValidationError() const {
        std::cerr << "[StepSlicer] ERROR: Shape validation failed." << std::endl;
    }

    void StepSlicer::LogShapeInfo(const TopoDS_Shape& shape) const {
        int solids = CountSubShapes(shape, TopAbs_SOLID);
        int shells = CountSubShapes(shape, TopAbs_SHELL);
        int faces = CountSubShapes(shape, TopAbs_FACE);
        
        std::cout << "[StepSlicer] Shape loaded: " << solids << " solids, " 
                  << shells << " shells, " << faces << " faces" << std::endl;
    }

} // namespace cad_slicer