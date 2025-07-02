#include "StepSlicer.h"
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
#include <Geom_Curve.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GCPnts_UniformDeflection.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <TopExp.hxx>
#include <TopAbs_ShapeEnum.hxx>

#include <optional>
#include <utility>
#include <iostream>

namespace geometry {

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

        const double slice_offset = params.layer_height / 2.0;
        
        for (double z = params.z_min; z < params.z_max; z += params.layer_height) {
            double slice_z = z + slice_offset;
            if (slice_z > params.z_max) break;

            auto layer = CreateSingleSlice(model, slice_z, z, params.deflection_tolerance);
            if (HasValidContours(layer)) {
                layers.push_back(std::move(layer));
            }
        }
        
        return layers;
    }

    geometry_contract::SlicedLayer StepSlicer::CreateSingleSlice(const TopoDS_Shape& model, double slice_z, double layer_z, double tolerance) const {
        geometry_contract::SlicedLayer layer;
        layer.ZHeight = layer_z;

        TopoDS_Shape section = PerformSliceOperation(model, slice_z);
        if (!section.IsNull()) {
            layer.contours = ExtractContoursFromSection(section, tolerance);
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

    std::vector<geometry_contract::Contour> StepSlicer::ExtractContoursFromSection(const TopoDS_Shape& section, double tolerance) const {
        std::vector<TopoDS_Wire> wires = ExtractWires(section);
        std::vector<geometry_contract::Contour> contours;
        
        for (const auto& wire : wires) {
            if (auto contour = ConvertWireToContour(wire, tolerance)) {
                contours.push_back(std::move(*contour));
            }
        }
        
        return contours;
    }

    std::vector<TopoDS_Wire> StepSlicer::ExtractWires(const TopoDS_Shape& section) const {
        std::vector<TopoDS_Wire> wires = FindExistingWires(section);
        
        if (wires.empty()) {
            wires = BuildWiresFromLooseEdges(section);
        }
        
        return wires;
    }

    std::optional<geometry_contract::Contour> StepSlicer::ConvertWireToContour(const TopoDS_Wire& wire, double tolerance) const {
        std::vector<TopoDS_Edge> edges = GetOrderedEdges(wire);
        geometry_contract::Contour contour;
        
        for (const auto& edge : edges) {
            if (auto points = DiscretizeEdge(edge, tolerance)) {
                AppendPointsToContour(contour, *points);
            }
        }
        
        return contour.points.empty() ? std::nullopt : std::make_optional(contour);
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

    std::vector<TopoDS_Wire> StepSlicer::FindExistingWires(const TopoDS_Shape& section) const {
        std::vector<TopoDS_Wire> wires;
        for (TopExp_Explorer exp(section, TopAbs_WIRE); exp.More(); exp.Next()) {
            wires.push_back(TopoDS::Wire(exp.Current()));
        }
        return wires;
    }

    std::vector<TopoDS_Wire> StepSlicer::BuildWiresFromLooseEdges(const TopoDS_Shape& section) const {
        std::vector<TopoDS_Edge> edges = FindLooseEdges(section);
        return AssembleWiresFromEdges(edges);
    }

    std::vector<TopoDS_Edge> StepSlicer::FindLooseEdges(const TopoDS_Shape& section) const {
        std::vector<TopoDS_Edge> edges;
        for (TopExp_Explorer exp(section, TopAbs_EDGE, TopAbs_WIRE); exp.More(); exp.Next()) {
            edges.push_back(TopoDS::Edge(exp.Current()));
        }
        return edges;
    }

    std::vector<TopoDS_Wire> StepSlicer::AssembleWiresFromEdges(const std::vector<TopoDS_Edge>& edges) const {
        std::vector<TopoDS_Wire> wires;
        std::vector<bool> used(edges.size(), false);

        for (size_t i = 0; i < edges.size(); ++i) {
            if (used[i]) continue;
            
            if (auto wire = BuildWireFromStartingEdge(edges, used, i)) {
                wires.push_back(*wire);
            }
        }

        return wires;
    }

    std::optional<TopoDS_Wire> StepSlicer::BuildWireFromStartingEdge(const std::vector<TopoDS_Edge>& edges, std::vector<bool>& used, size_t start_idx) const {
        BRepBuilderAPI_MakeWire builder;
        builder.Add(edges[start_idx]);
        used[start_idx] = true;

        bool found_connection = true;
        while (found_connection && builder.IsDone()) {
            found_connection = false;
            TopoDS_Vertex end_vertex = GetWireEndVertex(builder.Wire());

            for (size_t i = 0; i < edges.size(); ++i) {
                if (used[i]) continue;
                
                if (EdgeConnectsToVertex(edges[i], end_vertex)) {
                    builder.Add(edges[i]);
                    if (builder.IsDone()) {
                        used[i] = true;
                        found_connection = true;
                        break;
                    }
                }
            }
        }

        return builder.IsDone() ? std::make_optional(builder.Wire()) : std::nullopt;
    }

    std::vector<TopoDS_Edge> StepSlicer::GetOrderedEdges(const TopoDS_Wire& wire) const {
        std::vector<TopoDS_Edge> edges;
        for (BRepTools_WireExplorer exp(wire); exp.More(); exp.Next()) {
            edges.push_back(exp.Current());
        }
        return edges;
    }

    std::optional<std::vector<geometry_contract::Point2D>> StepSlicer::DiscretizeEdge(const TopoDS_Edge& edge, double tolerance) const {
        Standard_Real first, last;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);
        
        if (curve.IsNull()) return std::nullopt;
        
        return DiscretizeCurve(curve, first, last, tolerance);
    }

    std::vector<geometry_contract::Point2D> StepSlicer::DiscretizeCurve(const opencascade::handle<Geom_Curve>& curve, Standard_Real first, Standard_Real last, double tolerance) const {
        GeomAdaptor_Curve adaptor(curve);
        GCPnts_UniformDeflection discretizer;
        discretizer.Initialize(adaptor, tolerance, first, last);

        if (!discretizer.IsDone()) return {};

        std::vector<geometry_contract::Point2D> points;
        points.reserve(discretizer.NbPoints());
        
        for (int i = 1; i <= discretizer.NbPoints(); ++i) {
            gp_Pnt p = discretizer.Value(i);
            points.push_back({p.X(), p.Y()});
        }
        
        return points;
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

    int StepSlicer::CountSubShapes(const TopoDS_Shape& shape, TopAbs_ShapeEnum type) const {
        int count = 0;
        for (TopExp_Explorer exp(shape, type); exp.More(); exp.Next()) {
            count++;
        }
        return count;
    }

    TopoDS_Vertex StepSlicer::GetWireEndVertex(const TopoDS_Wire& wire) const {
        TopoDS_Vertex start, end;
        TopExp::Vertices(wire, start, end);
        return end;
    }

    bool StepSlicer::EdgeConnectsToVertex(const TopoDS_Edge& edge, const TopoDS_Vertex& vertex) const {
        TopoDS_Vertex start, end;
        TopExp::Vertices(edge, start, end);
        return vertex.IsSame(start) || vertex.IsSame(end);
    }

    void StepSlicer::AppendPointsToContour(geometry_contract::Contour& contour, const std::vector<geometry_contract::Point2D>& points) const {
        if (contour.points.empty()) {
            contour.points = points;
        } else {
            contour.points.insert(contour.points.end(), points.begin() + 1, points.end());
        }
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

} // namespace geometry