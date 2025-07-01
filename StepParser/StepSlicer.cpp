#include "StepSlicer.h"
#include "GeometryContract.h"

// --- OCCT Includes ---
#include <STEPControl_Reader.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
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

#include <optional>
#include <utility> // For std::move

namespace geometry {

    StepSlicer::StepSlicer(const std::string& step_file_path)
        : m_file_path(step_file_path) {
    }

    std::vector<geometry_contract::SlicedLayer> StepSlicer::Slice(double layer_height, double deflection_tolerance) {
        TopoDS_Shape model = LoadModel();
        if (model.IsNull()) {
            return {};
        }
        SlicingParameters params = CalculateSlicingParameters(model, layer_height, deflection_tolerance);
        return GenerateAllLayers(model, params);
    }

    TopoDS_Shape StepSlicer::LoadModel() const {
        STEPControl_Reader reader;
        if (reader.ReadFile(m_file_path.c_str()) != IFSelect_RetDone) {
            return {};
        }
        reader.TransferRoots();
        return reader.OneShape();
    }

    StepSlicer::SlicingParameters StepSlicer::CalculateSlicingParameters(const TopoDS_Shape& model, double layer_height, double deflection_tolerance) const {
        Bnd_Box bounding_box;
        BRepBndLib::Add(model, bounding_box);
        Standard_Real x_min, y_min, z_min, x_max, y_max, z_max;
        bounding_box.Get(x_min, y_min, z_min, x_max, y_max, z_max);

        return { z_min, z_max, layer_height, deflection_tolerance };
    }

    std::vector<geometry_contract::SlicedLayer> StepSlicer::GenerateAllLayers(const TopoDS_Shape& model, const SlicingParameters& params) const {
        std::vector<geometry_contract::SlicedLayer> all_layers;
        all_layers.reserve(static_cast<size_t>((params.z_max - params.z_min) / params.layer_height) + 1);

        for (double z = params.z_min; z <= params.z_max + 1e-9; z += params.layer_height) {
            auto current_layer = SliceSingleLayer(model, z, params.deflection_tolerance);
            if (!current_layer.contours.empty()) {
                all_layers.push_back(std::move(current_layer));
            }
        }
        return all_layers;
    }

    geometry_contract::SlicedLayer StepSlicer::SliceSingleLayer(const TopoDS_Shape& model, double z_height, double deflection_tolerance) const {
        geometry_contract::SlicedLayer layer;
        layer.ZHeight = z_height;
        TopoDS_Shape section_shape = PerformSection(model, z_height);
        if (!section_shape.IsNull()) {
            layer.contours = BuildContoursFromSection(section_shape, deflection_tolerance);
        }
        return layer;
    }

    TopoDS_Shape StepSlicer::PerformSection(const TopoDS_Shape& model, double z_height) const {
        gp_Pln slicing_plane(gp_Pnt(0, 0, z_height), gp_Dir(0, 0, 1));
        BRepAlgoAPI_Section section_algorithm(model, slicing_plane);
        section_algorithm.Build();
        return section_algorithm.IsDone() ? section_algorithm.Shape() : TopoDS_Shape();
    }

    std::vector<geometry_contract::Contour> StepSlicer::BuildContoursFromSection(const TopoDS_Shape& section_shape, double deflection_tolerance) const {
        std::vector<geometry_contract::Contour> all_contours;
        std::vector<TopoDS_Wire> wires = GetSectionWires(section_shape);
        for (const auto& wire : wires) {
            if (auto contour = BuildContourFromWire(wire, deflection_tolerance)) {
                all_contours.push_back(std::move(*contour));
            }
        }
        return all_contours;
    }

    std::vector<TopoDS_Wire> StepSlicer::GetSectionWires(const TopoDS_Shape& section_shape) const {
        std::vector<TopoDS_Wire> wires;
        TopExp_Explorer wire_explorer(section_shape, TopAbs_WIRE);
        while (wire_explorer.More()) {
            TopoDS_Wire wire = TopoDS::Wire(wire_explorer.Current());
            if (!wire.IsNull()) {
                wires.push_back(wire);
            }
            wire_explorer.Next();
        }
        return wires;
    }

    std::optional<geometry_contract::Contour> StepSlicer::BuildContourFromWire(const TopoDS_Wire& wire, double deflection_tolerance) const {
        geometry_contract::Contour assembled_contour;
        std::vector<TopoDS_Edge> ordered_edges = GetWireEdges(wire);
        for (const auto& edge : ordered_edges) {
            if (auto points_from_edge = DiscretizeEdge(edge, deflection_tolerance)) {
                if (assembled_contour.points.empty()) {
                    assembled_contour.points = std::move(*points_from_edge);
                }
                else {
                    assembled_contour.points.insert(assembled_contour.points.end(),
                        points_from_edge->begin() + 1,
                        points_from_edge->end());
                }
            }
        }
        if (assembled_contour.points.empty()) return std::nullopt;
        return assembled_contour;
    }

    std::vector<TopoDS_Edge> StepSlicer::GetWireEdges(const TopoDS_Wire& wire) const {
        std::vector<TopoDS_Edge> edges;
        BRepTools_WireExplorer ordered_edge_explorer(wire);
        while (ordered_edge_explorer.More()) {
            edges.push_back(ordered_edge_explorer.Current());
            ordered_edge_explorer.Next();
        }
        return edges;
    }

    std::optional<std::vector<geometry_contract::Point2D>> StepSlicer::DiscretizeEdge(const TopoDS_Edge& edge, double deflection_tolerance) const {
        Standard_Real first, last;
        // The Handle() macro is fine here as it expands to Handle<Geom_Curve>
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);
        if (curve.IsNull()) {
            return std::nullopt;
        }
        return DiscretizeCurve(curve, first, last, deflection_tolerance);
    }

    std::vector<geometry_contract::Point2D> StepSlicer::DiscretizeCurve(const opencascade::handle<Geom_Curve>& curve, Standard_Real first_param, Standard_Real last_param, double tolerance) const {
        GeomAdaptor_Curve adaptor(curve);
        GCPnts_UniformDeflection discretizer;
        discretizer.Initialize(adaptor, tolerance, first_param, last_param);

        if (!discretizer.IsDone()) {
            return {};
        }

        std::vector<geometry_contract::Point2D> points;
        points.reserve(discretizer.NbPoints());
        for (int i = 1; i <= discretizer.NbPoints(); ++i) {
            gp_Pnt p = discretizer.Value(i);
            points.push_back({ p.X(), p.Y() });
        }
        return points;
    }

} // namespace geometry