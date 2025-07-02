#include "StepSlicer.h"
#include "GeometryContract.h"

// --- OCCT Includes ---
#include <STEPControl_Reader.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Solid.hxx>
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
#include <TopoDS_Vertex.hxx>

#include <optional>
#include <utility> // For std::move

namespace geometry {

    StepSlicer::StepSlicer(const std::string& step_file_path)
        : m_file_path(step_file_path) {
    }

    std::vector<geometry_contract::SlicedLayer> StepSlicer::Slice(double layer_height, double deflection_tolerance) {
        std::cout << "[StepSlicer] Starting slicing process..." << std::endl;

        TopoDS_Shape model = LoadModel();
        if (model.IsNull()) {
            // The error happens here, so this message is key!
            std::cerr << "[StepSlicer] ERROR: LoadModel() failed and returned a null shape." << std::endl;
            return {};
        }

        SlicingParameters params = CalculateSlicingParameters(model, layer_height, deflection_tolerance);

        auto all_layers = GenerateAllLayers(model, params);

        std::cout << "[StepSlicer] Slicing process finished. Returning " << all_layers.size() << " layers." << std::endl;
        return all_layers;

        std::cout << "[StepSlicer] Slicing process finished. Returning " << all_layers.size() << " layers." << std::endl;
        return all_layers;
    }

    TopoDS_Shape StepSlicer::LoadModel() const {
        std::cout << "  [LoadModel] Attempting to read file: " << m_file_path << std::endl;
        STEPControl_Reader reader;

        IFSelect_ReturnStatus status = reader.ReadFile(m_file_path.c_str());
        if (status != IFSelect_RetDone) {
            std::cerr << "    [LoadModel] ERROR: STEPControl_Reader::ReadFile() failed with status code: " << status << std::endl;
            return {};
        }
        std::cout << "    [LoadModel] File read successfully. Transferring roots..." << std::endl;
        reader.TransferRoots();

        TopoDS_Shape loaded_shape = reader.OneShape();
        if (loaded_shape.IsNull()) {
            std::cerr << "    [LoadModel] ERROR: Shape is null after transferring roots." << std::endl;
            return {};
        }

        std::cout << "    [LoadModel] Initial shape loaded. ShapeType: " << loaded_shape.ShapeType() << std::endl;

        // Add more detailed shape type information
        std::cout << "    [LoadModel] Shape type details: ";
        switch (loaded_shape.ShapeType()) {
        case TopAbs_COMPOUND: std::cout << "COMPOUND"; break;
        case TopAbs_COMPSOLID: std::cout << "COMPSOLID"; break;
        case TopAbs_SOLID: std::cout << "SOLID"; break;
        case TopAbs_SHELL: std::cout << "SHELL"; break;
        case TopAbs_FACE: std::cout << "FACE"; break;
        case TopAbs_WIRE: std::cout << "WIRE"; break;
        case TopAbs_EDGE: std::cout << "EDGE"; break;
        case TopAbs_VERTEX: std::cout << "VERTEX"; break;
        default: std::cout << "UNKNOWN"; break;
        }
        std::cout << std::endl;

        //================================================================
        // THE FINAL DIAGNOSTIC: Check the validity of the loaded shape.
        //================================================================
        std::cout << "    [LoadModel] Performing validity check on the loaded shape..." << std::endl;
        BRepCheck_Analyzer anAnalyzer(loaded_shape);

        if (!anAnalyzer.IsValid()) {
            std::cerr << "    [LoadModel] ERROR: THE LOADED SHAPE IS NOT VALID." << std::endl;
            std::cerr << "      This is the likely cause of the slicing failure." << std::endl;
            // The shape is invalid, so we stop here.
            return {};
        }

        std::cout << "    [LoadModel] Shape validity check passed." << std::endl;

        // Count sub-shapes for better understanding
        int solidCount = 0, shellCount = 0, faceCount = 0;
        for (TopExp_Explorer exp(loaded_shape, TopAbs_SOLID); exp.More(); exp.Next()) solidCount++;
        for (TopExp_Explorer exp(loaded_shape, TopAbs_SHELL); exp.More(); exp.Next()) shellCount++;
        for (TopExp_Explorer exp(loaded_shape, TopAbs_FACE); exp.More(); exp.Next()) faceCount++;

        std::cout << "    [LoadModel] Shape contains: " << solidCount << " solids, "
            << shellCount << " shells, " << faceCount << " faces" << std::endl;

        // The solid-building logic is still good practice for other files, so we keep it.
        if (loaded_shape.ShapeType() >= TopAbs_SOLID) {
            return loaded_shape;
        }

        if (loaded_shape.ShapeType() < TopAbs_SHELL) {
            std::cerr << "    [LoadModel] ERROR: Loaded shape is not a shell or solid, cannot create solid." << std::endl;
            return {};
        }

        std::cout << "    [LoadModel] Shape is a Shell. Attempting to build a solid from it..." << std::endl;

        BRepBuilderAPI_MakeSolid solid_builder;
        solid_builder.Add(TopoDS::Shell(loaded_shape));

        if (solid_builder.IsDone()) {
            // THE FIX: The result of .Solid() is a TopoDS_Solid. It can be directly
            // returned as a TopoDS_Shape because of inheritance. No cast needed.
            std::cout << "    [LoadModel] Successfully built a solid from the shell." << std::endl;
            return solid_builder.Solid();
        }
        else {
            std::cerr << "      [LoadModel] WARNING: MakeSolid failed. Trying to sew faces first..." << std::endl;
            BRepBuilderAPI_Sewing sewed_shell_builder;
            sewed_shell_builder.Add(loaded_shape);
            sewed_shell_builder.Perform();
            TopoDS_Shape sewed_shell = sewed_shell_builder.SewedShape();

            if (!sewed_shell.IsNull() && sewed_shell.ShapeType() == TopAbs_SHELL) {
                // Re-initialize the builder or use a new one
                BRepBuilderAPI_MakeSolid solid_builder_from_sewed;
                solid_builder_from_sewed.Add(TopoDS::Shell(sewed_shell));
                if (solid_builder_from_sewed.IsDone()) {
                    std::cout << "    [LoadModel] Successfully built a solid from the SEWED shell." << std::endl;
                    // THE FIX: Same as above.
                    return solid_builder_from_sewed.Solid();
                }
            }
        }

        std::cerr << "    [LoadModel] ERROR: Failed to build a solid from the loaded shape." << std::endl;
        return {};
    }

    StepSlicer::SlicingParameters StepSlicer::CalculateSlicingParameters(const TopoDS_Shape& model, double layer_height, double deflection_tolerance) const {
        std::cout << "  [CalculateSlicingParameters] Calculating model bounds..." << std::endl;
        Bnd_Box bounding_box;
        BRepBndLib::Add(model, bounding_box);
        Standard_Real x_min, y_min, z_min, x_max, y_max, z_max;
        bounding_box.Get(x_min, y_min, z_min, x_max, y_max, z_max);

        std::cout << "    [CalculateSlicingParameters] Z-range found: " << z_min << " to " << z_max << std::endl;
        std::cout << "    [CalculateSlicingParameters] Full bounds: X(" << x_min << " to " << x_max
            << "), Y(" << y_min << " to " << y_max << "), Z(" << z_min << " to " << z_max << ")" << std::endl;

        return { z_min, z_max, layer_height, deflection_tolerance };
    }

    std::vector<geometry_contract::SlicedLayer> StepSlicer::GenerateAllLayers(const TopoDS_Shape& model, const SlicingParameters& params) const {
        std::cout << "  [GenerateAllLayers] Starting to generate layers..." << std::endl;
        std::vector<geometry_contract::SlicedLayer> all_layers;
        all_layers.reserve(static_cast<size_t>((params.z_max - params.z_min) / params.layer_height) + 1);

        // THE FIX: Define a small epsilon to offset the slice plane
        // This avoids degenerate intersections on flat faces.
        // We choose half the layer height to ensure we are centered within the layer.
        const double fuzz = params.layer_height / 2.0;

        int layer_count = 0;

        // The loop now stops *before* the last layer height to avoid slicing above the model.
        for (double z_base = params.z_min; z_base < params.z_max; z_base += params.layer_height) {

            // The actual slice happens in the middle of the conceptual layer
            double z_slice = z_base + fuzz;

            // Don't slice past the top of the model
            if (z_slice > params.z_max) {
                std::cout << "    [GenerateAllLayers] Skipping slice at Z = " << z_slice << " (beyond model bounds)" << std::endl;
                break;
            }

            std::cout << "    [GenerateAllLayers] Attempting to slice layer " << ++layer_count << " at Z = " << z_slice << std::endl;

            // Pass the fuzzed z_slice height, not the base height
            auto current_layer = SliceSingleLayer(model, z_slice, params.deflection_tolerance);

            // We will store the layer with its base height for consistency
            current_layer.ZHeight = z_base;

            if (!current_layer.contours.empty()) {
                all_layers.push_back(std::move(current_layer));
            }
            else {
                std::cout << "      [GenerateAllLayers] WARNING: Slice at Z = " << z_slice << " produced no contours." << std::endl;
            }
        }
        std::cout << "  [GenerateAllLayers] Finished generating layers." << std::endl;
        return all_layers;
    }

    geometry_contract::SlicedLayer StepSlicer::SliceSingleLayer(const TopoDS_Shape& model, double z_height, double deflection_tolerance) const {
        geometry_contract::SlicedLayer layer;
        // Note: We will set the final ZHeight in the calling function (GenerateAllLayers)
        layer.ZHeight = z_height;

        TopoDS_Shape section_shape = PerformSection(model, z_height);

        if (!section_shape.IsNull()) {
            std::cout << "        [SliceSingleLayer] Section shape is valid, building contours..." << std::endl;
            layer.contours = BuildContoursFromSection(section_shape, deflection_tolerance);
        }
        else {
            std::cout << "        [SliceSingleLayer] WARNING: Section shape is null!" << std::endl;
        }

        std::cout << "        [SliceSingleLayer] Found " << layer.contours.size() << " contours for this layer." << std::endl;

        return layer;
    }

    TopoDS_Shape StepSlicer::PerformSection(const TopoDS_Shape& model, double z_height) const {
        gp_Pln slicing_plane(gp_Pnt(0, 0, z_height), gp_Dir(0, 0, 1));
        BRepAlgoAPI_Section section_algorithm(model, slicing_plane);

        // Add logging around the Build() call
        std::cout << "          [PerformSection] Building section..." << std::endl;
        section_algorithm.Build();

        if (!section_algorithm.IsDone()) {
            std::cerr << "            [PerformSection] ERROR: BRepAlgoAPI_Section::Build() failed." << std::endl;
            return {};
        }

        std::cout << "          [PerformSection] Section built successfully." << std::endl;

        TopoDS_Shape result = section_algorithm.Shape();

        // Add detailed analysis of the section result
        if (result.IsNull()) {
            std::cout << "            [PerformSection] WARNING: Section result is null." << std::endl;
        }
        else {
            std::cout << "            [PerformSection] Section result type: ";
            switch (result.ShapeType()) {
            case TopAbs_COMPOUND: std::cout << "COMPOUND"; break;
            case TopAbs_COMPSOLID: std::cout << "COMPSOLID"; break;
            case TopAbs_SOLID: std::cout << "SOLID"; break;
            case TopAbs_SHELL: std::cout << "SHELL"; break;
            case TopAbs_FACE: std::cout << "FACE"; break;
            case TopAbs_WIRE: std::cout << "WIRE"; break;
            case TopAbs_EDGE: std::cout << "EDGE"; break;
            case TopAbs_VERTEX: std::cout << "VERTEX"; break;
            default: std::cout << "UNKNOWN"; break;
            }
            std::cout << std::endl;

            // Count elements in the section
            int edgeCount = 0, wireCount = 0, vertexCount = 0;
            for (TopExp_Explorer exp(result, TopAbs_EDGE); exp.More(); exp.Next()) edgeCount++;
            for (TopExp_Explorer exp(result, TopAbs_WIRE); exp.More(); exp.Next()) wireCount++;
            for (TopExp_Explorer exp(result, TopAbs_VERTEX); exp.More(); exp.Next()) vertexCount++;

            std::cout << "            [PerformSection] Section contains: " << wireCount << " wires, "
                << edgeCount << " edges, " << vertexCount << " vertices" << std::endl;
        }

        return result;
    }

    std::vector<geometry_contract::Contour> StepSlicer::BuildContoursFromSection(const TopoDS_Shape& section_shape, double deflection_tolerance) const {
        std::cout << "          [BuildContoursFromSection] Starting contour building..." << std::endl;

        std::vector<geometry_contract::Contour> all_contours;
        std::vector<TopoDS_Wire> wires = GetSectionWires(section_shape);

        std::cout << "          [BuildContoursFromSection] Found " << wires.size() << " wires in section." << std::endl;

        for (size_t i = 0; i < wires.size(); ++i) {
            std::cout << "            [BuildContoursFromSection] Processing wire " << (i + 1) << "..." << std::endl;
            if (auto contour = BuildContourFromWire(wires[i], deflection_tolerance)) {
                std::cout << "            [BuildContoursFromSection] Wire " << (i + 1) << " converted to contour with "
                    << contour->points.size() << " points." << std::endl;
                all_contours.push_back(std::move(*contour));
            }
            else {
                std::cout << "            [BuildContoursFromSection] WARNING: Wire " << (i + 1) << " failed to convert to contour." << std::endl;
            }
        }

        std::cout << "          [BuildContoursFromSection] Finished building " << all_contours.size() << " contours." << std::endl;
        return all_contours;
    }

    std::vector<TopoDS_Wire> StepSlicer::GetSectionWires(const TopoDS_Shape& section_shape) const {
        std::cout << "            [GetSectionWires] Searching for wires in section..." << std::endl;

        std::vector<TopoDS_Wire> wires;
        TopExp_Explorer wire_explorer(section_shape, TopAbs_WIRE);

        int wireCount = 0;
        while (wire_explorer.More()) {
            TopoDS_Wire wire = TopoDS::Wire(wire_explorer.Current());
            if (!wire.IsNull()) {
                wireCount++;
                std::cout << "            [GetSectionWires] Found valid wire " << wireCount << std::endl;
                wires.push_back(wire);
            }
            else {
                std::cout << "            [GetSectionWires] WARNING: Found null wire, skipping." << std::endl;
            }
            wire_explorer.Next();
        }

        // If no wires found, try to build wires from isolated edges
        if (wires.empty()) {
            std::cout << "            [GetSectionWires] No wires found, attempting to build wires from isolated edges..." << std::endl;

            std::vector<TopoDS_Edge> edges;
            TopExp_Explorer edge_explorer(section_shape, TopAbs_EDGE, TopAbs_WIRE);

            while (edge_explorer.More()) {
                TopoDS_Edge edge = TopoDS::Edge(edge_explorer.Current());
                if (!edge.IsNull()) {
                    edges.push_back(edge);
                }
                edge_explorer.Next();
            }

            std::cout << "            [GetSectionWires] Found " << edges.size() << " isolated edges, building wires..." << std::endl;

            if (!edges.empty()) {
                wires = BuildWiresFromEdges(edges);
                std::cout << "            [GetSectionWires] Successfully built " << wires.size() << " wires from edges." << std::endl;
            }
        }

        std::cout << "            [GetSectionWires] Returning " << wires.size() << " wires." << std::endl;
        return wires;
    }

    std::vector<TopoDS_Wire> StepSlicer::BuildWiresFromEdges(const std::vector<TopoDS_Edge>& edges) const {
        std::cout << "              [BuildWiresFromEdges] Building wires from " << edges.size() << " edges..." << std::endl;

        std::vector<TopoDS_Wire> wires;
        std::vector<bool> used_edges(edges.size(), false);

        for (size_t start_idx = 0; start_idx < edges.size(); ++start_idx) {
            if (used_edges[start_idx]) {
                continue; // Already used in another wire
            }

            std::cout << "              [BuildWiresFromEdges] Starting new wire with edge " << (start_idx + 1) << std::endl;

            BRepBuilderAPI_MakeWire wire_builder;
            std::vector<size_t> current_wire_edges;

            // Start with this edge
            wire_builder.Add(edges[start_idx]);
            used_edges[start_idx] = true;
            current_wire_edges.push_back(start_idx);

            // Try to find connected edges
            bool found_connection = true;
            while (found_connection && current_wire_edges.size() < edges.size()) {
                found_connection = false;

                // Get the current end vertices of the wire being built
                TopoDS_Vertex current_start, current_end;
                if (wire_builder.IsDone()) {
                    TopoDS_Wire current_wire = wire_builder.Wire();
                    TopExp::Vertices(current_wire, current_start, current_end); // Removed the third parameter
                }

                // Look for an unused edge that connects to current_end
                for (size_t edge_idx = 0; edge_idx < edges.size(); ++edge_idx) {
                    if (used_edges[edge_idx]) {
                        continue;
                    }

                    TopoDS_Vertex edge_start, edge_end;
                    TopExp::Vertices(edges[edge_idx], edge_start, edge_end);

                    // Check if this edge connects to the current wire's end
                    if ((!current_end.IsNull() && edge_start.IsSame(current_end)) ||
                        (!current_end.IsNull() && edge_end.IsSame(current_end))) {

                        std::cout << "                [BuildWiresFromEdges] Adding connected edge " << (edge_idx + 1) << " to wire" << std::endl;

                        wire_builder.Add(edges[edge_idx]);
                        if (wire_builder.IsDone()) {
                            used_edges[edge_idx] = true;
                            current_wire_edges.push_back(edge_idx);
                            found_connection = true;
                            break;
                        } else {
                            std::cout << "                [BuildWiresFromEdges] WARNING: Failed to add edge " << (edge_idx + 1) << " to wire" << std::endl;
                        }
                    }
                }
            }

            if (wire_builder.IsDone()) {
                TopoDS_Wire completed_wire = wire_builder.Wire();
                wires.push_back(completed_wire);
                std::cout << "              [BuildWiresFromEdges] Successfully built wire " << wires.size()
                    << " with " << current_wire_edges.size() << " edges" << std::endl;
            } else {
                std::cout << "              [BuildWiresFromEdges] WARNING: Failed to build wire starting with edge " << (start_idx + 1) << std::endl;
                // Mark the edge as unused again so it might be picked up later
                used_edges[start_idx] = false;
            }
        }

        std::cout << "              [BuildWiresFromEdges] Finished building " << wires.size() << " wires from edges." << std::endl;
        return wires;
    }

    std::optional<geometry_contract::Contour> StepSlicer::BuildContourFromWire(const TopoDS_Wire& wire, double deflection_tolerance) const {
        std::cout << "              [BuildContourFromWire] Building contour from wire..." << std::endl;

        geometry_contract::Contour assembled_contour;
        std::vector<TopoDS_Edge> ordered_edges = GetWireEdges(wire);

        std::cout << "              [BuildContourFromWire] Wire contains " << ordered_edges.size() << " edges." << std::endl;

        for (size_t i = 0; i < ordered_edges.size(); ++i) {
            std::cout << "              [BuildContourFromWire] Processing edge " << (i + 1) << "..." << std::endl;
            if (auto points_from_edge = DiscretizeEdge(ordered_edges[i], deflection_tolerance)) {
                std::cout << "              [BuildContourFromWire] Edge " << (i + 1) << " discretized to "
                    << points_from_edge->size() << " points." << std::endl;
                if (assembled_contour.points.empty()) {
                    assembled_contour.points = std::move(*points_from_edge);
                }
                else {
                    assembled_contour.points.insert(assembled_contour.points.end(),
                        points_from_edge->begin() + 1,
                        points_from_edge->end());
                }
            }
            else {
                std::cout << "              [BuildContourFromWire] WARNING: Edge " << (i + 1) << " failed to discretize." << std::endl;
            }
        }

        if (assembled_contour.points.empty()) {
            std::cout << "              [BuildContourFromWire] ERROR: No points generated from wire." << std::endl;
            return std::nullopt;
        }

        std::cout << "              [BuildContourFromWire] Successfully built contour with "
            << assembled_contour.points.size() << " points." << std::endl;
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
            std::cout << "                [DiscretizeEdge] WARNING: Edge has no 3D curve." << std::endl;
            return std::nullopt;
        }
        return DiscretizeCurve(curve, first, last, deflection_tolerance);
    }

    std::vector<geometry_contract::Point2D> StepSlicer::DiscretizeCurve(const opencascade::handle<Geom_Curve>& curve, Standard_Real first_param, Standard_Real last_param, double tolerance) const {
        GeomAdaptor_Curve adaptor(curve);
        GCPnts_UniformDeflection discretizer;
        discretizer.Initialize(adaptor, tolerance, first_param, last_param);

        if (!discretizer.IsDone()) {
            std::cout << "                [DiscretizeCurve] WARNING: Curve discretization failed." << std::endl;
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