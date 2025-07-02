#include "WireBuilder.h"
#include "CurveAnalyzer.h"

// --- OCCT Includes ---
#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Vertex.hxx>
#include <BRep_Tool.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <Geom_Curve.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GCPnts_UniformDeflection.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <TopExp.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <gp_Pnt.hxx>

#include <optional>
#include <utility>
#include <iostream>

namespace cad_slicer {

    // =============================================================================
    // PUBLIC INTERFACE - High-level wire operations
    // =============================================================================

    std::vector<geometry_contract::Contour> WireBuilder::ExtractContoursFromSection(const TopoDS_Shape& section, double tolerance) {
        std::vector<TopoDS_Wire> wires = ExtractWires(section);
        std::vector<geometry_contract::Contour> contours;
        
        for (const auto& wire : wires) {
            if (auto contour = ConvertWireToContour(wire, tolerance)) {
                contours.push_back(std::move(*contour));
            }
        }
        
        return contours;
    }

    std::vector<geometry_contract::Contour> WireBuilder::ExtractOptimalContoursFromSection(const TopoDS_Shape& section, double tolerance) {
        std::vector<TopoDS_Wire> wires = ExtractWires(section);
        std::vector<geometry_contract::Contour> contours;
        
        std::cout << "[WireBuilder] Processing " << wires.size() << " wires for optimal curve extraction" << std::endl;
        
        for (const auto& wire : wires) {
            if (auto contour = ConvertWireToOptimalContour(wire, tolerance)) {
                contours.push_back(std::move(*contour));
            }
        }
        
        std::cout << "[WireBuilder] Successfully extracted " << contours.size() << " optimal contours" << std::endl;
        return contours;
    }

    std::vector<TopoDS_Wire> WireBuilder::ExtractWires(const TopoDS_Shape& section) {
        std::vector<TopoDS_Wire> wires = FindExistingWires(section);
        
        if (wires.empty()) {
            wires = BuildWiresFromLooseEdges(section);
        }
        
        return wires;
    }

    std::optional<geometry_contract::Contour> WireBuilder::ConvertWireToContour(const TopoDS_Wire& wire, double tolerance) {
        std::vector<TopoDS_Edge> edges = GetOrderedEdges(wire);
        geometry_contract::Contour contour;
        
        for (const auto& edge : edges) {
            if (auto points = DiscretizeEdge(edge, tolerance)) {
                AppendPointsToContour(contour, *points);
            }
        }
        
        return contour.points().empty() ? std::nullopt : std::make_optional(contour);
    }

    std::optional<geometry_contract::Contour> WireBuilder::ConvertWireToOptimalContour(const TopoDS_Wire& wire, double tolerance) {
        // Use CurveAnalyzer to get optimal curve representation
        auto optimal_contour = CurveAnalyzer::AnalyzeWireToContour(wire, tolerance);
        
        if (optimal_contour.has_value()) {
            std::cout << "[WireBuilder] Successfully converted wire to optimal contour with " 
                      << optimal_contour->segments.size() << " curve segments" << std::endl;
                      
            // Log curve segment types for debugging
            for (size_t i = 0; i < optimal_contour->segments.size(); ++i) {
                const auto& segment = optimal_contour->segments[i];
                switch (segment.GetType()) {
                    case geometry_contract::CurveType::Arc:
                        std::cout << "  Segment " << i << ": Arc (radius: " 
                                  << segment.GetArc().radius << ")" << std::endl;
                        break;
                    case geometry_contract::CurveType::Ellipse:
                        std::cout << "  Segment " << i << ": Ellipse (a: " 
                                  << segment.GetEllipse().semi_major_axis << ", b: " 
                                  << segment.GetEllipse().semi_minor_axis << ")" << std::endl;
                        break;
                    case geometry_contract::CurveType::LineSequence:
                        std::cout << "  Segment " << i << ": LineSequence (" 
                                  << segment.GetPoints().size() << " points)" << std::endl;
                        break;
                }
            }
            
            return optimal_contour;
        }
        
        // Fallback to legacy discretization
        std::cout << "[WireBuilder] Falling back to legacy discretization for wire" << std::endl;
        return ConvertWireToContour(wire, tolerance);
    }

    // =============================================================================
    // WIRE EXTRACTION AND ASSEMBLY
    // =============================================================================

    std::vector<TopoDS_Wire> WireBuilder::FindExistingWires(const TopoDS_Shape& section) {
        std::vector<TopoDS_Wire> wires;
        for (TopExp_Explorer exp(section, TopAbs_WIRE); exp.More(); exp.Next()) {
            wires.push_back(TopoDS::Wire(exp.Current()));
        }
        return wires;
    }

    std::vector<TopoDS_Wire> WireBuilder::BuildWiresFromLooseEdges(const TopoDS_Shape& section) {
        std::vector<TopoDS_Edge> edges = FindLooseEdges(section);
        return AssembleWiresFromEdges(edges);
    }

    std::vector<TopoDS_Edge> WireBuilder::FindLooseEdges(const TopoDS_Shape& section) {
        std::vector<TopoDS_Edge> edges;
        for (TopExp_Explorer exp(section, TopAbs_EDGE, TopAbs_WIRE); exp.More(); exp.Next()) {
            edges.push_back(TopoDS::Edge(exp.Current()));
        }
        return edges;
    }

    std::vector<TopoDS_Wire> WireBuilder::AssembleWiresFromEdges(const std::vector<TopoDS_Edge>& edges) {
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

    std::optional<TopoDS_Wire> WireBuilder::BuildWireFromStartingEdge(const std::vector<TopoDS_Edge>& edges, std::vector<bool>& used, size_t start_idx) {
        BRepBuilderAPI_MakeWire builder = InitializeWireBuilder(edges[start_idx]);
        used[start_idx] = true;

        ExtendWireWithConnectedEdges(builder, edges, used);

        return builder.IsDone() ? std::make_optional(builder.Wire()) : std::nullopt;
    }

    // =============================================================================
    // WIRE BUILDING OPERATIONS
    // =============================================================================

    BRepBuilderAPI_MakeWire WireBuilder::InitializeWireBuilder(const TopoDS_Edge& starting_edge) {
        BRepBuilderAPI_MakeWire builder;
        builder.Add(starting_edge);
        return builder;
    }

    void WireBuilder::ExtendWireWithConnectedEdges(BRepBuilderAPI_MakeWire& builder, const std::vector<TopoDS_Edge>& edges, std::vector<bool>& used) {
        while (CanExtendWire(builder)) {
            size_t next_edge_idx = FindNextConnectedEdge(builder, edges, used);
            if (next_edge_idx == SIZE_MAX) break;
            
            if (TryAddEdgeToWire(builder, edges[next_edge_idx])) {
                used[next_edge_idx] = true;
            } else {
                break;
            }
        }
    }

    bool WireBuilder::CanExtendWire(const BRepBuilderAPI_MakeWire& builder) {
        return builder.IsDone();
    }

    size_t WireBuilder::FindNextConnectedEdge(const BRepBuilderAPI_MakeWire& builder, const std::vector<TopoDS_Edge>& edges, const std::vector<bool>& used) {
        if (!builder.IsDone()) {
            return SIZE_MAX;
        }
        
        // We need to cast away const to call Wire(), but this is safe since we're only reading
        TopoDS_Wire current_wire = const_cast<BRepBuilderAPI_MakeWire&>(builder).Wire();
        TopoDS_Vertex end_vertex = GetWireEndVertex(current_wire);
        
        for (size_t i = 0; i < edges.size(); ++i) {
            if (IsEdgeAvailableForConnection(i, used, edges[i], end_vertex)) {
                return i;
            }
        }
        
        return SIZE_MAX; // No connection found
    }

    bool WireBuilder::IsEdgeAvailableForConnection(size_t edge_idx, const std::vector<bool>& used, const TopoDS_Edge& edge, const TopoDS_Vertex& target_vertex) {
        return !used[edge_idx] && EdgeConnectsToVertex(edge, target_vertex);
    }

    bool WireBuilder::TryAddEdgeToWire(BRepBuilderAPI_MakeWire& builder, const TopoDS_Edge& edge) {
        builder.Add(edge);
        return builder.IsDone();
    }

    // =============================================================================
    // EDGE AND VERTEX OPERATIONS
    // =============================================================================

    std::vector<TopoDS_Edge> WireBuilder::GetOrderedEdges(const TopoDS_Wire& wire) {
        std::vector<TopoDS_Edge> edges;
        for (BRepTools_WireExplorer exp(wire); exp.More(); exp.Next()) {
            edges.push_back(exp.Current());
        }
        return edges;
    }

    TopoDS_Vertex WireBuilder::GetWireEndVertex(const TopoDS_Wire& wire) {
        TopoDS_Vertex start, end;
        TopExp::Vertices(wire, start, end);
        return end;
    }

    bool WireBuilder::EdgeConnectsToVertex(const TopoDS_Edge& edge, const TopoDS_Vertex& vertex) {
        TopoDS_Vertex start, end;
        TopExp::Vertices(edge, start, end);
        return vertex.IsSame(start) || vertex.IsSame(end);
    }

    // =============================================================================
    // DISCRETIZATION AND CONTOUR CONVERSION (Legacy methods)
    // =============================================================================

    std::optional<std::vector<geometry_contract::Point2D>> WireBuilder::DiscretizeEdge(const TopoDS_Edge& edge, double tolerance) {
        Standard_Real first, last;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);
        
        if (curve.IsNull()) return std::nullopt;
        
        return DiscretizeCurve(curve, first, last, tolerance);
    }

    std::vector<geometry_contract::Point2D> WireBuilder::DiscretizeCurve(const opencascade::handle<Geom_Curve>& curve, Standard_Real first, Standard_Real last, double tolerance) {
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

    void WireBuilder::AppendPointsToContour(geometry_contract::Contour& contour, const std::vector<geometry_contract::Point2D>& points) {
        // Convert to legacy format by creating a line sequence segment
        geometry_contract::Contour legacy_contour(points);
        
        // Copy the legacy points if this is the first segment
        if (contour.segments.empty() && contour.points().empty()) {
            contour = legacy_contour;
        } else {
            // Append points to existing legacy representation
            auto& existing_points = const_cast<std::vector<geometry_contract::Point2D>&>(contour.points());
            if (existing_points.empty()) {
                existing_points = points;
            } else {
                existing_points.insert(existing_points.end(), points.begin() + 1, points.end());
            }
        }
    }

} // namespace cad_slicer
