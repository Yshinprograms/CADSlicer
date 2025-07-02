#pragma once

#include <vector>
#include <optional>

// --- OCCT Forward Declarations ---
#include <Standard_Handle.hxx>
#include <Standard_Real.hxx>

class TopoDS_Shape;
class TopoDS_Edge;
class TopoDS_Wire;
class TopoDS_Vertex;
class Geom_Curve;
class BRepBuilderAPI_MakeWire;

#include "../SharedGeometry/GeometryContract.h"

namespace cad_slicer {

    class WireBuilder {
    public:
        // =============================================================================
        // PUBLIC INTERFACE - High-level wire operations
        // =============================================================================
        
        /// Extracts wires from a section shape and converts them to contours
        static std::vector<geometry_contract::Contour> ExtractContoursFromSection(const TopoDS_Shape& section, double tolerance);
        
        /// Enhanced method that preserves curve geometry using CurveAnalyzer
        static std::vector<geometry_contract::Contour> ExtractOptimalContoursFromSection(const TopoDS_Shape& section, double tolerance);
        
        /// Extracts all wires from a section shape
        static std::vector<TopoDS_Wire> ExtractWires(const TopoDS_Shape& section);
        
        /// Converts a single wire to a contour with specified tolerance
        static std::optional<geometry_contract::Contour> ConvertWireToContour(const TopoDS_Wire& wire, double tolerance);
        
        /// Enhanced conversion that preserves curve geometry
        static std::optional<geometry_contract::Contour> ConvertWireToOptimalContour(const TopoDS_Wire& wire, double tolerance);

    private:
        // =============================================================================
        // WIRE EXTRACTION AND ASSEMBLY
        // =============================================================================
        
        /// Finds existing wires in the section
        static std::vector<TopoDS_Wire> FindExistingWires(const TopoDS_Shape& section);
        
        /// Builds wires from loose edges when no wires exist
        static std::vector<TopoDS_Wire> BuildWiresFromLooseEdges(const TopoDS_Shape& section);
        
        /// Finds loose edges that are not part of any wire
        static std::vector<TopoDS_Edge> FindLooseEdges(const TopoDS_Shape& section);
        
        /// Assembles multiple wires from a collection of edges
        static std::vector<TopoDS_Wire> AssembleWiresFromEdges(const std::vector<TopoDS_Edge>& edges);
        
        /// Builds a single wire starting from a specific edge
        static std::optional<TopoDS_Wire> BuildWireFromStartingEdge(const std::vector<TopoDS_Edge>& edges, std::vector<bool>& used, size_t start_idx);

        // =============================================================================
        // WIRE BUILDING OPERATIONS
        // =============================================================================
        
        /// Initializes a wire builder with a starting edge
        static BRepBuilderAPI_MakeWire InitializeWireBuilder(const TopoDS_Edge& starting_edge);
        
        /// Extends a wire with connected edges
        static void ExtendWireWithConnectedEdges(BRepBuilderAPI_MakeWire& builder, const std::vector<TopoDS_Edge>& edges, std::vector<bool>& used);
        
        /// Checks if the wire can be extended further
        static bool CanExtendWire(const BRepBuilderAPI_MakeWire& builder);
        
        /// Finds the next edge that can be connected to the current wire
        static size_t FindNextConnectedEdge(const BRepBuilderAPI_MakeWire& builder, const std::vector<TopoDS_Edge>& edges, const std::vector<bool>& used);
        
        /// Checks if an edge is available for connection
        static bool IsEdgeAvailableForConnection(size_t edge_idx, const std::vector<bool>& used, const TopoDS_Edge& edge, const TopoDS_Vertex& target_vertex);
        
        /// Attempts to add an edge to the wire builder
        static bool TryAddEdgeToWire(BRepBuilderAPI_MakeWire& builder, const TopoDS_Edge& edge);

        // =============================================================================
        // EDGE AND VERTEX OPERATIONS
        // =============================================================================
        
        /// Gets ordered edges from a wire
        static std::vector<TopoDS_Edge> GetOrderedEdges(const TopoDS_Wire& wire);
        
        /// Gets the end vertex of a wire
        static TopoDS_Vertex GetWireEndVertex(const TopoDS_Wire& wire);
        
        /// Checks if an edge connects to a specific vertex
        static bool EdgeConnectsToVertex(const TopoDS_Edge& edge, const TopoDS_Vertex& vertex);

        // =============================================================================
        // DISCRETIZATION AND CONTOUR CONVERSION (Legacy methods)
        // =============================================================================
        
        /// Discretizes an edge into 2D points
        static std::optional<std::vector<geometry_contract::Point2D>> DiscretizeEdge(const TopoDS_Edge& edge, double tolerance);
        
        /// Discretizes a curve between two parameters
        static std::vector<geometry_contract::Point2D> DiscretizeCurve(const opencascade::handle<Geom_Curve>& curve, Standard_Real first, Standard_Real last, double tolerance);
        
        /// Appends points to a contour, avoiding duplication
        static void AppendPointsToContour(geometry_contract::Contour& contour, const std::vector<geometry_contract::Point2D>& points);
    };

} // namespace cad_slicer

