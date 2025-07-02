#pragma once

#include <string>
#include <vector>
#include "open_vector_format.pb.h" // OVF protobuf definitions
#include "ovf_file_writer.h"       // Include the writer header
#include "../SharedGeometry/GeometryContract.h"      // Your custom geometry structs

namespace ovf = open_vector_format;

/**
 * @brief A class to convert and write simple geometry data to the OVF format.
 *
 * This class acts as an adapter, taking application-specific geometry data
 * and using the OvfFileWriter to produce a valid .ovf file.
 * Enhanced to support native curve types (arcs and ellipses) from the post-processing module.
 */
class OvfConverter {
public:
    /**
     * @brief Constructs an OvfConverter and initializes a default Job shell.
     */
    OvfConverter();

    /**
     * @brief Sets the job name in the OVF metadata.
     * @param name The name of the job.
     */
    void SetJobName(const std::string& name);

    /**
     * @brief Sets the author in the OVF metadata.
     * @param author The name of the author.
     */
    void SetAuthor(const std::string& author);

    /**
     * @brief Adds a set of marking parameters to the job.
     * @param key The integer key to identify these parameters.
     * @param params The MarkingParams object to add.
     */
    void AddMarkingParameters(int key, const ovf::MarkingParams& params);

    /**
     * @brief Converts the provided layers and writes them to an OVF file.
     * @param layers A vector of SlicedLayer objects.
     * @param output_path The file path for the output .ovf file.
     * @return True on success, false on failure.
     */
    bool Write(const std::vector<geometry_contract::SlicedLayer>& layers, const std::string& output_path);

private:
    // This private member holds the Job's metadata, which is configured
    // by the public setter methods before being passed to the writer.
    ovf::Job job_shell_;

    struct ConversionStats {
        int layer_count = 0;
        int total_contours = 0;
        int total_arcs = 0;
        int total_ellipses = 0;
        int total_line_sequences = 0;
    };

    // =============================================================================
    // CONTOUR CONVERSION METHODS
    // =============================================================================

    /**
     * @brief Converts a single contour to VectorBlocks, handling different curve types.
     */
    std::vector<ovf::VectorBlock> ConvertContourToVectorBlocks(
        const geometry_contract::Contour& contour, int marking_params_key) const;

    /**
     * @brief (Legacy) Converts a single contour to a VectorBlock using points only.
     */
    ovf::VectorBlock convertContourToVectorBlock(const geometry_contract::Contour& contour, int marking_params_key) const;

    // =============================================================================
    // CURVE SEGMENT CONVERSION METHODS
    // =============================================================================

    /**
     * @brief Converts a curve segment to the appropriate OVF VectorBlock type.
     */
    ovf::VectorBlock ConvertCurveSegmentToVectorBlock(
        const geometry_contract::CurveSegment& segment, int marking_params_key) const;

    /**
     * @brief Converts an Arc to OVF Arcs format.
     */
    ovf::VectorBlock ConvertArcToVectorBlock(
        const geometry_contract::Arc& arc, int marking_params_key) const;

    /**
     * @brief Converts an Ellipse to OVF Ellipses format.
     */
    ovf::VectorBlock ConvertEllipseToVectorBlock(
        const geometry_contract::Ellipse& ellipse, int marking_params_key) const;

    /**
     * @brief Converts a line sequence to OVF LineSequence format.
     */
    ovf::VectorBlock ConvertLineSequenceToVectorBlock(
        const std::vector<geometry_contract::Point2D>& points, int marking_params_key) const;

    // =============================================================================
    // HELPER METHODS
    // =============================================================================

    /**
     * @brief Updates conversion statistics based on curve segment type.
     */
    void UpdateConversionStats(ConversionStats& stats, const geometry_contract::CurveSegment& segment) const;

    /**
     * @brief Logs detailed information about curve conversion.
     */
    void LogCurveConversionDetails(const geometry_contract::Contour& contour) const;

    // Refactored helper methods for SLAP/KISS principles
    void LogConversionStart(size_t layer_count) const;
    void LogConversionSuccess(const ConversionStats& stats, const std::string& output_path) const;
    ConversionStats ProcessAllLayers(open_vector_format::reader_writer::OvfFileWriter& writer, const std::vector<geometry_contract::SlicedLayer>& layers);
    void ProcessSingleLayer(open_vector_format::reader_writer::OvfFileWriter& writer, const geometry_contract::SlicedLayer& layer, int layer_number, ConversionStats& stats);
    void CreateAndAppendWorkPlane(open_vector_format::reader_writer::OvfFileWriter& writer, double z_height);
    void LogLayerProgress(int layer_number, double z_height, size_t contour_count) const;
    void ProcessLayerContours(open_vector_format::reader_writer::OvfFileWriter& writer, const geometry_contract::SlicedLayer& layer, int layer_number, ConversionStats& stats);
    void LogContourDetails(int layer_number, size_t contour_idx, const geometry_contract::Contour& contour) const;
};