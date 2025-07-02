#pragma once

#include <string>
#include <vector>
#include "open_vector_format.pb.h" // OVF protobuf definitions
#include "ovf_file_writer.h"       // Include the writer header
#include "GeometryContract.h"      // Your custom geometry structs

namespace ovf = open_vector_format;

/**
 * @brief A class to convert and write simple geometry data to the OVF format.
 *
 * This class acts as an adapter, taking application-specific geometry data
 * and using the OvfFileWriter to produce a valid .ovf file.
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
    };

    /**
     * @brief (Private Helper) Converts a single contour to a VectorBlock.
     */
    ovf::VectorBlock convertContourToVectorBlock(const geometry_contract::Contour& contour, int marking_params_key) const;

    // Refactored helper methods for SLAP/KISS principles
    void LogConversionStart(size_t layer_count) const;
    void LogConversionSuccess(const ConversionStats& stats, const std::string& output_path) const;
    ConversionStats ProcessAllLayers(open_vector_format::reader_writer::OvfFileWriter& writer, const std::vector<geometry_contract::SlicedLayer>& layers);
    void ProcessSingleLayer(open_vector_format::reader_writer::OvfFileWriter& writer, const geometry_contract::SlicedLayer& layer, int layer_number, int& total_contours);
    void CreateAndAppendWorkPlane(open_vector_format::reader_writer::OvfFileWriter& writer, double z_height);
    void LogLayerProgress(int layer_number, double z_height, size_t contour_count) const;
    void ProcessLayerContours(open_vector_format::reader_writer::OvfFileWriter& writer, const geometry_contract::SlicedLayer& layer, int layer_number, int& total_contours);
    void LogContourDetails(int layer_number, size_t contour_idx, const geometry_contract::Contour& contour) const;
};