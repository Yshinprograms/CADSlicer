#define NOMINMAX

#include <Windows.h>
#include "OvfConverter.h"
#include "GeometryContract.h"
#include "ovf_file_writer.h" // We need the writer to do the work
#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>
#include <algorithm>

#include "ovf_file_reader.h"
#include "open_vector_format.pb.h"
#include "google/protobuf/stubs/common.h"

namespace ovf = open_vector_format;

OvfConverter::OvfConverter() {
    // Initialize with some default metadata.
    job_shell_.mutable_job_meta_data()->set_job_name("Default Job");
    job_shell_.mutable_job_meta_data()->set_author("OvfConverter");
    
    // Initialize job parameters to ensure proper serialization
    // The JobParameters message will be created but can remain empty initially
    job_shell_.mutable_job_parameters();
}

void OvfConverter::SetJobName(const std::string& name) {
    job_shell_.mutable_job_meta_data()->set_job_name(name);
}

void OvfConverter::SetAuthor(const std::string& author) {
    job_shell_.mutable_job_meta_data()->set_author(author);
}

void OvfConverter::AddMarkingParameters(int key, const ovf::MarkingParams& params) {
    // Insert the provided parameters into the job's map using the given key.
    (*job_shell_.mutable_marking_params_map())[key] = params;
}

bool OvfConverter::Write(const std::vector<geometry_contract::SlicedLayer>& layers, const std::string& output_path) {
    std::cout << "DEBUG: OvfConverter::Write() called" << std::endl;
    std::cout << "DEBUG: Input layers size: " << layers.size() << std::endl;
    std::cout << "DEBUG: Output path: " << output_path << std::endl;
    
    LogConversionStart(layers.size());
    
    try {
        std::cout << "DEBUG: Creating OvfFileWriter..." << std::endl;
        open_vector_format::reader_writer::OvfFileWriter writer;
        
        std::cout << "DEBUG: Starting partial write..." << std::endl;
        writer.StartWritePartial(job_shell_, output_path);
        std::cout << "DEBUG: Partial write started successfully" << std::endl;
        
        std::cout << "DEBUG: About to process all layers..." << std::endl;
        ConversionStats stats = ProcessAllLayers(writer, layers);
        std::cout << "DEBUG: Finished processing all layers. Stats: layers=" << stats.layer_count << ", contours=" << stats.total_contours << std::endl;
        
        std::cout << "DEBUG: Finishing write..." << std::endl;
        writer.FinishWrite();
        std::cout << "DEBUG: Write finished successfully" << std::endl;
        
        LogConversionSuccess(stats, output_path);
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "ERROR during conversion: " << e.what() << std::endl;
        std::cerr << "DEBUG: Exception caught in OvfConverter::Write()" << std::endl;
        return false;
    }
}

void OvfConverter::LogConversionStart(size_t layer_count) const {
    std::cout << "--- Starting OVF Conversion using OvfConverter class ---\n";
    std::cout << "Processing " << layer_count << " layers for OVF conversion..." << std::endl;
}

void OvfConverter::LogConversionSuccess(const ConversionStats& stats, const std::string& output_path) const {
    std::cout << "CONVERSION SUMMARY:" << std::endl;
    std::cout << "  Total layers processed: " << stats.layer_count << std::endl;
    std::cout << "  Total contours written: " << stats.total_contours << std::endl;
    std::cout << "SUCCESS: Wrote converted geometry to \"" << output_path << "\"" << std::endl;
}

OvfConverter::ConversionStats OvfConverter::ProcessAllLayers(open_vector_format::reader_writer::OvfFileWriter& writer, 
                                                            const std::vector<geometry_contract::SlicedLayer>& layers) {
    std::cout << "DEBUG: ProcessAllLayers() called with " << layers.size() << " layers" << std::endl;
    ConversionStats stats;
    
    for (const auto& layer : layers) {
        std::cout << "DEBUG: Processing layer with Z=" << layer.ZHeight << ", contours=" << layer.contours.size() << std::endl;
        ProcessSingleLayer(writer, layer, ++stats.layer_count, stats.total_contours);
        std::cout << "DEBUG: Completed processing layer " << stats.layer_count << ". Total contours so far: " << stats.total_contours << std::endl;
    }
    
    std::cout << "DEBUG: ProcessAllLayers() completed. Final stats: layers=" << stats.layer_count << ", contours=" << stats.total_contours << std::endl;
    return stats;
}

void OvfConverter::ProcessSingleLayer(open_vector_format::reader_writer::OvfFileWriter& writer, 
                                     const geometry_contract::SlicedLayer& layer, 
                                     int layer_number, 
                                     int& total_contours) {
    std::cout << "DEBUG: ProcessSingleLayer() called - layer " << layer_number << ", Z=" << layer.ZHeight << std::endl;
    std::cout << "DEBUG: Layer has " << layer.contours.size() << " contours" << std::endl;
    
    CreateAndAppendWorkPlane(writer, layer.ZHeight);
    LogLayerProgress(layer_number, layer.ZHeight, layer.contours.size());
    
    std::cout << "DEBUG: About to process layer contours..." << std::endl;
    ProcessLayerContours(writer, layer, layer_number, total_contours);
    std::cout << "DEBUG: Finished processing layer contours" << std::endl;
}

void OvfConverter::CreateAndAppendWorkPlane(open_vector_format::reader_writer::OvfFileWriter& writer, double z_height) {
    ovf::WorkPlane wp_shell;
    wp_shell.set_z_pos_in_mm(static_cast<float>(z_height));
    writer.AppendWorkPlane(wp_shell);
}

void OvfConverter::LogLayerProgress(int layer_number, double z_height, size_t contour_count) const {
    if (layer_number <= 3 || layer_number % 100 == 0) {
        std::cout << "  Layer " << layer_number << " at Z = " << z_height 
                  << "mm with " << contour_count << " contours" << std::endl;
    }
}

void OvfConverter::ProcessLayerContours(open_vector_format::reader_writer::OvfFileWriter& writer, 
                                       const geometry_contract::SlicedLayer& layer, 
                                       int layer_number, 
                                       int& total_contours) {
    std::cout << "DEBUG: ProcessLayerContours() called - layer " << layer_number << std::endl;
    std::cout << "DEBUG: Processing " << layer.contours.size() << " contours in this layer" << std::endl;
    
    for (size_t contour_idx = 0; contour_idx < layer.contours.size(); ++contour_idx) {
        const auto& contour = layer.contours[contour_idx];
        std::cout << "DEBUG: Processing contour " << contour_idx << " with " << contour.points.size() << " points" << std::endl;
        
        if (contour.points.empty()) {
            std::cout << "WARNING: Contour " << contour_idx << " is empty (no points)!" << std::endl;
            continue;
        }
        
        total_contours++;
        std::cout << "DEBUG: Converting contour to VectorBlock..." << std::endl;
        
        // Use the private helper to do the conversion.
        // We assume all contours will use marking parameters with key '1'.
        // A more advanced implementation could have this key per-contour.
        ovf::VectorBlock vb = convertContourToVectorBlock(contour, 1);
        std::cout << "DEBUG: VectorBlock created, appending to writer..." << std::endl;
        
        writer.AppendVectorBlock(vb);
        std::cout << "DEBUG: VectorBlock appended successfully" << std::endl;
        
        LogContourDetails(layer_number, contour_idx, contour);
    }
    
    std::cout << "DEBUG: ProcessLayerContours() completed. Total contours processed in this layer: " << layer.contours.size() << std::endl;
}

void OvfConverter::LogContourDetails(int layer_number, size_t contour_idx, 
                                    const geometry_contract::Contour& contour) const {
    if (layer_number <= 3 && contour_idx == 0) {
        std::cout << "    Contour " << (contour_idx + 1) << " has " << contour.points.size() << " points:" << std::endl;
        
        const size_t max_points_to_show = 5;
        for (size_t i = 0; i < std::min(contour.points.size(), max_points_to_show); ++i) {
            std::cout << "      Point " << (i + 1) << ": (" 
                      << contour.points[i].x << ", " << contour.points[i].y << ")" << std::endl;
        }
        
        if (contour.points.size() > max_points_to_show) {
            std::cout << "      ... (" << (contour.points.size() - max_points_to_show) << " more points)" << std::endl;
        }
    }
}

// --- Private Helper Implementation ---
ovf::VectorBlock OvfConverter::convertContourToVectorBlock(const geometry_contract::Contour& contour, int marking_params_key) const {
    std::cout << "DEBUG: convertContourToVectorBlock() called with " << contour.points.size() << " points and marking_params_key=" << marking_params_key << std::endl;
    
    ovf::VectorBlock vb;
    vb.set_marking_params_key(marking_params_key);
    std::cout << "DEBUG: VectorBlock created and marking_params_key set to " << marking_params_key << std::endl;

    ovf::VectorBlock::LineSequence* line_seq = vb.mutable_line_sequence();
    std::cout << "DEBUG: LineSequence obtained from VectorBlock" << std::endl;
    
    int point_count = 0;
    for (const auto& point : contour.points) {
        line_seq->add_points(static_cast<float>(point.x));
        line_seq->add_points(static_cast<float>(point.y));
        point_count++;
        
        if (point_count <= 3) {  // Log first few points for verification
            std::cout << "DEBUG: Added point " << point_count << ": (" << point.x << ", " << point.y << ")" << std::endl;
        }
    }

    // Ensure the contour is closed.
    if (!contour.points.empty()) {
        const auto& first_point = contour.points.front();
        line_seq->add_points(static_cast<float>(first_point.x));
        line_seq->add_points(static_cast<float>(first_point.y));
        std::cout << "DEBUG: Added closing point: (" << first_point.x << ", " << first_point.y << ")" << std::endl;
    }

    std::cout << "DEBUG: VectorBlock conversion completed. Total points in LineSequence: " << line_seq->points_size()/2 << std::endl;
    return vb;
}