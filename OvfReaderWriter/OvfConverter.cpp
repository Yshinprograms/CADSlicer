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
#include <cmath>
#include <iomanip>

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
        std::cout << "DEBUG: Finished processing all layers. Stats: layers=" << stats.layer_count 
                  << ", contours=" << stats.total_contours 
                  << ", arcs=" << stats.total_arcs 
                  << ", ellipses=" << stats.total_ellipses 
                  << ", line_sequences=" << stats.total_line_sequences << std::endl;
        
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
    std::cout << "--- Starting Enhanced OVF Conversion with Native Curves ---\n";
    std::cout << "Processing " << layer_count << " layers for OVF conversion..." << std::endl;
}

void OvfConverter::LogConversionSuccess(const ConversionStats& stats, const std::string& output_path) const {
    std::cout << "\n=== ENHANCED CONVERSION SUMMARY ===" << std::endl;
    std::cout << "  Total layers processed: " << stats.layer_count << std::endl;
    std::cout << "  Total contours processed: " << stats.total_contours << std::endl;
    std::cout << "  Native curves extracted:" << std::endl;
    std::cout << "    - Arcs: " << stats.total_arcs << std::endl;
    std::cout << "    - Ellipses: " << stats.total_ellipses << std::endl;
    std::cout << "    - Line sequences: " << stats.total_line_sequences << std::endl;
    
    int total_native_curves = stats.total_arcs + stats.total_ellipses;
    if (total_native_curves > 0) {
        double native_curve_percentage = (100.0 * total_native_curves) / 
            (total_native_curves + stats.total_line_sequences);
        std::cout << "  Native curve optimization: " << std::fixed << std::setprecision(1) 
                  << native_curve_percentage << "% of curves preserved as native geometry" << std::endl;
    }
    
    std::cout << "SUCCESS: Wrote optimized geometry to \"" << output_path << "\"" << std::endl;
}

OvfConverter::ConversionStats OvfConverter::ProcessAllLayers(open_vector_format::reader_writer::OvfFileWriter& writer, 
                                                            const std::vector<geometry_contract::SlicedLayer>& layers) {
    std::cout << "DEBUG: ProcessAllLayers() called with " << layers.size() << " layers" << std::endl;
    ConversionStats stats;
    
    for (const auto& layer : layers) {
        std::cout << "DEBUG: Processing layer with Z=" << layer.ZHeight << ", contours=" << layer.contours.size() << std::endl;
        ProcessSingleLayer(writer, layer, ++stats.layer_count, stats);
        std::cout << "DEBUG: Completed processing layer " << stats.layer_count 
                  << ". Total contours so far: " << stats.total_contours << std::endl;
    }
    
    std::cout << "DEBUG: ProcessAllLayers() completed. Final stats: layers=" << stats.layer_count 
              << ", contours=" << stats.total_contours << std::endl;
    return stats;
}

void OvfConverter::ProcessSingleLayer(open_vector_format::reader_writer::OvfFileWriter& writer, 
                                     const geometry_contract::SlicedLayer& layer, 
                                     int layer_number, 
                                     ConversionStats& stats) {
    std::cout << "DEBUG: ProcessSingleLayer() called - layer " << layer_number << ", Z=" << layer.ZHeight << std::endl;
    std::cout << "DEBUG: Layer has " << layer.contours.size() << " contours" << std::endl;
    
    CreateAndAppendWorkPlane(writer, layer.ZHeight);
    LogLayerProgress(layer_number, layer.ZHeight, layer.contours.size());
    
    std::cout << "DEBUG: About to process layer contours..." << std::endl;
    ProcessLayerContours(writer, layer, layer_number, stats);
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
                                       ConversionStats& stats) {
    std::cout << "DEBUG: ProcessLayerContours() called - layer " << layer_number << std::endl;
    std::cout << "DEBUG: Processing " << layer.contours.size() << " contours in this layer" << std::endl;
    
    for (size_t contour_idx = 0; contour_idx < layer.contours.size(); ++contour_idx) {
        const auto& contour = layer.contours[contour_idx];
        std::cout << "DEBUG: Processing contour " << contour_idx;
        
        // Check if contour uses enhanced geometry or legacy points
        if (!contour.segments.empty()) {
            std::cout << " with " << contour.segments.size() << " curve segments" << std::endl;
            LogCurveConversionDetails(contour);
            
            // Convert each curve segment to appropriate VectorBlocks
            auto vector_blocks = ConvertContourToVectorBlocks(contour, 1);
            for (const auto& vb : vector_blocks) {
                writer.AppendVectorBlock(vb);
            }
            
            // Update statistics
            for (const auto& segment : contour.segments) {
                UpdateConversionStats(stats, segment);
            }
            
            stats.total_contours++;
        }
        else if (!contour.points().empty()) {
            std::cout << " with " << contour.points().size() << " legacy points" << std::endl;
            
            // Use legacy conversion for backwards compatibility
            ovf::VectorBlock vb = convertContourToVectorBlock(contour, 1);
            writer.AppendVectorBlock(vb);
            
            stats.total_contours++;
            stats.total_line_sequences++;
        }
        else {
            std::cout << " - empty contour, skipping" << std::endl;
            continue;
        }
        
        if (layer_number <= 3) {
            LogContourDetails(layer_number, contour_idx, contour);
        }
    }
    
    std::cout << "DEBUG: ProcessLayerContours() completed. Total contours processed in this layer: " 
              << layer.contours.size() << std::endl;
}

// =============================================================================
// CONTOUR CONVERSION METHODS IMPLEMENTATION
// =============================================================================

std::vector<ovf::VectorBlock> OvfConverter::ConvertContourToVectorBlocks(
    const geometry_contract::Contour& contour, int marking_params_key) const {
    
    std::vector<ovf::VectorBlock> vector_blocks;
    
    for (const auto& segment : contour.segments) {
        ovf::VectorBlock vb = ConvertCurveSegmentToVectorBlock(segment, marking_params_key);
        vector_blocks.push_back(vb);
    }
    
    return vector_blocks;
}

ovf::VectorBlock OvfConverter::ConvertCurveSegmentToVectorBlock(
    const geometry_contract::CurveSegment& segment, int marking_params_key) const {
    
    switch (segment.GetType()) {
        case geometry_contract::CurveType::Arc:
            return ConvertArcToVectorBlock(segment.GetArc(), marking_params_key);
            
        case geometry_contract::CurveType::Ellipse:
            return ConvertEllipseToVectorBlock(segment.GetEllipse(), marking_params_key);
            
        case geometry_contract::CurveType::LineSequence:
            return ConvertLineSequenceToVectorBlock(segment.GetPoints(), marking_params_key);
            
        default:
            // Fallback to line sequence
            auto points = segment.ToPoints(0.01);
            return ConvertLineSequenceToVectorBlock(points, marking_params_key);
    }
}

ovf::VectorBlock OvfConverter::ConvertArcToVectorBlock(
    const geometry_contract::Arc& arc, int marking_params_key) const {
    
    ovf::VectorBlock vb;
    vb.set_marking_params_key(marking_params_key);
    
    // Set up OVF Arc format
    ovf::VectorBlock::Arcs* arcs = vb.mutable__arcs();
    
    // Convert angles from radians to degrees for OVF format
    double angle_degrees = geometry_contract::curve_utils::RadiansToDegrees(arc.sweep_angle);
    arcs->set_angle(angle_degrees);
    
    // Calculate start direction vector
    float start_dx = static_cast<float>(arc.radius * std::cos(arc.start_angle));
    float start_dy = static_cast<float>(arc.radius * std::sin(arc.start_angle));
    arcs->set_start_dx(start_dx);
    arcs->set_start_dy(start_dy);
    
    // Add center point
    arcs->add_centers(static_cast<float>(arc.center.x));
    arcs->add_centers(static_cast<float>(arc.center.y));
    
    std::cout << "  [OvfConverter] Converted arc: center(" << arc.center.x << "," << arc.center.y 
              << ") radius=" << arc.radius << " angle=" << angle_degrees << "°" << std::endl;
    
    return vb;
}

ovf::VectorBlock OvfConverter::ConvertEllipseToVectorBlock(
    const geometry_contract::Ellipse& ellipse, int marking_params_key) const {
    
    ovf::VectorBlock vb;
    vb.set_marking_params_key(marking_params_key);
    
    // Set up OVF Ellipse format
    ovf::VectorBlock::Ellipses* ellipses = vb.mutable_ellipses();
    
    // Set semi-axes
    ellipses->set_a(static_cast<float>(ellipse.semi_major_axis));
    ellipses->set_b(static_cast<float>(ellipse.semi_minor_axis));
    
    // Set phase angle (start angle)
    ellipses->set_phi0(ellipse.start_angle);
    
    // Create embedded arc data for the ellipse
    ovf::VectorBlock::Arcs* embedded_arcs = ellipses->mutable_ellipses_arcs();
    
    // Convert sweep angle to degrees
    double angle_degrees = geometry_contract::curve_utils::RadiansToDegrees(ellipse.sweep_angle);
    embedded_arcs->set_angle(angle_degrees);
    
    // Calculate start direction (considering rotation)
    double cos_rot = std::cos(ellipse.rotation_angle);
    double sin_rot = std::sin(ellipse.rotation_angle);
    double start_x_local = ellipse.semi_major_axis * std::cos(ellipse.start_angle);
    double start_y_local = ellipse.semi_minor_axis * std::sin(ellipse.start_angle);
    
    float start_dx = static_cast<float>(start_x_local * cos_rot - start_y_local * sin_rot);
    float start_dy = static_cast<float>(start_x_local * sin_rot + start_y_local * cos_rot);
    
    embedded_arcs->set_start_dx(start_dx);
    embedded_arcs->set_start_dy(start_dy);
    
    // Add center point
    embedded_arcs->add_centers(static_cast<float>(ellipse.center.x));
    embedded_arcs->add_centers(static_cast<float>(ellipse.center.y));
    
    std::cout << "  [OvfConverter] Converted ellipse: center(" << ellipse.center.x << "," << ellipse.center.y 
              << ") a=" << ellipse.semi_major_axis << " b=" << ellipse.semi_minor_axis 
              << " rotation=" << geometry_contract::curve_utils::RadiansToDegrees(ellipse.rotation_angle) << "°" << std::endl;
    
    return vb;
}

ovf::VectorBlock OvfConverter::ConvertLineSequenceToVectorBlock(
    const std::vector<geometry_contract::Point2D>& points, int marking_params_key) const {
    
    ovf::VectorBlock vb;
    vb.set_marking_params_key(marking_params_key);
    
    ovf::VectorBlock::LineSequence* line_seq = vb.mutable_line_sequence();
    
    for (const auto& point : points) {
        line_seq->add_points(static_cast<float>(point.x));
        line_seq->add_points(static_cast<float>(point.y));
    }
    
    // Ensure the contour is closed if needed
    if (!points.empty()) {
        const auto& first_point = points.front();
        const auto& last_point = points.back();
        
        // Check if contour needs closing (tolerance-based check)
        double dx = last_point.x - first_point.x;
        double dy = last_point.y - first_point.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        if (distance > 1e-6) { // Only close if not already closed
            line_seq->add_points(static_cast<float>(first_point.x));
            line_seq->add_points(static_cast<float>(first_point.y));
        }
    }
    
    return vb;
}

// =============================================================================
// HELPER METHODS IMPLEMENTATION
// =============================================================================

void OvfConverter::UpdateConversionStats(ConversionStats& stats, const geometry_contract::CurveSegment& segment) const {
    switch (segment.GetType()) {
        case geometry_contract::CurveType::Arc:
            stats.total_arcs++;
            break;
        case geometry_contract::CurveType::Ellipse:
            stats.total_ellipses++;
            break;
        case geometry_contract::CurveType::LineSequence:
            stats.total_line_sequences++;
            break;
    }
}

void OvfConverter::LogCurveConversionDetails(const geometry_contract::Contour& contour) const {
    if (contour.segments.size() <= 5) { // Only log for simple contours to avoid spam
        for (size_t i = 0; i < contour.segments.size(); ++i) {
            const auto& segment = contour.segments[i];
            switch (segment.GetType()) {
                case geometry_contract::CurveType::Arc:
                    std::cout << "    Segment " << i << ": Arc (radius=" << segment.GetArc().radius << ")" << std::endl;
                    break;
                case geometry_contract::CurveType::Ellipse:
                    std::cout << "    Segment " << i << ": Ellipse (a=" << segment.GetEllipse().semi_major_axis 
                              << ", b=" << segment.GetEllipse().semi_minor_axis << ")" << std::endl;
                    break;
                case geometry_contract::CurveType::LineSequence:
                    std::cout << "    Segment " << i << ": LineSequence (" << segment.GetPoints().size() << " points)" << std::endl;
                    break;
            }
        }
    } else {
        std::cout << "    Complex contour with " << contour.segments.size() << " curve segments" << std::endl;
    }
}

void OvfConverter::LogContourDetails(int layer_number, size_t contour_idx, 
                                    const geometry_contract::Contour& contour) const {
    if (layer_number <= 3 && contour_idx == 0) {
        if (!contour.segments.empty()) {
            std::cout << "    Enhanced contour " << (contour_idx + 1) << " has " << contour.segments.size() << " curve segments" << std::endl;
        } else if (!contour.points().empty()) {
            std::cout << "    Legacy contour " << (contour_idx + 1) << " has " << contour.points().size() << " points:" << std::endl;
            
            const size_t max_points_to_show = 5;
            for (size_t i = 0; i < (std::min)(contour.points().size(), max_points_to_show); ++i) {
                std::cout << "      Point " << (i + 1) << ": (" 
                          << contour.points()[i].x << ", " << contour.points()[i].y << ")" << std::endl;
            }
            
            if (contour.points().size() > max_points_to_show) {
                std::cout << "      ... (" << (contour.points().size() - max_points_to_show) << " more points)" << std::endl;
            }
        }
    }
}

// --- Private Helper Implementation (Legacy Support) ---
ovf::VectorBlock OvfConverter::convertContourToVectorBlock(const geometry_contract::Contour& contour, int marking_params_key) const {
    std::cout << "DEBUG: convertContourToVectorBlock() called with " << contour.points().size() << " points and marking_params_key=" << marking_params_key << std::endl;
    
    return ConvertLineSequenceToVectorBlock(contour.points(), marking_params_key);
}