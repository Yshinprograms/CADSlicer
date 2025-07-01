#include "OvfConverter.h"
#include "GeometryContract.h"
#include "ovf_file_writer.h" // We need the writer to do the work
#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>

#include "ovf_file_reader.h"
#include "open_vector_format.pb.h"
#include "google/protobuf/stubs/common.h"

namespace ovf = open_vector_format;
namespace ovf_rw = open_vector_format::reader_writer;

OvfConverter::OvfConverter() {
    // Initialize with some default metadata.
    job_shell_.mutable_job_meta_data()->set_job_name("Default Job");
    job_shell_.mutable_job_meta_data()->set_author("OvfConverter");
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
    std::cout << "--- Starting OVF Conversion using OvfConverter class ---\n";

    try {
        ovf_rw::OvfFileWriter writer;
        writer.StartWritePartial(job_shell_, output_path);

        for (const auto& layer : layers) {
            ovf::WorkPlane wp_shell;
            wp_shell.set_z_pos_in_mm(static_cast<float>(layer.ZHeight));
            writer.AppendWorkPlane(wp_shell);

            for (const auto& contour : layer.contours) {
                // Use the private helper to do the conversion.
                // We assume all contours will use marking parameters with key '1'.
                // A more advanced implementation could have this key per-contour.
                ovf::VectorBlock vb = convertContourToVectorBlock(contour, 1);
                writer.AppendVectorBlock(vb);
            }
        }

        writer.FinishWrite();
        std::cout << "SUCCESS: Wrote converted geometry to \"" << output_path << "\"\n";
        return true;

    }
    catch (const std::exception& e) {
        std::cerr << "ERROR during conversion: " << e.what() << std::endl;
        return false;
    }
}

// --- Private Helper Implementation ---
ovf::VectorBlock OvfConverter::convertContourToVectorBlock(const geometry_contract::Contour& contour, int marking_params_key) const {
    ovf::VectorBlock vb;
    vb.set_marking_params_key(marking_params_key);

    ovf::VectorBlock::LineSequence* line_seq = vb.mutable_line_sequence();
    for (const auto& point : contour.points) {
        line_seq->add_points(static_cast<float>(point.x));
        line_seq->add_points(static_cast<float>(point.y));
    }

    // Ensure the contour is closed.
    if (!contour.points.empty()) {
        const auto& first_point = contour.points.front();
        line_seq->add_points(static_cast<float>(first_point.x));
        line_seq->add_points(static_cast<float>(first_point.y));
    }

    return vb;
}