#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <filesystem>

// Include your library classes
#include "StepSlicer.h"
#include "OvfConverter.h"
#include "GeometryContract.h"

// Include the generated Protobuf header
#include "open_vector_format.pb.h"

// It's good practice to alias the namespace
namespace ovf = open_vector_format;

int main() {
    // 1. SETUP: Define constants for clarity and easy modification
    // Note: In C++, backslashes in string literals must be escaped.
    const std::string step_file_path = "C:\\Users\\pin20\\Downloads\\SIMTech_Internship\\DesignSamples\\STEPSamples\\lilcomplex.step";
    const std::string ovf_output_path = "lilcomplex.ovf";
    const double layer_height = 0.1;          // Slice every 0.1 mm
    const double deflection_tolerance = 0.01; // High quality discretization

    std::cout << "--- OVF Conversion Process Started ---" << std::endl;

    // 1.5. FILE EXISTENCE CHECK: Verify the STEP file exists before proceeding
    std::cout << "Checking if STEP file exists: " << step_file_path << std::endl;
    if (!std::filesystem::exists(step_file_path)) {
        std::cerr << "ERROR: STEP file does not exist at path: " << step_file_path << std::endl;
        std::cerr << "Please verify the file path is correct." << std::endl;
        return 1;
    }
    std::cout << "STEP file found successfully." << std::endl;

    // 2. SLICING: Create the slicer and execute the slicing operation
    std::cout << "Initializing slicer for file: " << step_file_path << std::endl;
    
    try {
        geometry::StepSlicer slicer(step_file_path);

        std::cout << "Slicing model with layer height " << layer_height
            << "mm and tolerance " << deflection_tolerance << "mm..." << std::endl;

        // The Slice method does all the heavy lifting with OCCT
        std::vector<geometry_contract::SlicedLayer> sliced_layers = slicer.Slice(layer_height, deflection_tolerance);

        // 3. ERROR CHECK #1: Verify that slicing was successful
        if (sliced_layers.empty()) {
            std::cerr << "Error: Slicing failed. The model might be invalid, empty, or the file path is incorrect." << std::endl;
            std::cerr << "Common causes:" << std::endl;
            std::cerr << "  - Invalid or corrupted STEP file" << std::endl;
            std::cerr << "  - Empty geometry in STEP file" << std::endl;
            std::cerr << "  - STEP file contains only 2D geometry (no volume to slice)" << std::endl;
            std::cerr << "  - Layer height too large relative to model size" << std::endl;
            return 1; // Exit with an error code
        }

        std::cout << "Slicing successful. Found " << sliced_layers.size() << " layers." << std::endl;
        
        // Log some details about the sliced layers
        for (size_t i = 0; i < std::min(sliced_layers.size(), size_t{3}); ++i) {
            const auto& layer = sliced_layers[i];
            std::cout << "  Layer " << (i + 1) << ": Z=" << layer.ZHeight 
                      << "mm, contours=" << layer.contours.size() << std::endl;
        }
        if (sliced_layers.size() > 3) {
            std::cout << "  ... and " << (sliced_layers.size() - 3) << " more layers" << std::endl;
        }

        // 4. CONVERSION: Create the OVF converter
        std::cout << "\nInitializing OVF converter..." << std::endl;
        OvfConverter converter;

        // 5. CONFIGURE: Set up the OVF job metadata and parameters
        std::cout << "Configuring OVF job metadata..." << std::endl;
        converter.SetJobName("Sliced Cube Test");
        converter.SetAuthor("Pin20");

        // Example: Create a set of marking parameters for the contours.
        // We'll give this parameter set an ID of '1'. All contours will use it.
        ovf::MarkingParams contour_params;
        contour_params.set_laser_power_in_w(200.0f);
        contour_params.set_laser_speed_in_mm_per_s(1500.0f);
        contour_params.set_jump_speed_in_mm_s(7000.0f);
        contour_params.set_name("Contour Marking");

        converter.AddMarkingParameters(1, contour_params); // Add to the job's parameter map with key=1
        std::cout << "Added 'Contour Marking' parameter set with key 1." << std::endl;

        // 6. WRITE: Perform the conversion and write the final file
        std::cout << "Writing to OVF file: " << ovf_output_path << std::endl;

        // The Write method will iterate through your sliced_layers,
        // convert them to protobuf messages, and serialize them to a file.
        bool success = converter.Write(sliced_layers, ovf_output_path);

        // 7. FINAL REPORT: Check the result and inform the user
        if (success) {
            std::cout << "\n--- OVF Conversion Process Successful ---" << std::endl;
            std::cout << "Output file saved to: " << ovf_output_path << std::endl;
            
            // Verify the generated file
            std::cout << "\n--- Verifying Generated OVF File ---" << std::endl;
            std::ifstream file(ovf_output_path, std::ios::binary);
            if (file) {
                file.seekg(0, std::ios::end);
                auto file_size = file.tellg();
                std::cout << "Generated OVF file size: " << file_size << " bytes" << std::endl;
                
                if (file_size > 100) { // Should be much larger for a valid OVF file with data
                    std::cout << "SUCCESS: OVF file appears to contain data!" << std::endl;
                } else {
                    std::cout << "WARNING: OVF file seems unusually small." << std::endl;
                }
            }
        }
        else {
            std::cerr << "\n--- OVF Conversion Process Failed ---" << std::endl;
            std::cerr << "Error: Failed to write OVF file." << std::endl;
            return 1; // Exit with an error code
        }
    }
    catch (const std::exception& e) {
        std::cerr << "\n--- Exception Occurred ---" << std::endl;
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }

    return 0; // Success
}