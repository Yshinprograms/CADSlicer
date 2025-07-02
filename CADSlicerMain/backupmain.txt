#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <array>
#include <iomanip>
#include <exception>
#include <stdexcept>

#include "OvfConverter.h"
#include "GeometryContract.h"
#include "open_vector_format.pb.h"

// Helper function to create a square contour
geometry_contract::Contour createSquareContour(double x_offset, double y_offset, double size) {
    std::cout << "DEBUG: Creating square contour at (" << x_offset << ", " << y_offset << ") with size " << size << std::endl;
    
    geometry_contract::Contour square;
    square.points.push_back({x_offset, y_offset});
    square.points.push_back({x_offset + size, y_offset});
    square.points.push_back({x_offset + size, y_offset + size});
    square.points.push_back({x_offset, y_offset + size});
    square.points.push_back({x_offset, y_offset}); // Close the square
    
    std::cout << "DEBUG: Square contour created with " << square.points.size() << " points:" << std::endl;
    for (size_t i = 0; i < square.points.size(); ++i) {
        std::cout << "  Point " << i << ": (" << square.points[i].x << ", " << square.points[i].y << ")" << std::endl;
    }
    
    return square;
}

int main() {
    const std::string ovf_output_path = "squares.ovf";

    std::cout << "--- OVF Generation Process Started ---" << std::endl;
    std::cout << "DEBUG: Target output file: " << ovf_output_path << std::endl;

    // Create two layers of squares
    std::vector<geometry_contract::SlicedLayer> sliced_layers;

    std::cout << "DEBUG: Creating Layer 1..." << std::endl;
    // Layer 1
    geometry_contract::SlicedLayer layer1;
    layer1.ZHeight = 0.0;
    layer1.contours.push_back(createSquareContour(0.0, 0.0, 10.0)); // 10x10 square at (0,0)
    sliced_layers.push_back(layer1);
    std::cout << "DEBUG: Layer 1 created - Z=" << layer1.ZHeight << ", contours=" << layer1.contours.size() << std::endl;

    std::cout << "DEBUG: Creating Layer 2..." << std::endl;
    // Layer 2
    geometry_contract::SlicedLayer layer2;
    layer2.ZHeight = 1.0; // Slightly above layer 1
    layer2.contours.push_back(createSquareContour(5.0, 5.0, 10.0)); // 10x10 square at (5,5)
    sliced_layers.push_back(layer2);
    std::cout << "DEBUG: Layer 2 created - Z=" << layer2.ZHeight << ", contours=" << layer2.contours.size() << std::endl;

    std::cout << "DEBUG: Total layers created: " << sliced_layers.size() << std::endl;
    
    // Verify the data before passing to converter
    for (size_t layer_idx = 0; layer_idx < sliced_layers.size(); ++layer_idx) {
        const auto& layer = sliced_layers[layer_idx];
        std::cout << "DEBUG: Layer " << layer_idx << " verification:" << std::endl;
        std::cout << "  Z-Height: " << layer.ZHeight << std::endl;
        std::cout << "  Number of contours: " << layer.contours.size() << std::endl;
        
        for (size_t contour_idx = 0; contour_idx < layer.contours.size(); ++contour_idx) {
            const auto& contour = layer.contours[contour_idx];
            std::cout << "  Contour " << contour_idx << " has " << contour.points.size() << " points" << std::endl;
            
            if (!contour.points.empty()) {
                std::cout << "    First point: (" << contour.points[0].x << ", " << contour.points[0].y << ")" << std::endl;
                std::cout << "    Last point: (" << contour.points.back().x << ", " << contour.points.back().y << ")" << std::endl;
            }
        }
    }

    // Initialize OVF converter
    std::cout << "Initializing OVF converter..." << std::endl;
    OvfConverter converter;
    std::cout << "DEBUG: OVF converter initialized successfully" << std::endl;

    // Configure OVF job metadata
    std::cout << "Configuring OVF job metadata..." << std::endl;
    converter.SetJobName("Two Squares Test");
    converter.SetAuthor("Gemini");
    std::cout << "DEBUG: OVF job metadata configured" << std::endl;

    // Add default marking parameters
    std::cout << "DEBUG: Creating marking parameters..." << std::endl;
    ovf::MarkingParams contour_params;
    contour_params.set_laser_power_in_w(100.0f);
    contour_params.set_laser_speed_in_mm_per_s(500.0f);
    contour_params.set_jump_speed_in_mm_s(1000.0f);
    contour_params.set_name("Default Contour Params");
    
    std::cout << "DEBUG: Marking parameters created:" << std::endl;
    std::cout << "  Laser power: " << contour_params.laser_power_in_w() << "W" << std::endl;
    std::cout << "  Laser speed: " << contour_params.laser_speed_in_mm_per_s() << "mm/s" << std::endl;
    std::cout << "  Jump speed: " << contour_params.jump_speed_in_mm_s() << "mm/s" << std::endl;
    std::cout << "  Name: " << contour_params.name() << std::endl;
    
    converter.AddMarkingParameters(1, contour_params);
    std::cout << "DEBUG: Marking parameters added to converter with key=1" << std::endl;

    // Write to OVF file
    std::cout << "Writing to OVF file: " << ovf_output_path << std::endl;
    std::cout << "DEBUG: About to call converter.Write() with " << sliced_layers.size() << " layers" << std::endl;
    
    bool success = converter.Write(sliced_layers, ovf_output_path);
    
    std::cout << "DEBUG: converter.Write() returned: " << (success ? "true" : "false") << std::endl;

    // Report result
    if (success) {
        std::cout << "\n--- OVF Generation Process Successful ---" << std::endl;
        std::cout << "Output file saved to: " << ovf_output_path << std::endl;
        
        // Verify the generated file by reading it back
        std::cout << "\n--- Verifying Generated OVF File ---" << std::endl;
        try {
            std::cout << "DEBUG: Attempting to read back the generated OVF file..." << std::endl;
            
            std::ifstream file(ovf_output_path, std::ios::binary);
            if (!file) {
                std::cerr << "ERROR: Could not open generated file for verification" << std::endl;
                return 1;
            }
            
            // Check file size
            file.seekg(0, std::ios::end);
            std::streampos file_size = file.tellg();
            file.seekg(0, std::ios::beg);
            
            std::cout << "DEBUG: Generated file size: " << file_size << " bytes" << std::endl;
            
            if (file_size < 16) {  // Should be much larger than this for a valid OVF file
                std::cerr << "ERROR: Generated file is too small to be a valid OVF file" << std::endl;
                return 1;
            }
            
            // Check magic bytes
            std::array<uint8_t, 4> magic_check;
            file.read(reinterpret_cast<char*>(magic_check.data()), 4);
            
            std::cout << "DEBUG: Magic bytes read: 0x" << std::hex;
            for (auto byte : magic_check) {
                std::cout << std::setfill('0') << std::setw(2) << static_cast<int>(byte);
            }
            std::cout << std::dec << std::endl;
            
            // Expected magic bytes for OVF: "LVF!" = {0x4c, 0x56, 0x46, 0x21}
            std::array<uint8_t, 4> expected_magic = {{0x4c, 0x56, 0x46, 0x21}};
            
            if (magic_check == expected_magic) {
                std::cout << "SUCCESS: OVF file has correct magic bytes!" << std::endl;
            } else {
                std::cerr << "ERROR: OVF file has incorrect magic bytes!" << std::endl;
                std::cerr << "Expected: LVF! (0x4c564621)" << std::endl;
                return 1;
            }
            
            file.close();
            std::cout << "SUCCESS: OVF file appears to be valid!" << std::endl;
            
        } catch (const std::exception& e) {
            std::cerr << "ERROR during file verification: " << e.what() << std::endl;
            return 1;
        }
    } else {
        std::cerr << "\n--- OVF Generation Process Failed ---" << std::endl;
        std::cerr << "Error: Failed to write OVF file." << std::endl;
        return 1;
    }

    std::cout << "DEBUG: Main function completed successfully" << std::endl;
    return 0;
}
