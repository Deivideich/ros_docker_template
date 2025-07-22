#!/bin/bash

# make_docker_generator_file.sh
# This script creates a portable docker_template_generator.sh that can recreate the entire /docker/ structure
# Uses template analysis to generate a robust and flexible generator

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_DIR="$SCRIPT_DIR/docker"
OUTPUT_FILE="$SCRIPT_DIR/docker_template_generator.sh"

echo "🔧 Creating Docker template generator from existing docker/ folder..."

# Check if docker directory exists
if [[ ! -d "$DOCKER_DIR" ]]; then
    echo "❌ Error: /docker/ directory not found!"
    exit 1
fi

# Analyze existing docker structure to extract template patterns
analyze_template_patterns() {
    echo "🔍 Analyzing template patterns in docker/ folder..."
    
    # Find all unique project names and registry names used
    local project_patterns=($(find "$DOCKER_DIR" -type f -exec grep -l "application_sim\|test_project" {} \; 2>/dev/null | head -5))
    local registry_patterns=($(find "$DOCKER_DIR" -type f -exec grep -l "roborregos" {} \; 2>/dev/null | head -5))
    
    echo "  📋 Found files with project patterns: ${#project_patterns[@]}"
    echo "  📋 Found files with registry patterns: ${#registry_patterns[@]}"
}

# Generate the template file with smart content embedding
generate_template_generator() {
    cat > "$OUTPUT_FILE" << 'GENERATOR_EOF'
#!/bin/bash

# Docker Template Generator
# Auto-generated script to recreate the Docker development environment
# This script is self-contained and portable

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_NAME="${1:-application_sim}"
DOCKER_REGISTRY="${2:-roborregos}"

echo "🚀 Generating Docker template for project: $PROJECT_NAME"
echo "📦 Using Docker registry: $DOCKER_REGISTRY"

# Create directory structure
create_directory_structure() {
    echo "📁 Creating directory structure..."
    mkdir -p "${PROJECT_NAME}/docker/compose"
    mkdir -p "${PROJECT_NAME}/docker/dockerfiles"
    mkdir -p "${PROJECT_NAME}/docker/env_files"
    mkdir -p "${PROJECT_NAME}/docker/scripts"
    echo "✅ Directory structure created"
}

# Start file creation functions

GENERATOR_EOF
}

# Function to escape content for heredoc
escape_for_heredoc() {
    local file="$1"
    # Read file content and escape it properly for heredoc
    # Use a unique EOF marker to avoid conflicts
    local eof_marker="FILE_CONTENT_EOF_$(date +%s)"
    sed "s/EOF/${eof_marker}_ESCAPED_EOF/g" "$file"
}

# Generate function for each file type
generate_file_function() {
    local file_path="$1"
    local relative_path="${file_path#$DOCKER_DIR/}"
    local function_name="create_$(echo "$relative_path" | sed 's/[^a-zA-Z0-9]/_/g')"
    local unique_eof="EOF_$(echo "$relative_path" | sed 's/[^a-zA-Z0-9]/_/g' | tr '[:lower:]' '[:upper:]')"
    
    # Handle dynamic file naming for script files and dockerfiles
    local output_path="$relative_path"
    if [[ "$relative_path" =~ scripts/.*\.sh$ ]]; then
        # Replace any known project names in filename with ${PROJECT_NAME}
        output_path=$(echo "$relative_path" | sed 's/test_project/${PROJECT_NAME}/g; s/application_sim/${PROJECT_NAME}/g')
    elif [[ "$relative_path" =~ dockerfiles/.*\.dockerfile$ ]]; then
        # Replace project names in dockerfile names
        output_path=$(echo "$relative_path" | sed 's/test_project/${PROJECT_NAME}/g; s/application_sim/${PROJECT_NAME}/g')
    fi
    
    echo "  📝 Generating function for: $relative_path -> $output_path"
    
    # Add function to generator
    cat >> "$OUTPUT_FILE" << FUNC_EOF

# Create $relative_path
$function_name() {
    cat > "\${PROJECT_NAME}/docker/$output_path" << '$unique_eof'
$(cat "$file_path")
$unique_eof
FUNC_EOF

    # Add variable replacement logic based on file type
    case "$relative_path" in
        *.yml|*.yaml)
            cat >> "$OUTPUT_FILE" << VAR_EOF
    
    # Replace template variables with actual values
    sed -i "s/application_sim/\$PROJECT_NAME/g" "\${PROJECT_NAME}/docker/$output_path"
    sed -i "s/test_project/\$PROJECT_NAME/g" "\${PROJECT_NAME}/docker/$output_path"
    sed -i "s/APPLICATION_SIM_DOCKERFILE/\${PROJECT_NAME^^}_DOCKERFILE/g" "\${PROJECT_NAME}/docker/$output_path"
    sed -i "s/APPLICATION_TEST_PROJECT_DOCKERFILE/\${PROJECT_NAME^^}_DOCKERFILE/g" "\${PROJECT_NAME}/docker/$output_path"
    # Replace service names that contain the project name
    sed -i "s/application_cuda/\${PROJECT_NAME}_cuda/g" "\${PROJECT_NAME}/docker/$output_path"
    sed -i "s/application_gpu/\${PROJECT_NAME}_gpu/g" "\${PROJECT_NAME}/docker/$output_path"  
    sed -i "s/application_cpu/\${PROJECT_NAME}_cpu/g" "\${PROJECT_NAME}/docker/$output_path"
    # Replace workspace paths
    sed -i "s/application_ws/\${PROJECT_NAME}_ws/g" "\${PROJECT_NAME}/docker/$output_path"
VAR_EOF
            ;;
        *.env)
            cat >> "$OUTPUT_FILE" << VAR_EOF
    
    # Replace template variables with actual values
    sed -i "s/application_sim/\$PROJECT_NAME/g" "\${PROJECT_NAME}/docker/$output_path"
    sed -i "s/test_project/\$PROJECT_NAME/g" "\${PROJECT_NAME}/docker/$output_path"
    sed -i "s/roborregos/\$DOCKER_REGISTRY/g" "\${PROJECT_NAME}/docker/$output_path"
    # Replace dockerfile environment variables
    sed -i "s/APPLICATION_SIM_DOCKERFILE/\${PROJECT_NAME^^}_DOCKERFILE/g" "\${PROJECT_NAME}/docker/$output_path"
    sed -i "s/APPLICATION_DOCKERFILE/APPLICATION_DOCKERFILE/g" "\${PROJECT_NAME}/docker/$output_path"
VAR_EOF
            ;;
        scripts/*.sh)
            cat >> "$OUTPUT_FILE" << VAR_EOF
    
    # Replace template variables with actual values
    sed -i "s/application_sim/\$PROJECT_NAME/g" "\${PROJECT_NAME}/docker/$output_path"
    sed -i "s/test_project/\$PROJECT_NAME/g" "\${PROJECT_NAME}/docker/$output_path"
    sed -i "s/APPLICATION_SIM/\${PROJECT_NAME^^}/g" "\${PROJECT_NAME}/docker/$output_path"
    sed -i "s/TEST_PROJECT/\${PROJECT_NAME^^}/g" "\${PROJECT_NAME}/docker/$output_path"
    # Replace function names and variables that contain the project name
    sed -i "s/get_application_sim_service/get_\${PROJECT_NAME}_service/g" "\${PROJECT_NAME}/docker/$output_path"
    sed -i "s/build_application_sim_image/build_\${PROJECT_NAME}_image/g" "\${PROJECT_NAME}/docker/$output_path"
    sed -i "s/application_sim_BASE_IMAGE/\${PROJECT_NAME}_BASE_IMAGE/g" "\${PROJECT_NAME}/docker/$output_path"
    sed -i "s/application_sim_BASE_IMAGE_TAG/\${PROJECT_NAME}_BASE_IMAGE_TAG/g" "\${PROJECT_NAME}/docker/$output_path"
    # Replace help text and comments
    sed -i "s/vsss_sim\\.sh/\${PROJECT_NAME}.sh/g" "\${PROJECT_NAME}/docker/$output_path"
    sed -i "s/VSSS/\${PROJECT_NAME^^}/g" "\${PROJECT_NAME}/docker/$output_path"
    # Replace workspace paths for all script files
    sed -i "s/application_ws/\${PROJECT_NAME}_ws/g" "\${PROJECT_NAME}/docker/$output_path"
VAR_EOF
            ;;
        dockerfiles/*.dockerfile)
            # Handle dockerfile naming
            if [[ "$relative_path" =~ (test_project|application_sim)\.dockerfile$ ]]; then
                cat >> "$OUTPUT_FILE" << VAR_EOF
    
    # Replace template variables with actual values  
    sed -i "s/application_sim/\$PROJECT_NAME/g" "\${PROJECT_NAME}/docker/$output_path"
    sed -i "s/test_project/\$PROJECT_NAME/g" "\${PROJECT_NAME}/docker/$output_path"
    # Replace workspace paths in dockerfiles
    sed -i "s/application_ws/\${PROJECT_NAME}_ws/g" "\${PROJECT_NAME}/docker/$output_path"
VAR_EOF
            fi
            ;;
    esac
    
    cat >> "$OUTPUT_FILE" << 'FUNC_EOF'
}
FUNC_EOF
    
    # Store function name for main execution
    echo "$function_name" >> "/tmp/generator_functions.txt"
}

# Process all files and generate the template
process_docker_files() {
    echo "📋 Processing files in docker/ directory..."
    
    # Initialize function list
    echo "" > "/tmp/generator_functions.txt"
    
    # Process files in a specific order for better organization
    local file_order=(
        "scripts"
        "compose" 
        "dockerfiles"
        "env_files"
    )
    
    for dir in "${file_order[@]}"; do
        if [[ -d "$DOCKER_DIR/$dir" ]]; then
            echo "  📂 Processing directory: $dir/"
            find "$DOCKER_DIR/$dir" -type f | sort | while read -r file; do
                generate_file_function "$file"
            done
        fi
    done
    
    # Process any remaining files not in the standard directories
    find "$DOCKER_DIR" -maxdepth 1 -type f | sort | while read -r file; do
        generate_file_function "$file"
    done
}

# Add main execution function
add_main_function() {
    cat >> "$OUTPUT_FILE" << 'MAIN_EOF'

# Main execution function
main() {
    echo "🎯 Starting Docker template generation..."
    
    # Create base directory structure
    create_directory_structure
    
    # Create all files
MAIN_EOF

    # Add all function calls
    while IFS= read -r func; do
        if [[ -n "$func" ]]; then
            echo "    $func" >> "$OUTPUT_FILE"
        fi
    done < "/tmp/generator_functions.txt"

    cat >> "$OUTPUT_FILE" << 'MAIN_EOF'
    
    # Make scripts executable
    chmod +x "${PROJECT_NAME}/docker/scripts/"*.sh
    
    echo "✅ Docker template generated successfully!"
    echo ""
    echo "📋 Next steps:"
    echo "1. Navigate to the project directory: cd ${PROJECT_NAME}"
    echo "2. Navigate to the docker/compose directory: cd docker/compose"
    echo "3. Review and customize the .env file if needed"
    echo "4. Build and run your application:"
    echo "   ./docker/scripts/${PROJECT_NAME}.sh -deploy --gpu"
    echo ""
    echo "🔧 Available commands:"
    echo "   ./docker/scripts/${PROJECT_NAME}.sh -help"
}

# Script usage
show_usage() {
    echo "Usage: $0 [PROJECT_NAME] [DOCKER_REGISTRY]"
    echo ""
    echo "Arguments:"
    echo "  PROJECT_NAME     Name of your project (default: application_sim)"
    echo "  DOCKER_REGISTRY  Docker registry to use (default: roborregos)"
    echo ""
    echo "Examples:"
    echo "  $0                                    # Use defaults"
    echo "  $0 my_robot_project                   # Custom project name"
    echo "  $0 my_robot_project my_registry       # Custom project and registry"
}

# Parse command line arguments
case "$1" in
    -h|--help|help)
        show_usage
        exit 0
        ;;
    *)
        main
        ;;
esac
MAIN_EOF
}

# Main execution
main() {
    analyze_template_patterns
    generate_template_generator
    process_docker_files
    add_main_function
    
    # Make the generator executable
    chmod +x "$OUTPUT_FILE"
    
    # Clean up temporary files
    rm -f "/tmp/generator_functions.txt"
    
    echo "✅ Docker template generator created successfully!"
    echo "📄 Generator saved as: $OUTPUT_FILE"
    echo ""
    echo "🎯 To use the generator:"
    echo "1. Copy docker_template_generator.sh to any new project directory"
    echo "2. Run: ./docker_template_generator.sh [project_name] [docker_registry]"
    echo "3. The entire Docker structure will be recreated"
    echo ""
    echo "📋 Example usage:"
    echo "  ./docker_template_generator.sh my_new_project my_registry"
    echo ""
    echo "🔧 Testing the generated template..."
    
    # Quick validation test
    if [[ -x "$OUTPUT_FILE" ]]; then
        echo "✅ Generator script is executable"
        
        # Test syntax
        if bash -n "$OUTPUT_FILE" 2>/dev/null; then
            echo "✅ Generator script syntax is valid"
        else
            echo "⚠️  Warning: Generator script has syntax issues"
        fi
    else
        echo "❌ Error: Generator script is not executable"
    fi
}

# Run the script
main
