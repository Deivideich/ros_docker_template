#!/bin/bash

# make_docker_generator_file.sh
# This script creates a portable docker_template_generator.sh that can recreate the entire /docker/ structure

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_DIR="$SCRIPT_DIR/docker"
OUTPUT_FILE="$SCRIPT_DIR/docker_template_generator.sh"

echo "🔧 Creating Docker template generator..."

# Check if docker directory exists
if [[ ! -d "$DOCKER_DIR" ]]; then
    echo "❌ Error: /docker/ directory not found!"
    exit 1
fi

# Start creating the generator script
cat > "$OUTPUT_FILE" << 'EOF'
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
    mkdir -p docker/{compose,dockerfiles,env_files,scripts}
    echo "✅ Directory structure created"
}

# Function to create files with content
create_file_with_content() {
    local file_path="$1"
    local content="$2"
    
    echo "📝 Creating $file_path..."
    mkdir -p "$(dirname "$file_path")"
    echo "$content" > "$file_path"
}

# Start file creation functions
EOF

# Function to escape content for heredoc
escape_content() {
    local file="$1"
    # Read file and escape special characters for heredoc
    sed 's/\\/\\\\/g; s/`/\\`/g; s/\$/\\$/g' "$file"
}

# Function to add file creation to generator
add_file_to_generator() {
    local relative_path="$1"
    local full_path="$2"
    local function_name="create_$(echo "$relative_path" | sed 's/[^a-zA-Z0-9]/_/g')"
    
    echo "" >> "$OUTPUT_FILE"
    echo "# Create $relative_path" >> "$OUTPUT_FILE"
    echo "$function_name() {" >> "$OUTPUT_FILE"
    echo "    local content=\$'$(escape_content "$full_path")'" >> "$OUTPUT_FILE"
    echo "    # Replace template variables with actual values" >> "$OUTPUT_FILE"
    echo "    content=\${content//application_sim/\$PROJECT_NAME}" >> "$OUTPUT_FILE"
    echo "    content=\${content//roborregos/\$DOCKER_REGISTRY}" >> "$OUTPUT_FILE"
    echo "    create_file_with_content \"docker/$relative_path\" \"\$content\"" >> "$OUTPUT_FILE"
    echo "}" >> "$OUTPUT_FILE"
    
    # Add function call to main execution
    echo "$function_name" >> "/tmp/generator_calls.txt"
}

# Initialize calls file
echo "" > "/tmp/generator_calls.txt"

# Process all files in docker directory
echo "📋 Processing files in /docker/ directory..."

find "$DOCKER_DIR" -type f | while read -r file; do
    # Get relative path from docker directory
    relative_path="${file#$DOCKER_DIR/}"
    echo "  Processing: $relative_path"
    add_file_to_generator "$relative_path" "$file"
done

# Add main execution function to generator
cat >> "$OUTPUT_FILE" << 'EOF'

# Main execution function
main() {
    echo "🎯 Starting Docker template generation..."
    
    # Create base directory structure
    create_directory_structure
    
    # Create all files
EOF

# Add all function calls
while read -r call; do
    if [[ -n "$call" ]]; then
        echo "    $call" >> "$OUTPUT_FILE"
    fi
done < "/tmp/generator_calls.txt"

cat >> "$OUTPUT_FILE" << 'EOF'
    
    # Make scripts executable
    chmod +x docker/scripts/*.sh
    
    echo "✅ Docker template generated successfully!"
    echo ""
    echo "📋 Next steps:"
    echo "1. Navigate to the docker/compose directory: cd docker/compose"
    echo "2. Review and customize the .env file if needed"
    echo "3. Build and run your application:"
    echo "   ./docker/scripts/application_sim.sh -deploy --gpu"
    echo ""
    echo "🔧 Available commands:"
    echo "   ./docker/scripts/application_sim.sh -help"
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
EOF

# Make the generator executable
chmod +x "$OUTPUT_FILE"

# Clean up temporary file
rm -f "/tmp/generator_calls.txt"

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
