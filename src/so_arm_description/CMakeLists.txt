cmake_minimum_required(VERSION 3.10.2)

project(so_arm_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

# Install the mesh/model files
install(
	DIRECTORY
		models/
	DESTINATION
		share/${PROJECT_NAME}/models
	FILES_MATCHING PATTERN "*.stl"
)

# Install URDF files
install(
	DIRECTORY
		urdf/
	DESTINATION
		share/${PROJECT_NAME}/urdf
	FILES_MATCHING PATTERN "*.urdf"
)

# Install config directory
install(
	DIRECTORY
		config/
	DESTINATION
		share/${PROJECT_NAME}/config
	FILES_MATCHING PATTERN "*.yaml"
)



# Install joint properties XML
install(
	FILES
		joint_properties.xml
	DESTINATION
		share/${PROJECT_NAME}
)

ament_package()